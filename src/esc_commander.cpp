#include <arpa/inet.h>
#include <errno.h>
#include <fav_msgs/ThrusterSetpoint.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <boost/crc.hpp>

static constexpr uint16_t kPwmMax = 1900;
static constexpr uint16_t kPwmMin = 1100;
static constexpr int kNumberMotors = 8;
static constexpr int kPwmPayloadSize = sizeof(uint16_t) * kNumberMotors;
static constexpr int kPwmPayloadOffset = 1;
static constexpr int kPwmPacketSize =
    1 + kPwmPayloadSize + sizeof(uint32_t) + 1;
static constexpr int kPwmCrcOffset = 1 + kPwmPayloadSize;

/**
 * @brief
 *
 * @param _buffer
 * @param _length Total length including COBS byte at the start and the
 * delimiter byte at the end.
 */
void cobs_encode(uint8_t *_buffer, int _length) {
  _buffer[0] = 0;
  _buffer[_length - 1] = 0;
  uint8_t offset = 0;
  for (int i = 1; i < _length; ++i) {
    ++offset;
    if (_buffer[i] == 0) {
      _buffer[i - offset] = offset;
      offset = 0;
    }
  }
}
/**
 * @brief
 *
 * @param _buffer
 * @param _length Total length including COBY byte at the start and the
 * delimiter byte at the end.
 * @return uint8_t* Start of the decoded data.
 */
uint8_t *cobs_decode(uint8_t *_buffer, int _length) {
  if (!_buffer) {
    return nullptr;
  }
  int i = 0;
  int next_zero = _buffer[0];
  while (true) {
    i += next_zero;
    if (i >= _length) {
      return nullptr;
    }
    next_zero = _buffer[i];
    // delimiter found
    if (next_zero == 0) {
      return &_buffer[1];
    }
    _buffer[i] = 0;
  }
}

class EscCommanderNode {
 public:
  EscCommanderNode(ros::NodeHandle *_ros_node) {
    ros_node_ = _ros_node;
    setpoint_topic_ = "thruster_setpoint";
    if (!ros::param::get("~serial_port", port_name_)) {
      port_name_ = "/dev/teensy";
    }
  }

  void Run() {
    if (!InitSerial(port_name_)) {
      ros::shutdown();
      return;
    }
    watchdog_ = ros_node_->createTimer(
        ros::Duration(0.3),
        boost::bind(&EscCommanderNode::OnWatchdogTimeout, this, _1));
    setpoint_sub_ = ros_node_->subscribe<fav_msgs::ThrusterSetpoint>(
        setpoint_topic_, 1,
        boost::bind(&EscCommanderNode::OnThrusterSetpoint, this, _1));

    ros::spin();
  }

 private:
  bool InitSerial(std::string _port_name) {
    serial_port_ = open(_port_name.c_str(), O_RDWR);
    tty_.c_cflag &= ~PARENB;  // no parity
    tty_.c_cflag &= ~CSTOPB;  // 1 stop bit
    tty_.c_cflag &= ~CSIZE;
    tty_.c_cflag |= CS8;       // 8 bit mode
    tty_.c_cflag &= ~CRTSCTS;  // disable hardware flow control
    tty_.c_cflag |= CREAD | CLOCAL;
    tty_.c_lflag &= ~ICANON;
    tty_.c_lflag &= ~ECHO;    // Disable echo
    tty_.c_lflag &= ~ECHOE;   // Disable erasure
    tty_.c_lflag &= ~ECHONL;  // Disable new-line echo
    tty_.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP
    tty_.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
    tty_.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                      ICRNL);  // Disable any special handling of received bytes
    tty_.c_oflag &= ~OPOST;    // Prevent special interpretation of output bytes
                               // (e.g. newline chars)
    tty_.c_oflag &=
        ~ONLCR;  // Prevent conversion of newline to carriage return/line feed
    tty_.c_cc[VTIME] = 10;  // Wait for up to 1s (10 deciseconds), returning as
                            // soon as any data is received.
    tty_.c_cc[VMIN] = 0;

    // baud rate
    cfsetispeed(&tty_, B115200);
    cfsetospeed(&tty_, B115200);

    if (tcsetattr(serial_port_, TCSANOW, &tty_) != 0) {
      ROS_FATAL("Could not apply serial port settings!");
      return false;
    }
    return true;
  }

  inline uint16_t SetpointToPwm(double _input) {
    if (_input > 1.0) {
      _input = 1.0;
    } else if (_input < -1.0) {
      _input = -1.0;
    }
    return 0.5 * (kPwmMax + kPwmMin) + _input * 0.5 * (kPwmMax - kPwmMin);
  }

  void OnWatchdogTimeout(const ros::TimerEvent &) {
    ROS_WARN_COND(!timed_out_,
                  "'%s' timed out. Sending zero thrust until new "
                  "messages arrive.",
                  setpoint_topic_.c_str());
    timed_out_ = true;
    // DO SOMETHING
  }
  void OnThrusterSetpoint(const fav_msgs::ThrusterSetpoint::ConstPtr &_msg) {
    ROS_INFO_COND(timed_out_, "Received '%s'", setpoint_topic_.c_str());
    timed_out_ = false;
    watchdog_.stop();
    watchdog_ = ros_node_->createTimer(
        ros::Duration(0.3),
        boost::bind(&EscCommanderNode::OnWatchdogTimeout, this, _1));
    WriteThrustSetpoint(_msg->data);
  }

  bool WriteThrustSetpoint(
      const boost::array<double, kNumberMotors> &_setpoint) {
    uint8_t buffer[kPwmPacketSize];
    uint8_t *payload = buffer + kPwmPayloadOffset;
    uint8_t *crc = buffer + kPwmCrcOffset;
    for (int i = 0; i < kNumberMotors; ++i) {
      uint16_t pwm = SetpointToPwm(_setpoint[i]);
      payload[i * sizeof(uint16_t)] = (uint8_t)(pwm >> 8);
      payload[i * sizeof(uint16_t) + 1] = (uint8_t)(pwm & 0xFF);
    }
    boost::crc_32_type result;
    result.process_bytes(payload, kPwmPayloadSize);
    crc[0] = (uint8_t)(result.checksum() >> 24);
    crc[1] = (uint8_t)((result.checksum() >> 16) & 0xFF);
    crc[2] = (uint8_t)((result.checksum() >> 8) & 0xFF);
    crc[3] = (uint8_t)(result.checksum() & 0xFF);
    cobs_encode(buffer, kPwmPacketSize);
    int bytes_written = write(serial_port_, buffer, kPwmPacketSize);
    bool success = bytes_written == kPwmPacketSize;
    ROS_ERROR_COND(!success,
                   "Could not write all data to serial port. Written %d of %d.",
                   bytes_written, kPwmPacketSize);
    return success;
  }
  ros::NodeHandle *ros_node_;
  ros::Subscriber setpoint_sub_;
  ros::Timer watchdog_;
  std::string setpoint_topic_;
  bool timed_out_{false};
  int serial_port_;
  std::string port_name_;
  struct termios tty_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "esc_commander");
  ros::NodeHandle ros_node;
  EscCommanderNode node(&ros_node);
  node.Run();
}
