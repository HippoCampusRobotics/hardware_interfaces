#include <arpa/inet.h>
#include <errno.h>
#include <fav_msgs/ThrusterSetpoint.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <thread>

#include "cobs.hpp"
#include "packet.hpp"

static constexpr double kInputScaling = 0.4;
static constexpr uint16_t kPwmMax = 1900;
static constexpr uint16_t kPwmMin = 1100;
static constexpr int kNumberMotors = 8;
static constexpr int kPwmPayloadSize = sizeof(uint16_t) * kNumberMotors;
static constexpr int kPwmPayloadOffset = 1;
static constexpr int kPwmPacketSize =
    1 + kPwmPayloadSize + sizeof(uint32_t) + 1;
static constexpr int kPwmCrcOffset = 1 + kPwmPayloadSize;

class EscCommanderNode {
 public:
  EscCommanderNode(ros::NodeHandle *_ros_node) {
    ros_node_ = _ros_node;
    setpoint_topic_ = "thruster_setpoint";
    if (!ros::param::get("~serial_port", port_name_)) {
      port_name_ = "/dev/teensy";
    }
    ROS_INFO("Using port '%s'", port_name_.c_str());
  }

  void Run() {
    if (!InitSerial(port_name_)) {
      ros::shutdown();
      return;
    }
    arming_service_ = ros_node_->advertiseService(
        "arm", &EscCommanderNode::OnServeArming, this);
    watchdog_ = ros_node_->createTimer(
        ros::Duration(0.3),
        boost::bind(&EscCommanderNode::OnWatchdogTimeout, this, _1));
    setpoint_sub_ = ros_node_->subscribe<fav_msgs::ThrusterSetpoint>(
        setpoint_topic_, 1,
        boost::bind(&EscCommanderNode::OnThrusterSetpoint, this, _1));
    voltage_pub_ =
        ros_node_->advertise<std_msgs::Float64>("battery_voltage", 1);

    std::thread serial_thread(&EscCommanderNode::SerialThread, this);
    ros::spin();
  }

 private:
  union Voltage {
    float value;
    uint8_t bytes[sizeof(float)];
  };
  bool OnServeArming(std_srvs::SetBool::Request &_request,
                     std_srvs::SetBool::Response &_response) {
    armed_ = _request.data;
    _response.success = true;
    return true;
  }
  void PublishVoltage() {
    Voltage voltage;
    if (voltage_packet_.PayloadSize() != sizeof(voltage.bytes)) {
      ROS_WARN("Voltage Payload has wrong size. Has %d but expected %lu.",
               voltage_packet_.PayloadSize(), sizeof(voltage.bytes));
      return;
    }
    const uint8_t *data = voltage_packet_.Payload();
    for (int i = 0; i < voltage_packet_.PayloadSize(); ++i) {
      voltage.bytes[i] = data[i];
    }
    std_msgs::Float64 msg;
    msg.data = voltage.value;
    voltage_pub_.publish(msg);
  }
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
    _input *= kInputScaling;
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
    if (armed_) {
      WriteThrustSetpoint(_msg->data);
    }
  }

  bool WriteThrustSetpoint(
      const boost::array<double, kNumberMotors> &_setpoint) {
    pwm_packet_.Reset();
    uint8_t buffer[kNumberMotors * sizeof(uint16_t)];
    uint8_t *write_pointer = buffer;
    for (int i = 0; i < kNumberMotors; ++i) {
      uint16_t pwm = SetpointToPwm(_setpoint[i]);
      *write_pointer++ = (uint8_t)(pwm >> 8);
      *write_pointer++ = (uint8_t)(pwm & 0xFF);
    }
    pwm_packet_.SetPayload(buffer, sizeof(buffer));
    pwm_packet_.Packetize();
    int bytes_written =
        write(serial_port_, pwm_packet_.Data(), pwm_packet_.Size());
    bool success = bytes_written == kPwmPacketSize;
    ROS_ERROR_COND(!success,
                   "Could not write all data to serial port. Written %d of %d.",
                   bytes_written, kPwmPacketSize);
    return success;
  }

  void SerialThread() {
    auto rate = ros::Rate(ros::Duration(0.1));
    while (ros::ok()) {
      rate.sleep();
      int available = 0;
      if (ioctl(serial_port_, FIONREAD, &available) < 0) {
        continue;
      };
      for (int i = 0; i < available; ++i) {
        uint8_t byte;
        int length = read(serial_port_, &byte, 1);
        if (length) {
          if (!voltage_packet_.AddByte(byte)) {
            voltage_packet_.Reset();
            ROS_WARN("Receive buffer full before packet was complete.");
          } else if (voltage_packet_.Complete()) {
            if (!voltage_packet_.Decode()) {
              ROS_WARN("Failed to decode packet.");
            } else if (!voltage_packet_.CrcOk()) {
              ROS_WARN("CRC failed.");
            } else {
              PublishVoltage();
            }
            voltage_packet_.Reset();
          }
        }
      }
    }
  }
  ros::NodeHandle *ros_node_;
  ros::Subscriber setpoint_sub_;
  ros::Publisher voltage_pub_;
  ros::Timer watchdog_;
  ros::ServiceServer arming_service_;
  std::atomic<bool> armed_{false};
  std::string setpoint_topic_;
  bool timed_out_{false};
  int serial_port_;
  std::string port_name_;
  struct termios tty_;
  Voltage voltage;
  Packet pwm_packet_;
  Packet voltage_packet_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "esc_commander");
  ros::NodeHandle ros_node;
  EscCommanderNode node(&ros_node);
  node.Run();
}
