#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <hardware_interfaces/CameraConfig.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camera_ov9281.h"

class CameraNode {
  ros::NodeHandle nh_;
  image_transport::CameraPublisher image_pub_;
  sensor_msgs::Image image_msg_;
  cv_bridge::CvImage bridge_image_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_;
  std::string camera_name_;
  std::string camera_info_url_;
  double framerate_;

  dynamic_reconfigure::Server<hardware_interfaces::CameraConfig> server_;
  dynamic_reconfigure::Server<hardware_interfaces::CameraConfig>::CallbackType
      f_;

  CameraOV9281 camera_;

 public:
  CameraNode() : nh_("~") {
    image_transport::ImageTransport it(nh_);
    image_pub_ = it.advertiseCamera("image_raw", 1);

    // read parameters
    nh_.param<std::string>("camera_name", camera_name_, "camera");
    nh_.param<std::string>("camera_frame_id", bridge_image_.header.frame_id,
                           "camera_frame");
    nh_.param<std::string>("camera_info_url", camera_info_url_, "");
    nh_.param<double>("framerate", framerate_, 10.0);

    // reset camera info manager according to params.
    info_manager_.reset(new camera_info_manager::CameraInfoManager(
        nh_, camera_name_, camera_info_url_));

    if (!info_manager_->isCalibrated()) {
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = bridge_image_.header.frame_id;
      camera_info.width = camera_.GetWidth();
      camera_info.height = camera_.GetHeight();
      info_manager_->setCameraInfo(camera_info);
    }

    f_ = boost::bind(&CameraNode::ReconfigureCallback, this, _1, _2);
    server_.setCallback(f_);
  }

  void ReconfigureCallback(hardware_interfaces::CameraConfig &config,
                           uint32_t level) {
    if (camera_.SetExposure(config.Exposure)) {
      ROS_ERROR("Could not set exposure to %d.", config.Exposure);
    }
    if (camera_.SetGain(config.Gain)) {
      ROS_ERROR("Could not set gain to %d.", config.Gain);
    }
    camera_.GetExposure(&config.Exposure);
    camera_.GetGain(&config.Gain);
  }

  bool CaptureAndPublish() {
    cv::Mat *cv_image;
    camera_.Capture();
    cv_image = camera_.GetImage();

    bridge_image_.encoding = "mono8";
    bridge_image_.image = *cv_image;
    bridge_image_.toImageMsg(image_msg_);
    sensor_msgs::CameraInfoPtr info(
        new sensor_msgs::CameraInfo(info_manager_->getCameraInfo()));
    info->header.stamp = ros::Time::now();
    image_pub_.publish(image_msg_, *info);
    return true;
  }

  void Run() {
    ros::Rate loop_rate(framerate_);

    while (nh_.ok()) {
      CaptureAndPublish();
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera");
  ROS_INFO("Creating node...");
  CameraNode node;
  ROS_INFO("Calling Run()...");
  node.Run();
  return 0;
}
