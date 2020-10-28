#include "camera_ov9281.h"

#include <linux/v4l2-controls.h>

int CameraOV9281::SetMode(int mode) {
  int res = arducam_set_mode(camera_instance_, mode);
  GetFormat(&fmt_);
  return res;
}

int CameraOV9281::GetFormat(struct format *fmt) {
  return arducam_get_format(camera_instance_, fmt);
}

int CameraOV9281::GetWidth() { return fmt_.width; }

int CameraOV9281::GetHeight() { return fmt_.height; }

int CameraOV9281::SetExposure(int exposure) {
  return arducam_set_control(camera_instance_, V4L2_CID_EXPOSURE, exposure);
}

int CameraOV9281::GetExposure(int *exposure) {
  return arducam_get_control(camera_instance_, V4L2_CID_EXPOSURE, exposure);
}

int CameraOV9281::SetGain(int gain) {
  return arducam_set_control(camera_instance_, V4L2_CID_GAIN, gain);
}

int CameraOV9281::GetGain(int *gain) {
  return arducam_get_control(camera_instance_, V4L2_CID_GAIN, gain);
}

void CameraOV9281::Capture() {
  // make sure the buffer from last capture call is released.
  if (buffer_) {
    arducam_release_buffer(buffer_);
  }
  buffer_ = arducam_capture(camera_instance_, &image_fmt_, capture_timeout_);
}

cv::Mat *CameraOV9281::GetImage() {
  // check if buffer contains data
  if (!buffer_) {
    printf("Buffer not valid!\n");
    return NULL;
  }
  int height, width;
  height = VCOS_ALIGN_UP(fmt_.height, 16);
  width = VCOS_ALIGN_UP(fmt_.width, 32);
  image_ = new cv::Mat(height, width, CV_8UC1, buffer_->data);
  return image_;
}
