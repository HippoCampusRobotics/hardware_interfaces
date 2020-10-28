#pragma once
#include <arducam_mipicamera.h>

#include <opencv2/opencv.hpp>

#define VCOS_ALIGN_DOWN(p, n) (((ptrdiff_t)(p)) & ~((n)-1))
#define VCOS_ALIGN_UP(p, n) VCOS_ALIGN_DOWN((ptrdiff_t)(p) + (n)-1, (n))

class CameraOV9281 {
 private:
  CAMERA_INSTANCE camera_instance_;
  BUFFER *buffer_;
  cv::Mat *image_;
  int capture_timeout_;
  struct format fmt_;
  IMAGE_FORMAT image_fmt_ = {IMAGE_ENCODING_RAW_BAYER, 0};

 public:
  CameraOV9281(int mode = 0, int exposure = 100, int gain = 15) {
    int res = arducam_init_camera(&camera_instance_);
    if (!res) {
      printf("Camera successfully initialized\n");
    } else {
      printf("Failed to ini camera\n");
    }
    capture_timeout_ = 1000;
    buffer_ = NULL;
    SetMode(mode);
    GetFormat(&fmt_);
    SetExposure(exposure);
    SetGain(gain);
  }
  ~CameraOV9281() {
    arducam_release_buffer(buffer_);
    arducam_close_camera(camera_instance_);
  }
  int SetMode(int mode);
  int GetFormat(struct format *fmt);
  int GetWidth();
  int GetHeight();
  int SetExposure(int exposure);
  int GetExposure(int *exposure);
  int SetGain(int gain);
  int GetGain(int *gain);
  void Capture();
  cv::Mat *GetImage();
};
