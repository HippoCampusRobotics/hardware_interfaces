// install the arducam library by downloading 
// https://github.com/ArduCAM/MIPI_Camera/raw/master/RPI/lib/libarducam_mipicamera.so
// and install it by executing 
// 'sudo install -m 644 libarducam_mipicamera.so /usr/lib/'
#include <arducam_mipicamera.h>
#include <linux/v4l2-controls.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int main(int argc, char **argv) {
  CAMERA_INSTANCE camera_instance;
  struct format fmt;
  struct camera_ctrl ctrl;
  int index = 0;
  char pix_format[5];
  pix_format[4] = '\0';

  arducam_init_camera(&camera_instance);

  usleep(1000 * 1000 * 2);

  printf("Supported camera modes:\n");
  while (!arducam_get_support_formats(camera_instance, &fmt, index++)) {
    strncpy(pix_format, (char *)&fmt.pixelformat, 4);
    printf("mode: %d, width: %d, height: %d, pixelformat: %s, desc: %s\n",
           fmt.mode, fmt.width, fmt.height, pix_format, fmt.description);
  }

  index = 0;
  printf("Supported controls:\n");
  while (!arducam_get_support_controls(camera_instance, &ctrl, index++)) {
    int value = 0;
    arducam_get_control(camera_instance, ctrl.id, &value);
    printf(
        "index: %d, CID: 0x%08x, desc: %s, min: %d, max: %d, default: %d, "
        "current: %d\n",
        index - 1, ctrl.id, ctrl.desc, ctrl.min_value, ctrl.max_value,
        ctrl.default_value, value);
  }

  arducam_close_camera(camera_instance);

  return 0;
}
