#include "usb_cam/UsbCam.h"



int main(int argc, char **argv) {
  // 初始化ROS
  ros::init(argc, argv, "usb_cam");
  ros::NodeHandle nh;

  UsbCam usbCam(nh);

  usbCam.init_node();

  usbCam.run();

  return 0;
}