#include "go2/rgbd_camera.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RGBDCamera>());
  rclcpp::shutdown();
  return 0;
}