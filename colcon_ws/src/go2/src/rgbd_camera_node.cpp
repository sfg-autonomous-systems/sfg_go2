#include "go2/rgbd_camera.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("rgbd_camera");
  auto rgbd_camera = std::make_shared<RGBDCamera>(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}