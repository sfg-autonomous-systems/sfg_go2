#include "sfg_go2_trajectory_controller/trajectory_controller.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sfg_go2_trajectory_controller::TrajectoryController>(
        rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
