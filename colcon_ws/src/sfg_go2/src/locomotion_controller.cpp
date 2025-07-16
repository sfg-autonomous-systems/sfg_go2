#include "sfg_go2/locomotion_controller.hpp"

namespace sfg_go2
{
    LocomotionController::LocomotionController(const rclcpp::NodeOptions &options) : Node("locomotion_controller", options)
    {
        RCLCPP_INFO(get_logger(), "Started locomotion controller.");
    }
}