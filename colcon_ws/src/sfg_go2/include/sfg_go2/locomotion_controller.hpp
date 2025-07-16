#pragma once

#include <rclcpp/rclcpp.hpp>

namespace sfg_go2
{
    class LocomotionController : public rclcpp::Node
    {
    public:
        LocomotionController(const rclcpp::NodeOptions &options);
    };
}