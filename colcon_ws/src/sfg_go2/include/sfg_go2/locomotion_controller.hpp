#pragma once

#include <rclcpp/rclcpp.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace sfg_go2
{
    class LocomotionController : public rclcpp::Node
    {
    public:
        LocomotionController(const rclcpp::NodeOptions &options);

    private:
        // ROS parameters
        std::string m_network_interface;

        std::shared_ptr<unitree::robot::go2::SportClient> m_sport_client;
    };
}