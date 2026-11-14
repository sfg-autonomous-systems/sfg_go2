#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include "sfg_hardware_interface/locomotion_controller_base.hpp"

namespace sfg_go2_hardware_interface
{
    class LocomotionController : public sfg_hardware_interface::LocomotionControllerBase
    {
    public:
        LocomotionController(const rclcpp::NodeOptions &options);

    protected:
        void arm() override;
        void disarm() override;
        void emergency_stop() override;
        void apply_cmd(const geometry_msgs::msg::TwistStamped &cmd) override;

    private:
        void sport_mode_state_callback(const void *msg);

        // ROS parameters
        std::string m_network_interface;
        float m_client_timeout;

        unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> m_sport_mode_state_subscriber;
        std::unique_ptr<unitree::robot::go2::SportClient> m_sport_client;
        std::optional<unitree_go::msg::dds_::SportModeState_> m_last_sport_mode_state;
        std::mutex m_last_sport_mode_state_mutex;
    };
}