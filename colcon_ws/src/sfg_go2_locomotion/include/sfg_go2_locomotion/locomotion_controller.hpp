#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace sfg_go2_locomotion
{
    class LocomotionController : public rclcpp::Node
    {
    public:
        LocomotionController(const rclcpp::NodeOptions &options);

    private:
        void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
        void stop_move_timer_callback();
        void sport_mode_state_callback(const void *msg);

        // ROS parameters
        std::string m_network_interface;
        float m_client_timeout;
        float m_command_timeout;

        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_velocity_subscriber;
        rclcpp::TimerBase::SharedPtr m_stop_move_timer;

        std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> m_sport_mode_state_subscriber;
        std::shared_ptr<unitree::robot::go2::SportClient> m_sport_client;
        std::optional<unitree_go::msg::dds_::SportModeState_> m_sport_mode_state;
    };
}