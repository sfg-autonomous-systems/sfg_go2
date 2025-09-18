#pragma once

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include "sfg_agent_msgs/srv/trigger_action.hpp"

namespace sfg_go2_hardware_interface
{
    class LocomotionController : public rclcpp::Node
    {
    public:
        LocomotionController(const rclcpp::NodeOptions &options);

    private:
        void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg);
        void apply_move_callback();
        void set_state_callback(
            const std::shared_ptr<sfg_agent_msgs::srv::TriggerAction::Request> request,
            std::shared_ptr<sfg_agent_msgs::srv::TriggerAction::Response> response);
        void sport_mode_state_callback(const void *msg);

        // ROS parameters
        std::string m_network_interface;
        float m_client_timeout;
        float m_command_timeout;
        Eigen::Matrix<float, 3, 2> m_command_speed_limits;

        static const std::map<std::string, std::function<int32_t(unitree::robot::go2::SportClient *)>> s_state_map;

        unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> m_sport_mode_state_subscriber;
        std::shared_ptr<unitree::robot::go2::SportClient> m_sport_client;
        std::optional<unitree_go::msg::dds_::SportModeState_> m_last_sport_mode_state;
        std::mutex m_last_sport_mode_state_mutex;

        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_cmd_vel_subscriber;
        geometry_msgs::msg::TwistStamped m_last_cmd_vel;
        rclcpp::TimerBase::SharedPtr m_apply_move_timer;
        rclcpp::Service<sfg_agent_msgs::srv::TriggerAction>::SharedPtr m_set_state_service;
    };
}