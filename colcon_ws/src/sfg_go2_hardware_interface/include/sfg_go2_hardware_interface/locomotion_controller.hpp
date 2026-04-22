#pragma once

#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include "sfg_hardware_interface/locomotion_controller_base.hpp"

namespace sfg_go2_hardware_interface
{
    class LocomotionController : public sfg_hardware_interface::LocomotionControllerBase
    {
    public:
        explicit LocomotionController(const rclcpp::NodeOptions &options);

    protected:
        void arm() override;
        void disarm() override;
        void emergency_stop() override;
        void apply_cmd(const geometry_msgs::msg::TwistStamped &cmd) override;

    private:
        enum class ControlMode
        {
            SPORT,
            POLICY_LOW_LEVEL,
            IDLE
        };

        void sport_mode_state_callback(const void *msg);
        void policy_lowcmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void control_mode_callback(const std_msgs::msg::String::SharedPtr msg);

        static std::string control_mode_to_string(ControlMode mode);
        static ControlMode control_mode_from_string(const std::string &mode);

        // ROS parameters
        std::string m_network_interface;
        float m_client_timeout;
        std::string m_initial_control_mode;

        unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> m_sport_mode_state_subscriber;
        std::unique_ptr<unitree::robot::go2::SportClient> m_sport_client;
        std::optional<unitree_go::msg::dds_::SportModeState_> m_last_sport_mode_state;
        std::mutex m_last_sport_mode_state_mutex;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_policy_lowcmd_subscriber;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_control_mode_subscriber;

        ControlMode m_control_mode{ControlMode::SPORT};
        std::mutex m_control_mode_mutex;
    };
}