#pragma once

#include <mutex>
#include <tuple>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#include <unitree/robot/channel/channel_subscriber.hpp>

namespace sfg_go2_hardware_interface
{
    class RobotStateBridge : public rclcpp::Node
    {
    public:
        RobotStateBridge(const rclcpp::NodeOptions &options);

    private:
        void low_state_callback(const void *msg);
        void sport_mode_state_callback(const void *msg);

        void publish_state();

        // Parameters
        std::string m_network_interface;
        float m_publish_rate;
        std::vector<std::string> m_joint_names;

        // DDS subscribers
        unitree::robot::ChannelSubscriberPtr<
            unitree_go::msg::dds_::LowState_>
            m_low_state_subscriber;

        unitree::robot::ChannelSubscriberPtr<
            unitree_go::msg::dds_::SportModeState_>
            m_sport_state_subscriber;

        // Cached state
        std::tuple<
            rclcpp::Time,
            unitree_go::msg::dds_::LowState_>
            m_last_low_state;

        unitree_go::msg::dds_::SportModeState_
            m_last_sport_state;

        std::mutex m_low_state_mutex;
        std::mutex m_sport_state_mutex;

        // Publishers
        rclcpp::Publisher<
            sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;

        rclcpp::Publisher<
            sensor_msgs::msg::Imu>::SharedPtr m_imu_pub;

        rclcpp::Publisher<
            geometry_msgs::msg::Vector3Stamped>::SharedPtr m_base_lin_vel_pub;

        rclcpp::TimerBase::SharedPtr m_publish_timer;
    };
}