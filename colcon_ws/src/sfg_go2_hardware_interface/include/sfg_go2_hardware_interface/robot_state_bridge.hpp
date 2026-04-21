#pragma once

#include <mutex>
#include <tuple>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace sfg_go2_hardware_interface
{
    class RobotStateBridge : public rclcpp::Node
    {
    public:
        explicit RobotStateBridge(const rclcpp::NodeOptions &options);

    private:
        void low_state_callback(const void *msg);
        void publish_state_topics();
        void publish_joint_states(const rclcpp::Time &stamp, const unitree_go::msg::dds_::LowState_ &low_state);
        void publish_imu(const rclcpp::Time &stamp, const unitree_go::msg::dds_::LowState_ &low_state);

        // ROS parameters
        std::string m_network_interface;
        float m_publish_rate;
        std::vector<std::string> m_joint_names;
        bool m_publish_imu;

        unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> m_low_state_subscriber;
        std::tuple<rclcpp::Time, unitree_go::msg::dds_::LowState_> m_last_low_state;
        std::mutex m_last_low_state_mutex;
        bool m_has_low_state{false};

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_publisher;

        rclcpp::TimerBase::SharedPtr m_publish_state_timer;
    };
}