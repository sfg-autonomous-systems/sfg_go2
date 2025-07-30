#include "sfg_go2_hardware_interface/joint_state_publisher.hpp"

namespace sfg_go2_hardware_interface
{
    JointStatePublisher::JointStatePublisher(const rclcpp::NodeOptions &options) : Node("state_publisher", options)
    {
        // Declare and retrieve ROS parameters.
        m_network_interface = declare_parameter<std::string>(
            "network_interface",
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("The name of the network interface to use for communication with the robot."));

        m_publish_rate = declare_parameter(
            "publish_rate",
            20.0f,
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("The rate at which to publish joint states in [Hz]."));

        m_joint_names = declare_parameter<std::vector<std::string>>(
            "joint_names",
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("The names of the joints to be published."));

        unitree::robot::ChannelFactory::Instance()->Init(0, m_network_interface);
        m_lowstate_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>("rt/lowstate");
        m_lowstate_subscriber->InitChannel(std::bind(&JointStatePublisher::lowstate_callback, this, std::placeholders::_1));

        // Set up interfaces.
        m_joint_state_publisher = create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 10);
        m_publish_joint_states_timer = create_wall_timer(
            std::chrono::duration<float>(1.0f / m_publish_rate),
            std::bind(&JointStatePublisher::publish_joint_states, this));
    }

    void JointStatePublisher::lowstate_callback(const void *msg)
    {
        m_last_low_state = *static_cast<const unitree_go::msg::dds_::LowState_ *>(msg);
    }

    void JointStatePublisher::publish_joint_states()
    {
        sensor_msgs::msg::JointState msg;

        auto motor_state = m_last_low_state.motor_state();

        for (size_t index = 0; index < m_joint_names.size(); index++)
        {
            auto position = motor_state[index].q();     // Unit is [rad].
            auto velocity = motor_state[index].dq();    // Unit is [rad/s].
            auto effort = motor_state[index].tau_est(); // Unit now known at the moment.

            msg.name.push_back(m_joint_names[index]);
            msg.position.push_back(position);
            msg.velocity.push_back(velocity);
            msg.effort.push_back(effort);
        }

        m_joint_state_publisher->publish(msg);
    }
}