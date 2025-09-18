#include "sfg_go2_hardware_interface/joint_state_publisher.hpp"

#include "sfg_utils/fqn/ros_fqn_builder.hpp"

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
        m_low_state_subscriber = std::make_unique<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>("rt/lowstate");
        m_low_state_subscriber->InitChannel(std::bind(&JointStatePublisher::low_state_callback, this, std::placeholders::_1));

        // Set up interfaces.
        m_joint_state_publisher = create_publisher<sensor_msgs::msg::JointState>(
            sfg_utils::fqn::RosFqnBuilder().resource(sfg_utils::fqn::Resource::JointStates).build(sfg_utils::fqn::RosFqnSegment::Resource),
            10);
        m_publish_joint_states_timer = create_wall_timer(
            std::chrono::duration<float>(1.0f / m_publish_rate),
            std::bind(&JointStatePublisher::publish_joint_states, this));
    }

    void JointStatePublisher::low_state_callback(const void *msg)
    {
        std::lock_guard lock(m_last_low_state_mutex);
        m_last_low_state = std::make_tuple(now(), *static_cast<const unitree_go::msg::dds_::LowState_ *>(msg));
    }

    void JointStatePublisher::publish_joint_states()
    {
        auto msg = std::make_unique<sensor_msgs::msg::JointState>();
        {
            std::lock_guard lock(m_last_low_state_mutex);
            msg->header.stamp = std::get<0>(m_last_low_state);
            const auto &motor_state = std::get<1>(m_last_low_state).motor_state();

            for (size_t index = 0; index < std::min(m_joint_names.size(), motor_state.size()); index++)
            {
                msg->name.push_back(m_joint_names[index]);
                msg->position.push_back(motor_state[index].q());     // Unit is [rad].
                msg->velocity.push_back(motor_state[index].dq());    // Unit is [rad/s].
                msg->effort.push_back(motor_state[index].tau_est()); // Unit not known at the moment.
            }
        }
        m_joint_state_publisher->publish(std::move(msg));
    }
}