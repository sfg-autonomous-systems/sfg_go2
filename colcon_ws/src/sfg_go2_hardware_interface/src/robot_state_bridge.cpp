#include "sfg_go2_hardware_interface/robot_state_bridge.hpp"

#include "sfg_utils/fqn/ros_fqn_builder.hpp"

#include <unitree/robot/channel/channel_factory.hpp>

namespace sfg_go2_hardware_interface
{
    RobotStateBridge::RobotStateBridge(const rclcpp::NodeOptions &options)
        : Node("robot_state_bridge", options)
    {
        // Declare and retrieve ROS parameters.
        m_network_interface = declare_parameter<std::string>(
            "network_interface",
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("The name of the network interface to use for communication with the robot."));

        m_publish_rate = declare_parameter(
            "publish_rate",
            50.0f,
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("The rate at which to publish bridged robot state topics in [Hz]."));

        m_joint_names = declare_parameter<std::vector<std::string>>(
            "joint_names",
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("The names of the joints to be published."));

        m_publish_imu = declare_parameter(
            "publish_imu",
            true,
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("Whether to publish IMU data as sensor_msgs/msg/Imu."));

        unitree::robot::ChannelFactory::Instance()->Init(0, m_network_interface);
        m_low_state_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>("rt/lowstate");
        m_low_state_subscriber->InitChannel(std::bind(&RobotStateBridge::low_state_callback, this, std::placeholders::_1));

        // Set up interfaces.
        m_joint_state_publisher = create_publisher<sensor_msgs::msg::JointState>(
            sfg_utils::fqn::RosFqnBuilder()
                .resource(sfg_utils::fqn::Resource::JointStates)
                .build(sfg_utils::fqn::RosFqnSegment::Resource),
            10);

        if (m_publish_imu)
        {
            m_imu_publisher = create_publisher<sensor_msgs::msg::Imu>("/go2/imu", 10);
        }

        m_publish_state_timer = create_wall_timer(
            std::chrono::duration<float>(1.0f / m_publish_rate),
            std::bind(&RobotStateBridge::publish_state_topics, this));

        RCLCPP_INFO(get_logger(), "Started robot_state_bridge.");
    }

    void RobotStateBridge::low_state_callback(const void *msg)
    {
        std::lock_guard lock(m_last_low_state_mutex);
        m_last_low_state = std::make_tuple(now(), *static_cast<const unitree_go::msg::dds_::LowState_ *>(msg));
        m_has_low_state = true;
    }

    void RobotStateBridge::publish_state_topics()
    {
        rclcpp::Time stamp;
        unitree_go::msg::dds_::LowState_ low_state;

        {
            std::lock_guard lock(m_last_low_state_mutex);
            if (!m_has_low_state)
            {
                return;
            }

            stamp = std::get<0>(m_last_low_state);
            low_state = std::get<1>(m_last_low_state);
        }

        publish_joint_states(stamp, low_state);

        if (m_publish_imu)
        {
            publish_imu(stamp, low_state);
        }
    }

    void RobotStateBridge::publish_joint_states(
        const rclcpp::Time &stamp,
        const unitree_go::msg::dds_::LowState_ &low_state)
    {
        auto msg = std::make_unique<sensor_msgs::msg::JointState>();
        msg->header.stamp = stamp;

        const auto &motor_state = low_state.motor_state();

        for (size_t index = 0; index < std::min(m_joint_names.size(), motor_state.size()); index++)
        {
            msg->name.push_back(m_joint_names[index]);
            msg->position.push_back(motor_state[index].q());     // Unit is [rad].
            msg->velocity.push_back(motor_state[index].dq());    // Unit is [rad/s].
            msg->effort.push_back(motor_state[index].tau_est()); // Estimated torque.
        }

        m_joint_state_publisher->publish(std::move(msg));
    }

    void RobotStateBridge::publish_imu(
        const rclcpp::Time &stamp,
        const unitree_go::msg::dds_::LowState_ &low_state)
    {
        auto msg = std::make_unique<sensor_msgs::msg::Imu>();
        msg->header.stamp = stamp;
        msg->header.frame_id = "imu_link";

        const auto &imu_state = low_state.imu_state();

        // NOTE:
        // This assumes quaternion ordering is [w, x, y, z].
        // If your Unitree message stores [x, y, z, w], swap these assignments.
        const auto &quat = imu_state.quaternion();
        msg->orientation.x = quat[0];
        msg->orientation.y = quat[1];
        msg->orientation.z = quat[2];
        msg->orientation.w = quat[3];

        const auto &gyro = imu_state.gyroscope();
        msg->angular_velocity.x = gyro[0];
        msg->angular_velocity.y = gyro[1];
        msg->angular_velocity.z = gyro[2];

        const auto &acc = imu_state.accelerometer();
        msg->linear_acceleration.x = acc[0];
        msg->linear_acceleration.y = acc[1];
        msg->linear_acceleration.z = acc[2];

        m_imu_publisher->publish(std::move(msg));
    }
}