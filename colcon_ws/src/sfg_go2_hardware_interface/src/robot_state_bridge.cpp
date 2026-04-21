#include "sfg_go2_hardware_interface/robot_state_bridge.hpp"

#include "sfg_utils/fqn/ros_fqn_builder.hpp"

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

        m_publish_rate = declare_parameter<double>(
            "publish_rate",
            50.0,
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("The rate at which to publish bridged robot state topics in [Hz]."));

        m_joint_names = declare_parameter<std::vector<std::string>>(
            "joint_names",
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("The names of the joints to be published in JointState."));

        m_publish_imu = declare_parameter<bool>(
            "publish_imu",
            true,
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("Whether to publish IMU data as sensor_msgs/Imu."));

        m_publish_raw_low_state = declare_parameter<bool>(
            "publish_raw_low_state",
            false,
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("Whether to publish the raw low state as a ROS message. Requires a ROS-side LowState msg type."));

        unitree::robot::ChannelFactory::Instance()->Init(0, m_network_interface);
        m_low_state_subscriber =
            std::make_unique<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>("rt/lowstate");
        m_low_state_subscriber->InitChannel(
            std::bind(&RobotStateBridge::low_state_callback, this, std::placeholders::_1));

        // Set up ROS publishers.
        m_joint_state_publisher = create_publisher<sensor_msgs::msg::JointState>(
            sfg_utils::fqn::RosFqnBuilder()
                .resource(sfg_utils::fqn::Resource::JointStates)
                .build(sfg_utils::fqn::RosFqnSegment::Resource),
            10);

        if (m_publish_imu)
        {
            m_imu_publisher = create_publisher<sensor_msgs::msg::Imu>(
                "/go2/imu",
                10);
        }

        // Uncomment only if you have unitree_go::msg::LowState available.
        // if (m_publish_raw_low_state)
        // {
        //     m_low_state_publisher = create_publisher<unitree_go::msg::LowState>(
        //         "/go2/low_state",
        //         10);
        // }

        m_publish_timer = create_wall_timer(
            std::chrono::duration<double>(1.0 / m_publish_rate),
            std::bind(&RobotStateBridge::publish_state_topics, this));

        RCLCPP_INFO(get_logger(), "Started robot_state_bridge.");
    }

    void RobotStateBridge::low_state_callback(const void *msg)
    {
        std::lock_guard lock(m_last_low_state_mutex);
        m_last_low_state = std::make_tuple(
            now(),
            *static_cast<const unitree_go::msg::dds_::LowState_ *>(msg));
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

        // Uncomment only if you have unitree_go::msg::LowState available.
        // if (m_publish_raw_low_state)
        // {
        //     publish_low_state(stamp, low_state);
        // }
    }

    void RobotStateBridge::publish_joint_states(
        const rclcpp::Time &stamp,
        const unitree_go::msg::dds_::LowState_ &low_state)
    {
        auto msg = std::make_unique<sensor_msgs::msg::JointState>();
        msg->header.stamp = stamp;

        const auto &motor_state = low_state.motor_state();

        for (size_t index = 0; index < std::min(m_joint_names.size(), motor_state.size()); ++index)
        {
            msg->name.push_back(m_joint_names[index]);
            msg->position.push_back(motor_state[index].q());     // [rad]
            msg->velocity.push_back(motor_state[index].dq());    // [rad/s]
            msg->effort.push_back(motor_state[index].tau_est()); // estimated torque
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

        // NOTE:
        // These field names may need adjustment based on your exact Unitree message definition.
        // Common layout is something like:
        // low_state.imu_state().quaternion()
        // low_state.imu_state().gyroscope()
        // low_state.imu_state().accelerometer()

        const auto &imu_state = low_state.imu_state();

        const auto &quat = imu_state.quaternion();
        const auto &gyro = imu_state.gyroscope();
        const auto &acc = imu_state.accelerometer();

        // Assumes quaternion ordering is [w, x, y, z].
        // If your SDK stores [x, y, z, w], swap accordingly.
        msg->orientation.w = quat[0];
        msg->orientation.x = quat[1];
        msg->orientation.y = quat[2];
        msg->orientation.z = quat[3];

        msg->angular_velocity.x = gyro[0];
        msg->angular_velocity.y = gyro[1];
        msg->angular_velocity.z = gyro[2];

        msg->linear_acceleration.x = acc[0];
        msg->linear_acceleration.y = acc[1];
        msg->linear_acceleration.z = acc[2];

        m_imu_publisher->publish(std::move(msg));
    }

    // Uncomment and adapt only if you have a ROS-side LowState message.
    // void RobotStateBridge::publish_low_state(
    //     const rclcpp::Time &stamp,
    //     const unitree_go::msg::dds_::LowState_ &low_state)
    // {
    //     auto msg = std::make_unique<unitree_go::msg::LowState>();

    //     // TODO: map DDS low_state fields into the ROS LowState message here.
    //     // This depends on your exact ROS message definition.

    //     m_low_state_publisher->publish(std::move(msg));
    // }

} // namespace sfg_go2_hardware_interface