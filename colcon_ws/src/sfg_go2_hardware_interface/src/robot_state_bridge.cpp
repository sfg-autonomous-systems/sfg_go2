#include "sfg_go2_hardware_interface/robot_state_bridge.hpp"

#include "sfg_utils/fqn/ros_fqn_builder.hpp"

#include <unitree/robot/channel/channel_factory.hpp>

namespace sfg_go2_hardware_interface
{

    RobotStateBridge::RobotStateBridge(
        const rclcpp::NodeOptions &options)
        : Node("robot_state_bridge", options)
    {
        m_network_interface =
            declare_parameter<std::string>(
                "network_interface",
                "enP8p1s0");

        m_publish_rate =
            declare_parameter(
                "publish_rate",
                200.0f);

        m_joint_names =
            declare_parameter<std::vector<std::string>>(
                "joint_names");

        unitree::robot::ChannelFactory::Instance()->Init(
            0,
            m_network_interface);

        // DDS subscriptions

        m_low_state_subscriber =
            std::make_unique<
                unitree::robot::ChannelSubscriber<
                    unitree_go::msg::dds_::LowState_>>("rt/lowstate");

        m_low_state_subscriber->InitChannel(
            std::bind(
                &RobotStateBridge::low_state_callback,
                this,
                std::placeholders::_1));

        m_sport_state_subscriber =
            std::make_unique<
                unitree::robot::ChannelSubscriber<
                    unitree_go::msg::dds_::SportModeState_>>("rt/sportmodestate");

        m_sport_state_subscriber->InitChannel(
            std::bind(
                &RobotStateBridge::sport_mode_state_callback,
                this,
                std::placeholders::_1));

        // Publishers

        m_joint_state_pub =
            create_publisher<
                sensor_msgs::msg::JointState>(
                "/local/sfg_go2_01/joint_states",
                10);

        m_imu_pub =
            create_publisher<
                sensor_msgs::msg::Imu>(
                "/go2/imu",
                10);

        m_base_lin_vel_pub =
            create_publisher<
                geometry_msgs::msg::Vector3Stamped>(
                "/go2/base_lin_vel",
                10);

        m_publish_timer =
            create_wall_timer(
                std::chrono::duration<float>(
                    1.0f / m_publish_rate),
                std::bind(
                    &RobotStateBridge::publish_state,
                    this));

        RCLCPP_INFO(
            get_logger(),
            "RobotStateBridge started.");
    }

    void RobotStateBridge::low_state_callback(
        const void *msg)
    {
        std::lock_guard lock(m_low_state_mutex);

        m_last_low_state =
            std::make_tuple(
                now(),
                *static_cast<
                    const unitree_go::msg::dds_::LowState_ *>(msg));

        RCLCPP_INFO(
            this->get_logger(),
            "LOW STATE CALLBACK TRIGGERED");
    }

    void RobotStateBridge::sport_mode_state_callback(
        const void *msg)
    {
        std::lock_guard lock(m_sport_state_mutex);

        m_last_sport_state =
            *static_cast<
                const unitree_go::msg::dds_::SportModeState_ *>(msg);
    }

    void RobotStateBridge::publish_state()
    {
        sensor_msgs::msg::JointState joint_msg;
        sensor_msgs::msg::Imu imu_msg;
        geometry_msgs::msg::Vector3Stamped vel_msg;

        {
            std::lock_guard lock(m_low_state_mutex);

            joint_msg.header.stamp =
                std::get<0>(m_last_low_state);

            const auto &low_state =
                std::get<1>(m_last_low_state);

            const auto &motor_state =
                low_state.motor_state();

            for (
                size_t i = 0;
                i < std::min(
                        m_joint_names.size(),
                        motor_state.size());
                i++)
            {
                joint_msg.name.push_back(
                    m_joint_names[i]);

                joint_msg.position.push_back(
                    motor_state[i].q());

                joint_msg.velocity.push_back(
                    motor_state[i].dq());

                joint_msg.effort.push_back(
                    motor_state[i].tau_est());
            }

            // IMU

            imu_msg.header.stamp = now();
            imu_msg.header.frame_id = "imu_link";

            imu_msg.orientation.x =
                low_state.imu_state().quaternion()[1];

            imu_msg.orientation.y =
                low_state.imu_state().quaternion()[2];

            imu_msg.orientation.z =
                low_state.imu_state().quaternion()[3];

            imu_msg.orientation.w =
                low_state.imu_state().quaternion()[0];

            imu_msg.angular_velocity.x =
                low_state.imu_state().gyroscope()[0];

            imu_msg.angular_velocity.y =
                low_state.imu_state().gyroscope()[1];

            imu_msg.angular_velocity.z =
                low_state.imu_state().gyroscope()[2];

            imu_msg.linear_acceleration.x =
                low_state.imu_state().accelerometer()[0];

            imu_msg.linear_acceleration.y =
                low_state.imu_state().accelerometer()[1];

            imu_msg.linear_acceleration.z =
                low_state.imu_state().accelerometer()[2];
        }

        {
            std::lock_guard lock(m_sport_state_mutex);

            vel_msg.header.stamp = now();
            vel_msg.header.frame_id = "base";

            vel_msg.vector.x =
                m_last_sport_state.velocity()[0];

            vel_msg.vector.y =
                m_last_sport_state.velocity()[1];

            vel_msg.vector.z =
                m_last_sport_state.velocity()[2];
        }

        m_joint_state_pub->publish(joint_msg);
        m_imu_pub->publish(imu_msg);
        m_base_lin_vel_pub->publish(vel_msg);
    }

}