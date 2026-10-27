#include "sfg_go2_hardware_interface/locomotion_controller.hpp"

#include <cmath>

#include "sfg_utils/fqn/ros_fqn_builder.hpp"

namespace sfg_go2_hardware_interface
{
    LocomotionController::LocomotionController(const rclcpp::NodeOptions &options) : LocomotionControllerBase("locomotion_controller", options)
    {
        // Declare and retrieve ROS parameters.
        m_network_interface = declare_parameter<std::string>(
            "network_interface",
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("The name of the network interface to use for communication with the robot."));

        m_odom_frame_id = declare_parameter<std::string>(
            "odom_frame_id",
            "odom");

        m_base_frame_id = declare_parameter<std::string>(
            "base_frame_id",
            "base_link");

        m_client_timeout = declare_parameter(
            "client_timeout",
            10.0f,
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("Timeout for the SportClient in seconds."));

        unitree::robot::ChannelFactory::Instance()->Init(0, m_network_interface);
        m_sport_mode_state_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>("rt/sportmodestate");
        m_sport_mode_state_subscriber->InitChannel(std::bind(&LocomotionController::sport_mode_state_callback, this, std::placeholders::_1));

        using namespace sfg_utils::fqn;
        std::string odom_topic_name = RosFqnBuilder().scope(Scope::Global).agent().component(sfg_utils::fqn::Component::LocomotionController).resource(Resource::Custom, "odom").build();
        m_odometry_publisher = create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 10);

        m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        m_sport_client = std::make_unique<unitree::robot::go2::SportClient>();
        m_sport_client->SetTimeout(m_client_timeout);
        m_sport_client->Init();
        m_sport_client->AutoRecoverSet(false);

        RCLCPP_INFO(get_logger(), "Started locomotion controller; publishing odometry on '%s'.", odom_topic_name.c_str());
    }

    void LocomotionController::arm()
    {
        m_sport_client->RecoveryStand();
    }

    void LocomotionController::disarm()
    {
        m_sport_client->StandDown();
    }

    void LocomotionController::emergency_stop()
    {
        m_sport_client->Damp();
    }

    void LocomotionController::apply_cmd(const geometry_msgs::msg::TwistStamped &cmd)
    {
        if (auto error = m_sport_client->Move(cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z))
        {
            throw std::runtime_error("The underlying driver returned an error code of '" + std::to_string(error) + "'.");
        }
    }

    void LocomotionController::sport_mode_state_callback(const void *msg)
    {
        std::lock_guard lock(m_last_sport_mode_state_mutex);
        m_last_sport_mode_state = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(msg);

        const auto &position = m_last_sport_mode_state->position();
        const auto &velocity = m_last_sport_mode_state->velocity();
        const auto &orientation = m_last_sport_mode_state->imu_state().quaternion();
        const double quaternion_norm = std::sqrt(
            static_cast<double>(orientation[0]) * orientation[0] +
            static_cast<double>(orientation[1]) * orientation[1] +
            static_cast<double>(orientation[2]) * orientation[2] +
            static_cast<double>(orientation[3]) * orientation[3]);

        if (!std::isfinite(quaternion_norm) || quaternion_norm < 1e-6)
        {
            RCLCPP_WARN(get_logger(), "Ignoring sport-mode state with an invalid IMU quaternion.");
            return;
        }

        if (!std::isfinite(position[0]) || !std::isfinite(position[1]) ||
            !std::isfinite(position[2]) || !std::isfinite(velocity[0]) ||
            !std::isfinite(velocity[1]) || !std::isfinite(velocity[2]) ||
            !std::isfinite(m_last_sport_mode_state->yaw_speed()))
        {

            RCLCPP_WARN(get_logger(), "Ignoring sport-mode state with non-finite odometry values.");
            return;
        }

        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = now();
        odometry.header.frame_id = m_odom_frame_id;
        odometry.child_frame_id = m_base_frame_id;
        odometry.pose.pose.position.x = position[0];
        odometry.pose.pose.position.y = position[1];
        odometry.pose.pose.position.z = position[2];
        odometry.pose.pose.orientation.x = orientation[1] / quaternion_norm;
        odometry.pose.pose.orientation.y = orientation[2] / quaternion_norm;
        odometry.pose.pose.orientation.z = orientation[3] / quaternion_norm;
        odometry.pose.pose.orientation.w = orientation[0] / quaternion_norm;
        odometry.twist.twist.linear.x = velocity[0];
        odometry.twist.twist.linear.y = velocity[1];
        odometry.twist.twist.linear.z = velocity[2];
        odometry.twist.twist.angular.z = static_cast<double>(m_last_sport_mode_state->yaw_speed());
        m_odometry_publisher->publish(odometry);

        geometry_msgs::msg::TransformStamped transform;
        transform.header = odometry.header;
        transform.child_frame_id = odometry.child_frame_id;
        transform.transform.translation.x = odometry.pose.pose.position.x;
        transform.transform.translation.y = odometry.pose.pose.position.y;
        transform.transform.translation.z = odometry.pose.pose.position.z;
        transform.transform.rotation = odometry.pose.pose.orientation;
        m_tf_broadcaster->sendTransform(transform);
    }
}