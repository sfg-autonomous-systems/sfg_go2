#include "sfg_go2_trajectory_controller/trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <stdexcept>
#include <tf2/exceptions.h>

namespace sfg_go2_trajectory_controller
{
    namespace
    {
        double wrap_angle(double angle)
        {
            return std::atan2(std::sin(angle), std::cos(angle));
        }

        double clamp(double value, double limit)
        {
            return std::clamp(value, -limit, limit);
        }
    } // namespace

    void TrajectoryController::PIDState::reset()
    {
        integral = 0.0;
        previous_error = 0.0;
        has_previous_error = false;
    }

    double TrajectoryController::PIDState::update(double error, double dt, double kp, double ki, double kd, double integral_limit)
    {
        integral = clamp(integral + error * dt, integral_limit);
        const double derivative = has_previous_error ? (error - previous_error) / dt : 0.0;
        previous_error = error;
        has_previous_error = true;
        return kp * error + ki * integral + kd * derivative;
    }

    TrajectoryController::TrajectoryController(const rclcpp::NodeOptions &options) : Node("trajectory_controller", options)
    {
        m_target_initialized = !m_target_from_start;

        m_cmd_vel_publisher = create_publisher<geometry_msgs::msg::TwistStamped>(
            "cmd_vel", rclcpp::SystemDefaultsQoS());
        m_tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
        m_control_timer = create_wall_timer(
            std::chrono::duration<double>(1.0 / m_control_rate),
            std::bind(&TrajectoryController::control_callback, this));
        m_last_control_time = std::chrono::steady_clock::now();

        RCLCPP_INFO(get_logger(),
                    "Trajectory controller reading TF %s -> %s and publishing "
                    "locomotion commands on 'cmd_vel'.",
                    m_target_frame.c_str(), m_base_frame.c_str());
    }

    void TrajectoryController::control_callback()
    {
        const auto current_time = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(current_time - m_last_control_time).count();
        m_last_control_time = current_time;

        geometry_msgs::msg::TransformStamped base_transform;
        try
        {
            base_transform = m_tf_buffer->lookupTransform(m_target_frame, m_base_frame,
                                                          tf2::TimePointZero);
        }
        catch (const tf2::TransformException &exception)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Could not get TF %s -> %s: %s",
                m_target_frame.c_str(), m_base_frame.c_str(), exception.what());
            m_pid_x.reset();
            m_pid_y.reset();
            m_pid_yaw.reset();
            publish_stop();
            return;
        }

        const auto &translation = base_transform.transform.translation;
        const auto &orientation = base_transform.transform.rotation;
        const double quaternion_norm = std::sqrt(orientation.x * orientation.x + orientation.y * orientation.y + orientation.z * orientation.z + orientation.w * orientation.w);

        if (!std::isfinite(translation.x) || !std::isfinite(translation.y) ||
            !std::isfinite(quaternion_norm) || quaternion_norm < 1e-6)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Ignoring invalid Vicon transform.");
            m_pid_x.reset();
            m_pid_y.reset();
            m_pid_yaw.reset();
            publish_stop();
            return;
        }

        const double qx = orientation.x / quaternion_norm;
        const double qy = orientation.y / quaternion_norm;
        const double qz = orientation.z / quaternion_norm;
        const double qw = orientation.w / quaternion_norm;

        m_pose_x = translation.x;
        m_pose_y = translation.y;
        m_pose_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        const double transform_age = (now() - static_cast<rclcpp::Time>(base_transform.header.stamp,
                                                                        get_clock()->get_clock_type()))
                                         .seconds();
        if (transform_age < 0.0 || transform_age > m_pose_timeout)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Vicon transform is stale (age %.3f s); publishing zero velocity.",
                transform_age);
            m_pid_x.reset();
            m_pid_y.reset();
            m_pid_yaw.reset();
            publish_stop();
            return;
        }

        if (!m_target_initialized)
        {
            m_target_x = m_pose_x;
            m_target_y = m_pose_y;
            m_target_yaw = m_pose_yaw;
            m_target_initialized = true;
            RCLCPP_INFO(get_logger(),
                        "Holding the first valid Vicon transform as the target.");
        }

        if (dt <= 0.0 || dt > m_pose_timeout)
        {
            m_pid_x.reset();
            m_pid_y.reset();
            m_pid_yaw.reset();
            publish_stop();
            return;
        }

        const double error_x = m_target_x - m_pose_x;
        const double error_y = m_target_y - m_pose_y;
        const double world_velocity_x = m_pid_x.update(
            error_x, dt, m_linear_kp, m_linear_ki, m_linear_kd, m_integral_limit_x);
        const double world_velocity_y = m_pid_y.update(
            error_y, dt, m_linear_kp, m_linear_ki, m_linear_kd, m_integral_limit_y);

        const double cosine = std::cos(m_pose_yaw);
        const double sine = std::sin(m_pose_yaw);
        const double error_yaw = wrap_angle(m_target_yaw - m_pose_yaw);
        const double velocity_x =
            clamp(cosine * world_velocity_x + sine * world_velocity_y, m_max_vel_x);
        const double velocity_y =
            clamp(-sine * world_velocity_x + cosine * world_velocity_y, m_max_vel_y);
        const double velocity_yaw =
            clamp(m_pid_yaw.update(error_yaw, dt, m_yaw_kp, m_yaw_ki, m_yaw_kd,
                                   m_integral_limit_yaw),
                  m_max_angular_z);

        geometry_msgs::msg::TwistStamped command;
        command.header.stamp = now();
        command.header.frame_id = m_base_frame;
        command.twist.linear.x = velocity_x;
        command.twist.linear.y = velocity_y;
        command.twist.angular.z = velocity_yaw;
        m_cmd_vel_publisher->publish(command);
    }

    void TrajectoryController::publish_stop()
    {
        geometry_msgs::msg::TwistStamped command;
        command.header.stamp = now();
        command.header.frame_id = m_base_frame;
        m_cmd_vel_publisher->publish(command);
    }
} // namespace sfg_go2_trajectory_controller
