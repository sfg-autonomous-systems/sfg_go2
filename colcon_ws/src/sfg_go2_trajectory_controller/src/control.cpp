#include "sfg_go2_trajectory_controller/trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

namespace sfg_go2_trajectory_controller
{
    namespace
    {
        double clamp(double value, double limit)
        {
            return std::clamp(value, -limit, limit);
        }

        double wrap_angle(double angle)
        {
            return std::atan2(std::sin(angle), std::cos(angle));
        }
    } // namespace

    void TrajectoryController::control_callback()
    {
        const auto current_time = std::chrono::steady_clock::now();
        const double dt =
            std::chrono::duration<double>(current_time - m_last_control_time)
                .count();
        m_last_control_time = current_time;

        std::shared_ptr<GoalHandleFollowTrajectory> active_goal;
        {
            std::lock_guard<std::mutex> lock(m_goal_mutex);
            active_goal = m_active_goal;
        }
        if (handle_canceling_goal(active_goal))
        {
            return;
        }
        if (!update_robot_pose())
        {
            return;
        }
        if (dt <= 0.0 || dt > m_pose_timeout)
        {
            stop_and_reset();
            return;
        }

        const auto reference = get_trajectory_reference(active_goal);
        const double error_x = reference.x - m_pose_x;
        const double error_y = reference.y - m_pose_y;
        const double world_velocity_x =
            m_pid_x.update(
                error_x,
                dt,
                m_linear_kp,
                m_linear_ki,
                m_linear_kd,
                m_integral_limit_x) +
            reference.velocity_x;
        const double world_velocity_y =
            m_pid_y.update(
                error_y,
                dt,
                m_linear_kp,
                m_linear_ki,
                m_linear_kd,
                m_integral_limit_y) +
            reference.velocity_y;

        const double cosine = std::cos(m_pose_yaw);
        const double sine = std::sin(m_pose_yaw);
        const double error_yaw = wrap_angle(reference.yaw - m_pose_yaw);
        const double velocity_x = clamp(
            cosine * world_velocity_x + sine * world_velocity_y,
            m_max_vel_x);
        const double velocity_y = clamp(
            -sine * world_velocity_x + cosine * world_velocity_y,
            m_max_vel_y);
        const double velocity_yaw = clamp(
            m_pid_yaw.update(
                error_yaw,
                dt,
                m_yaw_kp,
                m_yaw_ki,
                m_yaw_kd,
                m_integral_limit_yaw) +
                reference.velocity_yaw,
            m_max_angular_z);

        geometry_msgs::msg::TwistStamped command;
        command.header.stamp = now();
        command.header.frame_id = m_base_frame;
        command.twist.linear.x = velocity_x;
        command.twist.linear.y = velocity_y;
        command.twist.angular.z = velocity_yaw;
        m_cmd_vel_publisher->publish(command);

        if (active_goal && reference.finished &&
            std::hypot(reference.final_x - m_pose_x,
                       reference.final_y - m_pose_y) <= m_position_tolerance &&
            std::abs(wrap_angle(reference.final_yaw - m_pose_yaw)) <=
                m_yaw_tolerance)
        {
            complete_trajectory(active_goal, reference);
        }
    }

    bool TrajectoryController::handle_canceling_goal(
        const std::shared_ptr<GoalHandleFollowTrajectory> &goal_handle)
    {
        if (!goal_handle || !goal_handle->is_canceling())
        {
            return false;
        }

        auto result = std::make_shared<FollowTrajectory::Result>();
        result->result = FollowTrajectory::Result::ERROR;
        result->message = "Trajectory canceled.";
        goal_handle->canceled(result);
        {
            std::lock_guard<std::mutex> lock(m_goal_mutex);
            if (m_active_goal == goal_handle)
            {
                m_active_goal.reset();
            }
        }
        if (m_target_initialized)
        {
            m_target_x = m_pose_x;
            m_target_y = m_pose_y;
            m_target_yaw = m_pose_yaw;
        }
        stop_and_reset();
        return true;
    }

    bool TrajectoryController::update_robot_pose()
    {
        geometry_msgs::msg::TransformStamped base_transform;
        try
        {
            base_transform = m_tf_buffer->lookupTransform(
                m_target_frame, m_base_frame, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &exception)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Could not get TF %s -> %s: %s",
                m_target_frame.c_str(),
                m_base_frame.c_str(),
                exception.what());
            stop_and_reset();
            return false;
        }

        const auto &translation = base_transform.transform.translation;
        const auto &orientation = base_transform.transform.rotation;
        const double quaternion_norm = std::sqrt(
            orientation.x * orientation.x +
            orientation.y * orientation.y +
            orientation.z * orientation.z +
            orientation.w * orientation.w);
        if (!std::isfinite(translation.x) || !std::isfinite(translation.y) ||
            !std::isfinite(quaternion_norm) || quaternion_norm < 1e-6)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Ignoring invalid Vicon transform.");
            stop_and_reset();
            return false;
        }

        const double qx = orientation.x / quaternion_norm;
        const double qy = orientation.y / quaternion_norm;
        const double qz = orientation.z / quaternion_norm;
        const double qw = orientation.w / quaternion_norm;
        m_pose_x = translation.x;
        m_pose_y = translation.y;
        m_pose_yaw = std::atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz));

        const double transform_age =
            (now() - rclcpp::Time(
                         base_transform.header.stamp,
                         get_clock()->get_clock_type()))
                .seconds();
        if (transform_age < 0.0 || transform_age > m_pose_timeout)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Vicon transform is stale (age %.3f s); publishing zero "
                "velocity.",
                transform_age);
            stop_and_reset();
            return false;
        }

        if (!m_target_initialized)
        {
            m_target_x = m_pose_x;
            m_target_y = m_pose_y;
            m_target_yaw = m_pose_yaw;
            m_target_initialized = true;
            RCLCPP_INFO(
                get_logger(),
                "Holding the first valid Vicon transform as the target.");
        }
        return true;
    }

    void TrajectoryController::stop_and_reset()
    {
        reset_pid();
        publish_stop();
    }

    void TrajectoryController::complete_trajectory(
        const std::shared_ptr<GoalHandleFollowTrajectory> &goal_handle,
        const TrajectoryReference &reference)
    {
        auto result = std::make_shared<FollowTrajectory::Result>();
        result->result = FollowTrajectory::Result::SUCCESS;
        result->message = "Trajectory completed.";
        goal_handle->succeed(result);

        std::lock_guard<std::mutex> lock(m_goal_mutex);
        if (m_active_goal == goal_handle)
        {
            m_active_goal.reset();
        }
        m_target_x = reference.final_x;
        m_target_y = reference.final_y;
        m_target_yaw = reference.final_yaw;
        m_target_initialized = true;
        reset_pid();
    }

    void TrajectoryController::publish_stop()
    {
        geometry_msgs::msg::TwistStamped command;
        command.header.stamp = now();
        command.header.frame_id = m_base_frame;
        m_cmd_vel_publisher->publish(command);
    }
} // namespace sfg_go2_trajectory_controller
