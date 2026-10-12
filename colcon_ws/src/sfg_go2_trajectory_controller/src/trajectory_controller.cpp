#include "sfg_go2_trajectory_controller/trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <stdexcept>
#include <string>
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
        m_target_frame = declare_parameter<std::string>("target_frame", "map");
        m_base_frame = declare_parameter<std::string>("base_frame", "sfg_go2_01/base_link");
        m_action_name = declare_parameter<std::string>("action_name", "follow_trajectory");
        m_target_x = declare_parameter<double>("target_x", 0.0);
        m_target_y = declare_parameter<double>("target_y", 0.0);
        m_target_yaw = declare_parameter<double>("target_yaw", 0.0);
        m_target_from_start = declare_parameter<bool>("target_from_start", true);
        m_control_rate = declare_parameter<double>("control_rate", 30.0);
        m_pose_timeout = declare_parameter<double>("pose_timeout", 0.5);
        m_max_vel_x = declare_parameter<double>("max_vel_x", 0.3);
        m_max_vel_y = declare_parameter<double>("max_vel_y", 0.3);
        m_max_angular_z = declare_parameter<double>("max_angular_z", 0.6);
        m_linear_kp = declare_parameter<double>("linear_kp", 0.8);
        m_linear_ki = declare_parameter<double>("linear_ki", 0.0);
        m_linear_kd = declare_parameter<double>("linear_kd", 0.0);
        m_yaw_kp = declare_parameter<double>("yaw_kp", 1.2);
        m_yaw_ki = declare_parameter<double>("yaw_ki", 0.0);
        m_yaw_kd = declare_parameter<double>("yaw_kd", 0.0);
        m_integral_limit_x = declare_parameter<double>("integral_limit_x", 1.0);
        m_integral_limit_y = declare_parameter<double>("integral_limit_y", 1.0);
        m_integral_limit_yaw = declare_parameter<double>("integral_limit_yaw", 1.0);
        m_position_tolerance = declare_parameter<double>("position_tolerance", 0.05);
        m_yaw_tolerance = declare_parameter<double>("yaw_tolerance", 0.1);

        if (!std::isfinite(m_control_rate) || m_control_rate <= 0.0 ||
            !std::isfinite(m_pose_timeout) || m_pose_timeout <= 0.0 ||
            !std::isfinite(m_max_vel_x) || m_max_vel_x <= 0.0 ||
            !std::isfinite(m_max_vel_y) || m_max_vel_y <= 0.0 ||
            !std::isfinite(m_max_angular_z) || m_max_angular_z <= 0.0 ||
            !std::isfinite(m_position_tolerance) || m_position_tolerance <= 0.0 ||
            !std::isfinite(m_yaw_tolerance) || m_yaw_tolerance <= 0.0)
        {
            throw std::invalid_argument("Controller rates, limits, timeouts, and tolerances must be positive and finite.");
        }

        m_target_initialized = !m_target_from_start;

        m_cmd_vel_publisher = create_publisher<geometry_msgs::msg::TwistStamped>(
            "cmd_vel", rclcpp::SystemDefaultsQoS());
        m_action_server = rclcpp_action::create_server<FollowTrajectory>(
            this, m_action_name,
            std::bind(&TrajectoryController::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&TrajectoryController::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&TrajectoryController::handle_accepted, this,
                      std::placeholders::_1));
        m_tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
        m_control_timer = create_wall_timer(
            std::chrono::duration<double>(1.0 / m_control_rate),
            std::bind(&TrajectoryController::control_callback, this));
        m_last_control_time = std::chrono::steady_clock::now();

        RCLCPP_INFO(get_logger(),
                    "Trajectory controller reading TF %s -> %s and publishing "
                    "locomotion commands on 'cmd_vel'; serving action '%s'.",
                    m_target_frame.c_str(), m_base_frame.c_str(), m_action_name.c_str());
    }

    rclcpp_action::GoalResponse TrajectoryController::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const FollowTrajectory::Goal> goal)
    {
        (void)uuid;
        const auto &trajectory = goal->trajectory;
        if (trajectory.waypoints.empty())
        {
            RCLCPP_WARN(get_logger(), "Rejecting an empty trajectory.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (trajectory.header.frame_id != m_target_frame)
        {
            RCLCPP_WARN(get_logger(),
                        "Rejecting trajectory in frame '%s'; expected '%s'.",
                        trajectory.header.frame_id.c_str(), m_target_frame.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (trajectory.header.stamp.sec < 0 ||
            trajectory.header.stamp.nanosec >= 1000000000U)
        {
            RCLCPP_WARN(get_logger(), "Rejecting trajectory with an invalid start timestamp.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        double previous_time = -1.0;
        for (const auto &waypoint : trajectory.waypoints)
        {
            if (waypoint.time_from_start.sec < 0 ||
                waypoint.time_from_start.nanosec >= 1000000000U)
            {
                RCLCPP_WARN(get_logger(),
                            "Rejecting trajectory with invalid waypoint timing or pose.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            const double time = rclcpp::Duration(waypoint.time_from_start).seconds();
            const auto &position = waypoint.pose.position;
            const auto &orientation = waypoint.pose.orientation;
            const double quaternion_norm = std::sqrt(
                orientation.x * orientation.x + orientation.y * orientation.y +
                orientation.z * orientation.z + orientation.w * orientation.w);
            if (!std::isfinite(time) || time < 0.0 || time <= previous_time ||
                !std::isfinite(position.x) || !std::isfinite(position.y) ||
                !std::isfinite(quaternion_norm) || quaternion_norm < 1e-6)
            {
                RCLCPP_WARN(get_logger(),
                            "Rejecting trajectory with invalid waypoint timing or pose.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            previous_time = time;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse TrajectoryController::handle_cancel(
        const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle)
    {
        std::lock_guard<std::mutex> lock(m_goal_mutex);
        return m_active_goal == goal_handle ? rclcpp_action::CancelResponse::ACCEPT
                                            : rclcpp_action::CancelResponse::REJECT;
    }

    void TrajectoryController::handle_accepted(
        const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle)
    {
        std::lock_guard<std::mutex> lock(m_goal_mutex);
        if (m_active_goal && m_active_goal->is_active())
        {
            auto result = std::make_shared<FollowTrajectory::Result>();
            result->result = FollowTrajectory::Result::ERROR;
            result->message = "Superseded by a newer trajectory.";
            m_active_goal->abort(result);
        }
        m_pid_x.reset();
        m_pid_y.reset();
        m_pid_yaw.reset();
        m_active_goal = goal_handle;
    }

    void TrajectoryController::control_callback()
    {
        const auto current_time = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(current_time - m_last_control_time).count();
        m_last_control_time = current_time;

        std::shared_ptr<GoalHandleFollowTrajectory> active_goal;
        {
            std::lock_guard<std::mutex> lock(m_goal_mutex);
            active_goal = m_active_goal;
        }
        if (active_goal && active_goal->is_canceling())
        {
            auto result = std::make_shared<FollowTrajectory::Result>();
            result->result = FollowTrajectory::Result::ERROR;
            result->message = "Trajectory canceled.";
            active_goal->canceled(result);
            {
                std::lock_guard<std::mutex> lock(m_goal_mutex);
                if (m_active_goal == active_goal)
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
            m_pid_x.reset();
            m_pid_y.reset();
            m_pid_yaw.reset();
            publish_stop();
            return;
        }

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

        const double transform_age =
            (now() - rclcpp::Time(base_transform.header.stamp, get_clock()->get_clock_type()))
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

        double reference_x = m_target_x;
        double reference_y = m_target_y;
        double reference_yaw = m_target_yaw;
        double feedforward_x = 0.0;
        double feedforward_y = 0.0;
        double feedforward_yaw = 0.0;
        bool trajectory_finished = false;
        double final_x = 0.0;
        double final_y = 0.0;
        double final_yaw = 0.0;
        if (active_goal)
        {
            const auto &trajectory = active_goal->get_goal()->trajectory;
            const auto start_time = rclcpp::Time(
                trajectory.header.stamp, get_clock()->get_clock_type());
            const double elapsed = (now() - start_time).seconds();
            const auto &waypoints = trajectory.waypoints;
            const auto waypoint_time = [](const auto &waypoint)
            {
                return rclcpp::Duration(waypoint.time_from_start).seconds();
            };
            const auto waypoint_yaw = [](const auto &waypoint)
            {
                const auto &q = waypoint.pose.orientation;
                const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
                const double qx = q.x / norm;
                const double qy = q.y / norm;
                const double qz = q.z / norm;
                const double qw = q.w / norm;
                return std::atan2(2.0 * (qw * qz + qx * qy),
                                  1.0 - 2.0 * (qy * qy + qz * qz));
            };
            const auto &last = waypoints.back();
            final_x = last.pose.position.x;
            final_y = last.pose.position.y;
            final_yaw = waypoint_yaw(last);

            size_t next_index = 0;
            while (next_index < waypoints.size() &&
                   waypoint_time(waypoints[next_index]) < elapsed)
            {
                ++next_index;
            }

            if (next_index == 0)
            {
                const auto &first = waypoints.front();
                reference_x = first.pose.position.x;
                reference_y = first.pose.position.y;
                reference_yaw = waypoint_yaw(first);
            }
            else if (next_index >= waypoints.size())
            {
                reference_x = final_x;
                reference_y = final_y;
                reference_yaw = final_yaw;
                trajectory_finished = elapsed >= waypoint_time(last);
            }
            else
            {
                const auto &previous = waypoints[next_index - 1];
                const auto &next = waypoints[next_index];
                const double previous_time = waypoint_time(previous);
                const double next_time = waypoint_time(next);
                const double segment_duration = next_time - previous_time;
                const double fraction = (elapsed - previous_time) / segment_duration;
                const double previous_yaw = waypoint_yaw(previous);
                const double yaw_delta = wrap_angle(waypoint_yaw(next) - previous_yaw);
                reference_x = previous.pose.position.x +
                              fraction * (next.pose.position.x - previous.pose.position.x);
                reference_y = previous.pose.position.y +
                              fraction * (next.pose.position.y - previous.pose.position.y);
                reference_yaw = wrap_angle(previous_yaw + fraction * yaw_delta);
                feedforward_x = (next.pose.position.x - previous.pose.position.x) /
                                segment_duration;
                feedforward_y = (next.pose.position.y - previous.pose.position.y) /
                                segment_duration;
                feedforward_yaw = yaw_delta / segment_duration;
            }
        }

        const double error_x = reference_x - m_pose_x;
        const double error_y = reference_y - m_pose_y;
        const double world_velocity_x = m_pid_x.update(
                                            error_x, dt, m_linear_kp, m_linear_ki, m_linear_kd, m_integral_limit_x) +
                                        feedforward_x;
        const double world_velocity_y = m_pid_y.update(
                                            error_y, dt, m_linear_kp, m_linear_ki, m_linear_kd, m_integral_limit_y) +
                                        feedforward_y;

        const double cosine = std::cos(m_pose_yaw);
        const double sine = std::sin(m_pose_yaw);
        const double error_yaw = wrap_angle(reference_yaw - m_pose_yaw);
        const double velocity_x =
            clamp(cosine * world_velocity_x + sine * world_velocity_y, m_max_vel_x);
        const double velocity_y =
            clamp(-sine * world_velocity_x + cosine * world_velocity_y, m_max_vel_y);
        const double velocity_yaw =
            clamp(m_pid_yaw.update(error_yaw, dt, m_yaw_kp, m_yaw_ki, m_yaw_kd,
                                   m_integral_limit_yaw) +
                      feedforward_yaw,
                  m_max_angular_z);

        geometry_msgs::msg::TwistStamped command;
        command.header.stamp = now();
        command.header.frame_id = m_base_frame;
        command.twist.linear.x = velocity_x;
        command.twist.linear.y = velocity_y;
        command.twist.angular.z = velocity_yaw;
        m_cmd_vel_publisher->publish(command);

        if (active_goal && trajectory_finished &&
            std::hypot(final_x - m_pose_x, final_y - m_pose_y) <= m_position_tolerance &&
            std::abs(wrap_angle(final_yaw - m_pose_yaw)) <= m_yaw_tolerance)
        {
            auto result = std::make_shared<FollowTrajectory::Result>();
            result->result = FollowTrajectory::Result::SUCCESS;
            result->message = "Trajectory completed.";
            active_goal->succeed(result);
            std::lock_guard<std::mutex> lock(m_goal_mutex);
            if (m_active_goal == active_goal)
            {
                m_active_goal.reset();
            }
            m_target_x = final_x;
            m_target_y = final_y;
            m_target_yaw = final_yaw;
            m_target_initialized = true;
            m_pid_x.reset();
            m_pid_y.reset();
            m_pid_yaw.reset();
        }
    }

    void TrajectoryController::publish_stop()
    {
        geometry_msgs::msg::TwistStamped command;
        command.header.stamp = now();
        command.header.frame_id = m_base_frame;
        m_cmd_vel_publisher->publish(command);
    }
} // namespace sfg_go2_trajectory_controller
