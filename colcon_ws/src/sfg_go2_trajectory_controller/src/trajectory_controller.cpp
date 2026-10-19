#include "sfg_go2_trajectory_controller/trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
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

    void TrajectoryController::PIDState::reset()
    {
        integral = 0.0;
        previous_error = 0.0;
        has_previous_error = false;
    }

    double TrajectoryController::PIDState::update(double error, double dt, double kp, double ki, double kd, double setpoint_limit)
    {
        const double derivative = has_previous_error ? (error - previous_error) / dt : 0.0;
        previous_error = error;
        has_previous_error = true;
        double output = kp * error + ki * integral + kd * derivative;
        if (std::abs(output) < setpoint_limit)
        {
            integral += error * dt;
        }
        return kp * error + ki * integral + kd * derivative;
    }

    TrajectoryController::TrajectoryController(const rclcpp::NodeOptions &options)
        : Node("trajectory_controller", options)
    {
        m_target_frame = declare_parameter<std::string>("target_frame");
        m_base_frame = declare_parameter<std::string>("base_frame");
        m_action_name = declare_parameter<std::string>("action_name");
        m_target_x = declare_parameter<double>("target_x");
        m_target_y = declare_parameter<double>("target_y");
        m_target_yaw = declare_parameter<double>("target_yaw");
        m_target_from_start = declare_parameter<bool>("target_from_start");
        m_control_rate = declare_parameter<double>("control_rate");
        m_pose_timeout = declare_parameter<double>("pose_timeout");
        m_path_lookahead_distance = declare_parameter<double>("path_lookahead_distance");
        m_max_vel_x = declare_parameter<double>("max_vel_x");
        m_max_vel_y = declare_parameter<double>("max_vel_y");
        m_max_angular_z = declare_parameter<double>("max_angular_z");
        m_linear_kp = declare_parameter<double>("linear_kp");
        m_linear_ki = declare_parameter<double>("linear_ki");
        m_linear_kd = declare_parameter<double>("linear_kd");
        m_yaw_kp = declare_parameter<double>("yaw_kp");
        m_yaw_ki = declare_parameter<double>("yaw_ki");
        m_yaw_kd = declare_parameter<double>("yaw_kd");
        m_integral_limit_x = declare_parameter<double>("integral_limit_x");
        m_integral_limit_y = declare_parameter<double>("integral_limit_y");
        m_integral_limit_yaw = declare_parameter<double>("integral_limit_yaw");
        m_position_tolerance = declare_parameter<double>("position_tolerance");
        m_yaw_tolerance = declare_parameter<double>("yaw_tolerance");

        m_target_initialized = !m_target_from_start;
        m_cmd_vel_publisher = create_publisher<geometry_msgs::msg::TwistStamped>(RosFqnBuilder().resource(Resource::CmdVel).build(RosFqnSegment::Resource), rclcpp::SystemDefaultsQoS());
        m_action_server = rclcpp_action::create_server<sfg_agent_msgs::action::FollowTrajectory>(
            this,
            m_action_name,
            std::bind(
                &TrajectoryController::handle_goal,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            std::bind(
                &TrajectoryController::handle_cancel,
                this,
                std::placeholders::_1),
            std::bind(
                &TrajectoryController::handle_accepted,
                this,
                std::placeholders::_1));

        m_tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
        m_control_timer = create_wall_timer(
            std::chrono::duration<double>(1.0 / m_control_rate),
            std::bind(&TrajectoryController::control_callback, this));
        m_last_control_time = std::chrono::steady_clock::now();

        RCLCPP_INFO(
            get_logger(),
            "Trajectory controller reading TF %s -> %s and publishing "
            "locomotion commands on 'cmd_vel'; serving action '%s'.",
            m_target_frame.c_str(),
            m_base_frame.c_str(),
            m_action_name.c_str());
    }

    rclcpp_action::GoalResponse TrajectoryController::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const sfg_agent_msgs::action::FollowTrajectory::Goal> goal)
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
            RCLCPP_WARN(
                get_logger(),
                "Rejecting trajectory in frame '%s'; expected '%s'.",
                trajectory.header.frame_id.c_str(),
                m_target_frame.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (trajectory.header.stamp.sec < 0 ||
            trajectory.header.stamp.nanosec >= 1000000000U)
        {
            RCLCPP_WARN(
                get_logger(),
                "Rejecting trajectory with an invalid start timestamp.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        double previous_time = -1.0;
        for (const auto &waypoint : trajectory.waypoints)
        {
            if (waypoint.time_from_start.sec < 0 ||
                waypoint.time_from_start.nanosec >= 1000000000U)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Rejecting trajectory with invalid waypoint timing or pose.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            const double time =
                rclcpp::Duration(waypoint.time_from_start).seconds();
            const auto &position = waypoint.pose.position;
            const auto &orientation = waypoint.pose.orientation;
            const double quaternion_norm = std::sqrt(
                orientation.x * orientation.x +
                orientation.y * orientation.y +
                orientation.z * orientation.z +
                orientation.w * orientation.w);
            if (!std::isfinite(time) || time < 0.0 || time <= previous_time ||
                !std::isfinite(position.x) || !std::isfinite(position.y) ||
                !std::isfinite(quaternion_norm) || quaternion_norm < 1e-6)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Rejecting trajectory with invalid waypoint timing or pose.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            previous_time = time;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse TrajectoryController::handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<sfg_agent_msgs::action::FollowTrajectory>> goal_handle)
    {
        std::lock_guard<std::mutex> lock(m_goal_mutex);
        return m_active_goal == goal_handle
                   ? rclcpp_action::CancelResponse::ACCEPT
                   : rclcpp_action::CancelResponse::REJECT;
    }

    void TrajectoryController::handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<sfg_agent_msgs::action::FollowTrajectory>> goal_handle)
    {
        std::lock_guard<std::mutex> lock(m_goal_mutex);
        if (m_active_goal && m_active_goal->is_active())
        {
            auto result = std::make_shared<sfg_agent_msgs::action::FollowTrajectory::Result>();
            result->result = sfg_agent_msgs::action::FollowTrajectory::Result::ERROR;
            result->message = "Superseded by a newer trajectory.";
            m_active_goal->abort(result);
        }
        reset_pid();
        m_active_waypoint_index = 0;
        m_active_goal = goal_handle;
    }

    void TrajectoryController::control_callback()
    {
        const auto current_time = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(current_time - m_last_control_time).count();
        m_last_control_time = current_time;

        std::shared_ptr<rclcpp_action::ServerGoalHandle<sfg_agent_msgs::action::FollowTrajectory>> active_goal;
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
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<sfg_agent_msgs::action::FollowTrajectory>> &goal_handle)
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

    void TrajectoryController::complete_trajectory(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<sfg_agent_msgs::action::FollowTrajectory>> &goal_handle,
        const TrajectoryReference &reference)
    {
        auto result = std::make_shared<sfg_agent_msgs::action::FollowTrajectory::Result>();
        result->result = sfg_agent_msgs::action::FollowTrajectory::Result::SUCCESS;
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

    void TrajectoryController::stop_and_reset()
    {
        reset_pid();
        publish_stop();
    }

    void TrajectoryController::reset_pid()
    {
        m_pid_x.reset();
        m_pid_y.reset();
        m_pid_yaw.reset();
    }

    void TrajectoryController::publish_stop()
    {
        geometry_msgs::msg::TwistStamped command;
        command.header.stamp = now();
        command.header.frame_id = m_base_frame;
        m_cmd_vel_publisher->publish(command);
    }
} // namespace sfg_go2_trajectory_controller
