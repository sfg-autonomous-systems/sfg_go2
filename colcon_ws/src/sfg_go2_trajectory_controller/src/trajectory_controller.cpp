#include "sfg_go2_trajectory_controller/trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace sfg_go2_trajectory_controller
{
    namespace
    {
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

    double TrajectoryController::PIDState::update(
        double error,
        double dt,
        double kp,
        double ki,
        double kd,
        double integral_limit)
    {
        integral = clamp(integral + error * dt, integral_limit);
        const double derivative = has_previous_error
                                      ? (error - previous_error) / dt
                                      : 0.0;
        previous_error = error;
        has_previous_error = true;
        return kp * error + ki * integral + kd * derivative;
    }

    TrajectoryController::TrajectoryController(const rclcpp::NodeOptions &options)
        : Node("trajectory_controller", options)
    {
        m_target_frame = declare_parameter<std::string>("target_frame", "map");
        m_base_frame = declare_parameter<std::string>(
            "base_frame", "sfg_go2_01/base_link");
        m_action_name = declare_parameter<std::string>(
            "action_name", "follow_trajectory");
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
        m_integral_limit_yaw = declare_parameter<double>(
            "integral_limit_yaw", 1.0);
        m_position_tolerance = declare_parameter<double>(
            "position_tolerance", 0.05);
        m_yaw_tolerance = declare_parameter<double>("yaw_tolerance", 0.1);

        if (!std::isfinite(m_control_rate) || m_control_rate <= 0.0 ||
            !std::isfinite(m_pose_timeout) || m_pose_timeout <= 0.0 ||
            !std::isfinite(m_max_vel_x) || m_max_vel_x <= 0.0 ||
            !std::isfinite(m_max_vel_y) || m_max_vel_y <= 0.0 ||
            !std::isfinite(m_max_angular_z) || m_max_angular_z <= 0.0 ||
            !std::isfinite(m_position_tolerance) ||
            m_position_tolerance <= 0.0 ||
            !std::isfinite(m_yaw_tolerance) || m_yaw_tolerance <= 0.0)
        {
            throw std::invalid_argument(
                "Controller rates, limits, timeouts, and tolerances must be "
                "positive and finite.");
        }

        m_target_initialized = !m_target_from_start;
        m_cmd_vel_publisher = create_publisher<geometry_msgs::msg::TwistStamped>(
            "cmd_vel", rclcpp::SystemDefaultsQoS());
        m_action_server = rclcpp_action::create_server<FollowTrajectory>(
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
        m_tf_listener =
            std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
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
        const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle)
    {
        std::lock_guard<std::mutex> lock(m_goal_mutex);
        return m_active_goal == goal_handle
                   ? rclcpp_action::CancelResponse::ACCEPT
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
        reset_pid();
        m_active_goal = goal_handle;
    }

    void TrajectoryController::reset_pid()
    {
        m_pid_x.reset();
        m_pid_y.reset();
        m_pid_yaw.reset();
    }
} // namespace sfg_go2_trajectory_controller
