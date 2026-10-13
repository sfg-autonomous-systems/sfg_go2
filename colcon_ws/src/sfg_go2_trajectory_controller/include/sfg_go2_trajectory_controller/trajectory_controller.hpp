#pragma once

#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sfg_agent_msgs/action/follow_trajectory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>

namespace sfg_go2_trajectory_controller
{
    class TrajectoryController : public rclcpp::Node
    {
    public:
        TrajectoryController(const rclcpp::NodeOptions &options);

    private:
        using FollowTrajectory = sfg_agent_msgs::action::FollowTrajectory;
        using GoalHandleFollowTrajectory =
            rclcpp_action::ServerGoalHandle<FollowTrajectory>;

        struct PIDState
        {
            double integral{0.0};
            double previous_error{0.0};
            bool has_previous_error{false};

            void reset();
            double update(
                double error,
                double dt,
                double kp,
                double ki,
                double kd,
                double integral_limit);
        };

        struct TrajectoryReference
        {
            double x{0.0};
            double y{0.0};
            double yaw{0.0};
            double velocity_x{0.0};
            double velocity_y{0.0};
            double velocity_yaw{0.0};
            double final_x{0.0};
            double final_y{0.0};
            double final_yaw{0.0};
            bool finished{false};
        };

        void control_callback();
        bool handle_canceling_goal(
            const std::shared_ptr<GoalHandleFollowTrajectory> &goal_handle);
        bool update_robot_pose();
        TrajectoryReference get_trajectory_reference(
            const std::shared_ptr<GoalHandleFollowTrajectory> &goal_handle) const;
        void reset_pid();
        void stop_and_reset();
        void complete_trajectory(
            const std::shared_ptr<GoalHandleFollowTrajectory> &goal_handle,
            const TrajectoryReference &reference);
        void publish_stop();
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const FollowTrajectory::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle);
        void handle_accepted(
            const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle);

        std::string m_target_frame;
        std::string m_base_frame;
        std::string m_action_name;
        double m_target_x{0.0};
        double m_target_y{0.0};
        double m_target_yaw{0.0};
        bool m_target_from_start{true};
        double m_control_rate{30.0};
        double m_pose_timeout{0.5};
        double m_max_vel_x{0.3};
        double m_max_vel_y{0.3};
        double m_max_angular_z{0.6};
        double m_linear_kp{0.8};
        double m_linear_ki{0.0};
        double m_linear_kd{0.0};
        double m_yaw_kp{1.2};
        double m_yaw_ki{0.0};
        double m_yaw_kd{0.0};
        double m_integral_limit_x{1.0};
        double m_integral_limit_y{1.0};
        double m_integral_limit_yaw{1.0};
        double m_position_tolerance{0.05};
        double m_yaw_tolerance{0.1};

        bool m_target_initialized{false};
        double m_pose_x{0.0};
        double m_pose_y{0.0};
        double m_pose_yaw{0.0};
        std::chrono::steady_clock::time_point m_last_control_time;

        PIDState m_pid_x;
        PIDState m_pid_y;
        PIDState m_pid_yaw;

        std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
        rclcpp_action::Server<FollowTrajectory>::SharedPtr m_action_server;
        std::shared_ptr<GoalHandleFollowTrajectory> m_active_goal;
        std::mutex m_goal_mutex;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
            m_cmd_vel_publisher;
        rclcpp::TimerBase::SharedPtr m_control_timer;
    };
} // namespace sfg_go2_trajectory_controller
