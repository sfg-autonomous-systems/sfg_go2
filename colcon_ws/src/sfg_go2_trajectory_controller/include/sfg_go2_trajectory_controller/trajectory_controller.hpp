#pragma once

#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sfg_agent_msgs/action/follow_trajectory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstddef>
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
        struct PIDState
        {
            double integral;
            double previous_error;
            bool has_previous_error;

            void reset();
            double update(double error, double dt, double kp, double ki, double kd, double setpoint_limit);
        };

        struct TrajectoryReference
        {
            double x;
            double y;
            double yaw;
            double velocity_x;
            double velocity_y;
            double velocity_yaw;
            double final_x;
            double final_y;
            double final_yaw;
            bool finished;
        };

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const sfg_agent_msgs::action::FollowTrajectory::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sfg_agent_msgs::action::FollowTrajectory>> goal_handle);
        void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sfg_agent_msgs::action::FollowTrajectory>> goal_handle);
        void control_callback();
        bool handle_canceling_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sfg_agent_msgs::action::FollowTrajectory>> &goal_handle);
        bool update_robot_pose();
        TrajectoryReference get_trajectory_reference(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sfg_agent_msgs::action::FollowTrajectory>> &goal_handle);
        void complete_trajectory(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<sfg_agent_msgs::action::FollowTrajectory>> &goal_handle,
            const TrajectoryReference &reference);
        void stop_and_reset();
        void reset_pid();
        void publish_stop();

        // ROS parameters
        std::string m_target_frame;
        std::string m_base_frame;
        std::string m_action_name;
        double m_target_x;
        double m_target_y;
        double m_target_yaw;
        bool m_target_from_start;
        double m_control_rate;
        double m_pose_timeout;
        double m_path_lookahead_distance;
        double m_max_vel_x;
        double m_max_vel_y;
        double m_max_angular_z;
        double m_linear_kp;
        double m_linear_ki;
        double m_linear_kd;
        double m_yaw_kp;
        double m_yaw_ki;
        double m_yaw_kd;
        double m_integral_limit_x;
        double m_integral_limit_y;
        double m_integral_limit_yaw;
        double m_position_tolerance;
        double m_yaw_tolerance;

        bool m_target_initialized;
        double m_pose_x;
        double m_pose_y;
        double m_pose_yaw;
        std::chrono::steady_clock::time_point m_last_control_time;

        PIDState m_pid_x;
        PIDState m_pid_y;
        PIDState m_pid_yaw;

        std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
        rclcpp_action::Server<sfg_agent_msgs::action::FollowTrajectory>::SharedPtr m_action_server;
        std::shared_ptr<rclcpp_action::ServerGoalHandle<sfg_agent_msgs::action::FollowTrajectory>> m_active_goal;
        std::size_t m_active_waypoint_index;
        std::mutex m_goal_mutex;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_cmd_vel_publisher;
        rclcpp::TimerBase::SharedPtr m_control_timer;
    };
} // namespace sfg_go2_trajectory_controller
