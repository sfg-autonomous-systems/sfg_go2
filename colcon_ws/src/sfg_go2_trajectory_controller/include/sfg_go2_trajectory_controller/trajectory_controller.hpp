#pragma once

#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <functional>
#include <memory>
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
            double integral{0.0};
            double previous_error{0.0};
            bool has_previous_error{false};

            void reset();
            double update(double error, double dt, double kp, double ki, double kd, double integral_limit);
        };

        void control_callback();
        void publish_stop();

        std::string m_target_frame;
        std::string m_base_frame;
        double m_target_x;
        double m_target_y;
        double m_target_yaw;
        bool m_target_from_start;
        double m_control_rate;
        double m_pose_timeout;
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
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
            m_cmd_vel_publisher;
        rclcpp::TimerBase::SharedPtr m_control_timer;
    };
} // namespace sfg_go2_trajectory_controller
