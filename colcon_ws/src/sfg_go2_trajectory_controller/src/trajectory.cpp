#include "sfg_go2_trajectory_controller/trajectory_controller.hpp"

#include <cmath>
#include <sfg_agent_msgs/msg/waypoint.hpp>

namespace sfg_go2_trajectory_controller
{
    namespace
    {
        double wrap_angle(double angle)
        {
            return std::atan2(std::sin(angle), std::cos(angle));
        }

        double waypoint_yaw(const sfg_agent_msgs::msg::Waypoint &waypoint)
        {
            const auto &orientation = waypoint.pose.orientation;
            const double quaternion_norm = std::sqrt(
                orientation.x * orientation.x +
                orientation.y * orientation.y +
                orientation.z * orientation.z +
                orientation.w * orientation.w);
            const double qx = orientation.x / quaternion_norm;
            const double qy = orientation.y / quaternion_norm;
            const double qz = orientation.z / quaternion_norm;
            const double qw = orientation.w / quaternion_norm;
            return std::atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz));
        }
    } // namespace

    TrajectoryController::TrajectoryReference
    TrajectoryController::get_trajectory_reference(
        const std::shared_ptr<GoalHandleFollowTrajectory> &goal_handle) const
    {
        TrajectoryReference reference;
        reference.x = m_target_x;
        reference.y = m_target_y;
        reference.yaw = m_target_yaw;
        if (!goal_handle)
        {
            return reference;
        }

        const auto &trajectory = goal_handle->get_goal()->trajectory;
        const auto start_time = rclcpp::Time(
            trajectory.header.stamp, get_clock()->get_clock_type());
        const double elapsed = (now() - start_time).seconds();
        const auto &waypoints = trajectory.waypoints;
        const auto waypoint_time = [](const auto &waypoint)
        {
            return rclcpp::Duration(waypoint.time_from_start).seconds();
        };

        const auto &last = waypoints.back();
        reference.final_x = last.pose.position.x;
        reference.final_y = last.pose.position.y;
        reference.final_yaw = waypoint_yaw(last);

        size_t next_index = 0;
        while (next_index < waypoints.size() &&
               waypoint_time(waypoints[next_index]) < elapsed)
        {
            ++next_index;
        }

        if (next_index == 0)
        {
            const auto &first = waypoints.front();
            reference.x = first.pose.position.x;
            reference.y = first.pose.position.y;
            reference.yaw = waypoint_yaw(first);
        }
        else if (next_index >= waypoints.size())
        {
            reference.x = reference.final_x;
            reference.y = reference.final_y;
            reference.yaw = reference.final_yaw;
            reference.finished = elapsed >= waypoint_time(last);
        }
        else
        {
            const auto &previous = waypoints[next_index - 1];
            const auto &next = waypoints[next_index];
            const double previous_time = waypoint_time(previous);
            const double next_time = waypoint_time(next);
            const double segment_duration = next_time - previous_time;
            const double fraction =
                (elapsed - previous_time) / segment_duration;
            const double previous_yaw = waypoint_yaw(previous);
            const double yaw_delta =
                wrap_angle(waypoint_yaw(next) - previous_yaw);

            reference.x = previous.pose.position.x +
                          fraction *
                              (next.pose.position.x -
                               previous.pose.position.x);
            reference.y = previous.pose.position.y +
                          fraction *
                              (next.pose.position.y -
                               previous.pose.position.y);
            reference.yaw =
                wrap_angle(previous_yaw + fraction * yaw_delta);
            reference.velocity_x =
                (next.pose.position.x - previous.pose.position.x) /
                segment_duration;
            reference.velocity_y =
                (next.pose.position.y - previous.pose.position.y) /
                segment_duration;
            reference.velocity_yaw = yaw_delta / segment_duration;
        }
        return reference;
    }
} // namespace sfg_go2_trajectory_controller
