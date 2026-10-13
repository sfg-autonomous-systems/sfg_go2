#include "sfg_go2_trajectory_controller/trajectory_controller.hpp"

#include <algorithm>
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
        const std::shared_ptr<GoalHandleFollowTrajectory> &goal_handle)
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
        const auto &waypoints = trajectory.waypoints;
        const auto waypoint_time = [](const auto &waypoint)
        {
            return rclcpp::Duration(waypoint.time_from_start).seconds();
        };

        const auto &last = waypoints.back();
        reference.final_x = last.pose.position.x;
        reference.final_y = last.pose.position.y;
        reference.final_yaw = waypoint_yaw(last);

        std::size_t active_index;
        {
            std::lock_guard<std::mutex> lock(m_goal_mutex);
            if (m_active_goal != goal_handle)
            {
                return reference;
            }

            active_index = m_active_waypoint_index;
            if (active_index + 1 < waypoints.size())
            {
                const auto &active = waypoints[active_index].pose.position;
                const double distance_to_active = std::hypot(
                    m_pose_x - active.x, m_pose_y - active.y);
                bool advance =
                    distance_to_active <= m_position_tolerance;

                if (!advance && active_index > 0)
                {
                    const auto &previous =
                        waypoints[active_index - 1].pose.position;
                    const auto &next =
                        waypoints[active_index + 1].pose.position;
                    const double segment_x = active.x - previous.x;
                    const double segment_y = active.y - previous.y;
                    const double segment_length_squared =
                        segment_x * segment_x + segment_y * segment_y;
                    if (segment_length_squared > 1e-12)
                    {
                        const double projection =
                            ((m_pose_x - previous.x) * segment_x +
                             (m_pose_y - previous.y) * segment_y) /
                            segment_length_squared;
                        const double distance_to_next = std::hypot(
                            m_pose_x - next.x, m_pose_y - next.y);
                        advance = projection >= 1.0 &&
                                  distance_to_next < distance_to_active;
                    }
                }

                if (advance)
                {
                    ++active_index;
                    m_active_waypoint_index = active_index;
                }
            }
        }

        const double elapsed =
            (now() - rclcpp::Time(
                         trajectory.header.stamp,
                         get_clock()->get_clock_type()))
                .seconds();
        if (active_index == 0)
        {
            const auto &first = waypoints.front();
            reference.x = first.pose.position.x;
            reference.y = first.pose.position.y;
            reference.yaw = waypoint_yaw(first);
            reference.finished =
                waypoints.size() == 1 &&
                elapsed >= waypoint_time(first);
            return reference;
        }

        const auto &previous = waypoints[active_index - 1];
        const auto &next = waypoints[active_index];
        const auto &previous_position = previous.pose.position;
        const auto &next_position = next.pose.position;
        const double previous_time = waypoint_time(previous);
        const double next_time = waypoint_time(next);
        const double segment_duration = next_time - previous_time;
        const double segment_x = next_position.x - previous_position.x;
        const double segment_y = next_position.y - previous_position.y;
        const double segment_length = std::hypot(segment_x, segment_y);
        const double previous_yaw = waypoint_yaw(previous);
        const double yaw_delta =
            wrap_angle(waypoint_yaw(next) - previous_yaw);

        if (segment_length > 1e-6)
        {
            const double projection = std::clamp(
                ((m_pose_x - previous_position.x) * segment_x +
                 (m_pose_y - previous_position.y) * segment_y) /
                    (segment_length * segment_length),
                0.0,
                1.0);
            const double distance_along = std::min(
                projection * segment_length + m_path_lookahead_distance,
                segment_length);
            const double fraction = distance_along / segment_length;
            reference.x = previous_position.x + fraction * segment_x;
            reference.y = previous_position.y + fraction * segment_y;
            reference.yaw =
                wrap_angle(previous_yaw + fraction * yaw_delta);
        }
        else
        {
            reference.x = next_position.x;
            reference.y = next_position.y;
            reference.yaw = waypoint_yaw(next);
        }

        if (elapsed >= previous_time && elapsed < next_time)
        {
            reference.velocity_x = segment_x / segment_duration;
            reference.velocity_y = segment_y / segment_duration;
            reference.velocity_yaw = yaw_delta / segment_duration;
        }

        if (active_index + 1 == waypoints.size())
        {
            reference.finished = elapsed >= next_time;
        }
        return reference;
    }
} // namespace sfg_go2_trajectory_controller
