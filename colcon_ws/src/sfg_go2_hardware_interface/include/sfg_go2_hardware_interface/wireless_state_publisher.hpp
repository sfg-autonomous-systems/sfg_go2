#pragma once

#include <fstream>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sfg_go2_hardware_interface
{
    class WirelessStatePublisher : public rclcpp::Node
    {
    public:
        explicit WirelessStatePublisher(const rclcpp::NodeOptions &options);

    private:
        bool loadMapFile(const std::string &path);
        void rssiCallback(const std_msgs::msg::Float32::SharedPtr message);
        void publishMap();
        int rssiToCost(float rssi) const;

        // ROS parameters
        std::string m_map_frame_id;
        std::string m_base_frame_id;
        std::string m_rssi_topic;
        std::string m_map_topic;
        std::string m_map_file;
        double m_rssi_min;
        double m_rssi_max;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_rssi_subscription;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_publisher;
        rclcpp::TimerBase::SharedPtr m_publish_timer;
        nav_msgs::msg::OccupancyGrid m_map;
        std::mutex m_map_mutex;

        std::unique_ptr<tf2_ros::Buffer> m_transform_buffer;
        std::unique_ptr<tf2_ros::TransformListener> m_transform_listener;
    };
} // namespace sfg_go2_hardware_interface
