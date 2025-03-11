#pragma once

#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string.h>

class RGBDCamera : public rclcpp::Node
{

public:
    RGBDCamera();

private:
    void camera_callback(const rs2::frame &frame);

    std::string m_frame_id;
    int m_framerate;
    int m_width;
    int m_height;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_rgb_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_depth_publisher;

    sensor_msgs::msg::Image m_rgb_image;
    sensor_msgs::msg::Image m_depth_image;

    rs2::pipeline m_pipeline;
    rs2::config m_config;
    rs2::align m_align;
};