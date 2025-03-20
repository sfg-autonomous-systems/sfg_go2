#pragma once

#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string.h>
#include <image_transport/image_transport.hpp>

class RGBDCamera
{

public:
    RGBDCamera(rclcpp::Node::SharedPtr node);
    ~RGBDCamera();

private:
    void camera_callback(const rs2::frame &frame);

    rclcpp::Node::SharedPtr m_node;

    std::string m_frame_id;
    int m_framerate;
    int m_width;
    int m_height;

    image_transport::ImageTransport m_image_transport;
    image_transport::Publisher m_rgb_publisher;
    image_transport::Publisher m_depth_publisher;

    sensor_msgs::msg::Image m_rgb_image;
    sensor_msgs::msg::Image m_depth_image;

    rs2::pipeline m_pipeline;
    rs2::config m_config;
};