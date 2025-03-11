#include "go2/rgbd_camera.h"

#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>

RGBDCamera::RGBDCamera() : rclcpp::Node("rgbd_camera"), m_align(RS2_STREAM_COLOR)
{
    this->declare_parameter("frame_id", "rgbd_camera");
    this->declare_parameter("framerate", 30);
    this->declare_parameter("width", 640);
    this->declare_parameter("height", 480);

    this->get_parameter("frame_id", m_frame_id);
    this->get_parameter("framerate", m_framerate);
    this->get_parameter("width", m_width);
    this->get_parameter("height", m_height);

    m_rgb_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera/rgb", 10);
    m_depth_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera/depth", 10);

    m_config.enable_stream(RS2_STREAM_COLOR, m_width, m_height, RS2_FORMAT_BGR8, m_framerate);
    m_config.enable_stream(RS2_STREAM_DEPTH, m_width, m_height, RS2_FORMAT_Z16, m_framerate);
    m_pipeline.start(m_config, std::bind(&RGBDCamera::camera_callback, this, std::placeholders::_1));
}

void RGBDCamera::camera_callback(const rs2::frame &frame)
{
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = m_frame_id;

    if (auto frameset = frame.as<rs2::frameset>())
    {
        frameset = m_align.process(frameset);

        auto rgb_frame = frameset.get_color_frame();
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGRA8, cv::Mat(cv::Size(m_width, m_height), CV_8UC3, (void *)rgb_frame.get_data(), cv::Mat::AUTO_STEP)).toImageMsg(m_rgb_image);
        m_rgb_image.header = header;

        auto depth_frame = frameset.get_depth_frame();
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, cv::Mat(cv::Size(m_width, m_height), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP)).toImageMsg(m_depth_image);
        m_depth_image.header = header;

        m_rgb_publisher->publish(m_rgb_image);
        m_depth_publisher->publish(m_depth_image);
    }
}
