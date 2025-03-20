#include "go2/rgbd_camera.h"

#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>

RGBDCamera::RGBDCamera(rclcpp::Node::SharedPtr node) : m_node(node), m_image_transport(node)
{
    m_node->declare_parameter("frame_id", "rgbd_camera");
    m_node->declare_parameter("framerate", 15);
    m_node->declare_parameter("width", 1280);
    m_node->declare_parameter("height", 720);

    m_node->get_parameter("frame_id", m_frame_id);
    m_node->get_parameter("framerate", m_framerate);
    m_node->get_parameter("width", m_width);
    m_node->get_parameter("height", m_height);

    m_rgb_publisher = m_image_transport.advertise("camera/rgb", 10);
    m_depth_publisher = m_image_transport.advertise("camera/depth", 10);

    m_config.enable_stream(RS2_STREAM_COLOR, m_width, m_height, RS2_FORMAT_BGR8, m_framerate);
    m_config.enable_stream(RS2_STREAM_DEPTH, m_width, m_height, RS2_FORMAT_Z16, m_framerate);
    m_pipeline.start(m_config, std::bind(&RGBDCamera::camera_callback, this, std::placeholders::_1));
}

RGBDCamera::~RGBDCamera()
{
    m_pipeline.stop();
}

void RGBDCamera::camera_callback(const rs2::frame &frame)
{
    std_msgs::msg::Header header;
    header.stamp = m_node->now();
    header.frame_id = m_frame_id;

    if (auto frameset = frame.as<rs2::frameset>())
    {
        auto rgb_frame = frameset.get_color_frame();
        auto rgb_mat = cv::Mat(cv::Size(m_width, m_height), CV_8UC3, (void *)rgb_frame.get_data(), cv::Mat::AUTO_STEP);
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, rgb_mat).toImageMsg(m_rgb_image);
        m_rgb_image.header = header;

        auto depth_frame = frameset.get_depth_frame();
        auto depth_mat = cv::Mat(cv::Size(m_width, m_height), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, depth_mat).toImageMsg(m_depth_image);
        m_depth_image.header = header;

        m_rgb_publisher.publish(m_rgb_image);
        m_depth_publisher.publish(m_depth_image);

        RCLCPP_INFO(m_node->get_logger(), "Published RGB and Depth images");
    }
}
