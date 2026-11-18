#include "sfg_go2_hardware_interface/wireless_state_publisher.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include "tf2/exceptions.h"

namespace sfg_go2_hardware_interface
{
    WirelessStatePublisher::WirelessStatePublisher(const rclcpp::NodeOptions &options) : Node("wireless_state_publisher", options),
                                                                                         m_transform_buffer(std::make_unique<tf2_ros::Buffer>(get_clock())),
                                                                                         m_transform_listener(std::make_unique<tf2_ros::TransformListener>(*m_transform_buffer))
    {
        m_map_frame_id = declare_parameter("map_frame_id", "map");
        m_base_frame_id = declare_parameter("base_frame_id", "sfg_go2_01/base_link");
        m_rssi_topic = declare_parameter("rssi_topic", "/local/sfg_go2_01/rssi");
        m_map_topic = declare_parameter("map_topic", "/global/sfg_go2_01/qos_map");
        m_map_file = declare_parameter("map_file", "");
        m_rssi_min = declare_parameter("min_rssi", -90.0);
        m_rssi_max = declare_parameter("max_rssi", -30.0);

        if (m_rssi_min >= m_rssi_max)
        {
            throw std::invalid_argument("min_rssi must be smaller than max_rssi");
        }

        if (m_map_file.empty())
        {
            throw std::invalid_argument("map_file must point to the QoS foundation map");
        }

        if (!loadMapFile(m_map_file))
        {
            throw std::runtime_error("Could not load QoS foundation map: " + m_map_file);
        }

        m_rssi_subscription = create_subscription<std_msgs::msg::Float32>(m_rssi_topic, rclcpp::SensorDataQoS(), std::bind(&WirelessStatePublisher::rssiCallback, this, std::placeholders::_1));

        m_map_publisher = create_publisher<nav_msgs::msg::OccupancyGrid>(m_map_topic, rclcpp::QoS(1).transient_local().reliable());

        m_publish_timer = create_wall_timer(std::chrono::seconds(1), std::bind(&WirelessStatePublisher::publishMap, this));
    }

    bool WirelessStatePublisher::loadMapFile(const std::string &path)
    {
        std::ifstream input(path);
        if (!input)
        {
            return false;
        }

        std::string header;
        if (!std::getline(input, header))
        {
            return false;
        }
        std::replace(header.begin(), header.end(), ',', ' ');
        std::istringstream header_stream(header);

        unsigned int width;
        unsigned int height;
        double resolution;
        double origin_x;
        double origin_y;

        if (!(header_stream >> width >> height >> resolution >> origin_x >> origin_y) || width == 0 || height == 0 || resolution <= 0.0)
        {
            return false;
        }

        std::ostringstream values_stream;
        values_stream << input.rdbuf();
        std::string values_text = values_stream.str();
        std::replace(values_text.begin(), values_text.end(), ',', ' ');
        std::istringstream values(values_text);

        nav_msgs::msg::OccupancyGrid loaded_map;
        loaded_map.header.frame_id = m_map_frame_id;
        loaded_map.info.width = width;
        loaded_map.info.height = height;
        loaded_map.info.resolution = static_cast<float>(resolution);
        loaded_map.info.origin.position.x = origin_x;
        loaded_map.info.origin.position.y = origin_y;
        loaded_map.info.origin.orientation.w = 1.0;
        loaded_map.data.reserve(static_cast<std::size_t>(width) * height);

        int value;
        while (values >> value)
        {
            if (value < -1 || value > 100)
            {
                return false;
            }
            loaded_map.data.push_back(static_cast<int8_t>(value));
        }
        if (loaded_map.data.size() != static_cast<std::size_t>(width) * height)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(m_map_mutex);
        m_map = std::move(loaded_map);
        return true;
    }

    void WirelessStatePublisher::rssiCallback(const std_msgs::msg::Float32::SharedPtr message)
    {
        if (!message || !std::isfinite(message->data))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring non-finite RSSI measurement");
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = m_transform_buffer->lookupTransform(m_map_frame_id, m_base_frame_id, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &exception)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Could not transform %s to %s: %s", m_base_frame_id.c_str(), m_map_frame_id.c_str(), exception.what());
            return;
        }

        {
            std::lock_guard<std::mutex> lock(m_map_mutex);
            unsigned int map_x;
            unsigned int map_y;

            if (!m_map.info.width || !m_map.info.height || m_map.info.resolution <= 0.0f)
            {
                return;
            }

            const double world_x = transform.transform.translation.x;
            const double world_y = transform.transform.translation.y;
            const double map_min_x = m_map.info.origin.position.x;
            const double map_min_y = m_map.info.origin.position.y;
            const double map_max_x = map_min_x + m_map.info.width * m_map.info.resolution;
            const double map_max_y = map_min_y + m_map.info.height * m_map.info.resolution;

            if (world_x < map_min_x || world_x >= map_max_x || world_y < map_min_y || world_y >= map_max_y)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Robot position (%.2f, %.2f) is outside the QoS map", world_x, world_y);
                return;
            }

            map_x = static_cast<unsigned int>((world_x - map_min_x) / m_map.info.resolution);
            map_y = static_cast<unsigned int>((world_y - map_min_y) / m_map.info.resolution);
            const auto index = static_cast<std::size_t>(map_y) * m_map.info.width + map_x;
            m_map.data[index] = static_cast<int8_t>(rssiToCost(message->data));
            m_map.header.stamp = get_clock()->now();
        }

        publishMap();
    }

    void WirelessStatePublisher::publishMap()
    {
        std::lock_guard<std::mutex> lock(m_map_mutex);

        if (m_map.data.empty())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "QoS map is empty");
            return;
        }

        m_map.header.stamp = get_clock()->now();
        m_map_publisher->publish(m_map);
    }

    int WirelessStatePublisher::rssiToCost(float rssi) const
    {
        const double normalized = std::clamp((static_cast<double>(rssi) - m_rssi_max) / (m_rssi_min - m_rssi_max), 0.0, 1.0);
        return static_cast<int>(std::lround(normalized * 100.0));
    }

} // namespace sfg_go2_hardware_interface
