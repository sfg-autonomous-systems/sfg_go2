#include "sfg_go2_rssi_costmap_plugin/rssi_layer.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace sfg_go2_rssi_costmap_plugin
{

    void RssiLayer::onInitialize()
    {
        auto node = node_.lock();
        if (!node)
        {
            throw std::runtime_error("RSSI layer could not access its lifecycle node");
        }

        enabled_ = node->declare_parameter("enabled", true);
        m_map_topic = node->declare_parameter("map_topic", "/global/sfg_go2_01/rssi_map");
        m_max_cost = node->declare_parameter("max_cost", 200);

        if (m_max_cost < 0 || m_max_cost > 200)
        {
            throw std::invalid_argument("RSSI layer max_cost must be in [0, 200]");
        }

        m_map_subscription = node->create_subscription<nav_msgs::msg::OccupancyGrid>(m_map_topic, rclcpp::QoS(1).transient_local().reliable(), std::bind(&RssiLayer::mapCallback, this, std::placeholders::_1));

        current_ = true;
    }

    void RssiLayer::matchSize()
    {
        auto master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());

        std::lock_guard<std::mutex> lock(m_mutex);
        m_need_recalculation = true;
    }

    void RssiLayer::updateBounds(double /* robot_x */, double /* robot_y */, double /* robot_yaw */, double *min_x, double *min_y, double *max_x, double *max_y)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!enabled_ || !m_need_recalculation || !m_map)
        {
            return;
        }

        *min_x = std::min(*min_x, m_map_min_x);
        *min_y = std::min(*min_y, m_map_min_y);
        *max_x = std::max(*max_x, m_map_max_x);
        *max_y = std::max(*max_y, m_map_max_y);
    }

    void RssiLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_need_recalculation)
        {
            rebuildQosCost();
            m_need_recalculation = false;
        }

        // Merge only the layer's grid. Other layers remain responsible for their
        // own costs and updateWithMax() preserves the layer ordering semantics.
        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    }

    void RssiLayer::rebuildQosCost()
    {
        const auto cell_count = getSizeInCellsX() * getSizeInCellsY();
        std::fill_n(getCharMap(), cell_count, nav2_costmap_2d::FREE_SPACE);

        if (!m_map)
        {
            return;
        }

        const auto &source = *m_map;
        for (unsigned int source_y = 0; source_y < source.info.height; ++source_y)
        {
            for (unsigned int source_x = 0; source_x < source.info.width; ++source_x)
            {
                const auto source_index = static_cast<std::size_t>(source_y) * source.info.width + source_x;
                const int value = source.data[source_index];

                if (value < 0)
                {
                    continue;
                }

                const double world_x = source.info.origin.position.x + (source_x + 0.5) * source.info.resolution;
                const double world_y = source.info.origin.position.y + (source_y + 0.5) * source.info.resolution;
                unsigned int target_x;
                unsigned int target_y;

                if (!worldToMap(world_x, world_y, target_x, target_y))
                {
                    continue;
                }

                const auto scaled = static_cast<unsigned char>(std::clamp(value * m_max_cost / 100, 0, m_max_cost));
                setCost(target_x, target_y, std::max(getCost(target_x, target_y), scaled));
            }
        }
    }

    void RssiLayer::reset()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        const auto cell_count = getSizeInCellsX() * getSizeInCellsY();
        std::fill_n(getCharMap(), cell_count, nav2_costmap_2d::FREE_SPACE);
        m_need_recalculation = true;
    }

    bool RssiLayer::isClearable()
    {
        return false;
    }

    void RssiLayer::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
    {
        if (!message || message->info.width == 0 || message->info.height == 0 || message->info.resolution <= 0.0f || message->data.size() != static_cast<std::size_t>(message->info.width) * message->info.height)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(m_mutex);
        m_map = message;
        m_map_min_x = message->info.origin.position.x;
        m_map_min_y = message->info.origin.position.y;
        m_map_max_x = m_map_min_x + message->info.width * message->info.resolution;
        m_map_max_y = m_map_min_y + message->info.height * message->info.resolution;
        m_need_recalculation = true;
    }

} // namespace sfg_go2_rssi_costmap_plugin

PLUGINLIB_EXPORT_CLASS(sfg_go2_rssi_costmap_plugin::RssiLayer, nav2_costmap_2d::Layer)
