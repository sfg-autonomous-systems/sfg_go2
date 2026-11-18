#pragma once

#include <mutex>
#include <string>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/subscription.hpp"

namespace sfg_go2_rssi_costmap_plugin
{
    class RssiLayer : public nav2_costmap_2d::CostmapLayer
    {
    public:
        void onInitialize() override;
        void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y) override;
        void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) override;
        void reset() override;
        void matchSize() override;
        bool isClearable() override;

    private:
        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
        bool loadMapFile(const std::string &path);
        void rebuildQosCost();

        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_subscription;
        std::mutex m_mutex;
        bool m_need_recalculation;
        double m_map_min_x;
        double m_map_min_y;
        double m_map_max_x;
        double m_map_max_y;
        int m_max_cost;
        std::string m_map_topic;
        nav_msgs::msg::OccupancyGrid::SharedPtr m_map;
    };

} // namespace sfg_go2_rssi_costmap_plugin
