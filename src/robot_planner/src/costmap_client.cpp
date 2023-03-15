
#include "robot_planner/costmap_client.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"


#include <functional>
#include <mutex>
#include <string>

namespace robot_planner
{
    std::array<unsigned char, 256>  init_translation_table();
    std::array<unsigned char, 256> cost_translation_table__ = init_translation_table();
    
    Costmap2DClient::Costmap2DClient(rclcpp::Node& node, const tf2_ros::Buffer* tf)
    : tf_(tf), node_(node)
    {
        costmap_sub_ = node_.create_subscription<nav_msgs::msg::OccupancyGrid>(
            "global_costmap/costmap", rclcpp::SystemDefaultsQoS(),
            std::bind(&Costmap2DClient::costmapCallback, this, std::placeholders::_1));
    }

    void Costmap2DClient::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        costmap_received = true;
        unsigned int num_cells_x  = msg->info.width;
        unsigned int num_cells_y = msg->info.height;
        double resolution = msg->info.resolution;
        double origin_x = msg->info.origin.position.x;
        double origin_y = msg->info.origin.position.y;

        costmap_.resizeMap(num_cells_x, num_cells_y, resolution, origin_x, origin_y);
        auto* mutex = costmap_.getMutex();
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

        unsigned char* costmap_data = costmap_.getCharMap();
        size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
        RCLCPP_INFO(node_.get_logger(), "full map update, %lu values", costmap_size);
        for (size_t i = 0; i < costmap_size && i < msg->data.size(); ++i) {
            unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
            costmap_data[i] = cost_translation_table__[cell_cost];
        }
        RCLCPP_INFO(node_.get_logger(), "map updated, written %lu values",
                    costmap_size);

    }

    std::array<unsigned char, 256> init_translation_table()
    {
        std::array<unsigned char, 256> cost_translation_table;

        // lineary mapped from [0..100] to [0..255]
        for (size_t i = 0; i < 256; ++i) {
            cost_translation_table[i] =
                static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
        }

        // special values:
        cost_translation_table[0] = 0;      // NO obstacle
        cost_translation_table[99] = 253;   // INSCRIBED obstacle
        cost_translation_table[100] = 254;  // LETHAL obstacle
        cost_translation_table[static_cast<unsigned char>(-1)] = 255;  // UNKNOWN

        return cost_translation_table;
    }
}

