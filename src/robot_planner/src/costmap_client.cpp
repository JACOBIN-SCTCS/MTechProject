
#include "robot_planner/costmap_client.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"

#include <functional>
#include <mutex>
#include <string>
#include <unistd.h>
#include <exception>
#include "Eigen/Dense"

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
        Eigen::MatrixXd m(2,2);
        m(0,0) = 3;
        m(1,0) = 2.5;
        m(0,1) = -1;
        m(1,1) = m(0,0) + m(0,1);
        int ele = m.coeff(1,1);
        RCLCPP_INFO(node_.get_logger(), "Waiting for the global costmap to arrive %d",ele);
        while(!costmap_received)
        {
            rclcpp::spin_some(node_.get_node_base_interface());
            usleep(100000);
        }

        costmap_update_sub_ = node_.create_subscription<map_msgs::msg::OccupancyGridUpdate>(
            "/global_costmap/costmap_updates", rclcpp::SystemDefaultsQoS(),
            std::bind(&Costmap2DClient::costmapUpdateCallback, this, std::placeholders::_1));  


        /*auto last_error = node_.now();
        std::string tf_error;
        while (rclcpp::ok() && !tf_->canTransform("map", "base_link",
                            tf2::TimePointZero, tf2::durationFromSec(0.1),
                            &tf_error)) 
        {
            rclcpp::spin_some(node_.get_node_base_interface());
            if (last_error + tf2::durationFromSec(5.0) < node_.now()) {
                RCLCPP_WARN(node_.get_logger(),
                  "Timed out waiting for transform to become "
                  "available "
                  "before subscribing to costmap, tf error: %s",
                  tf_error.c_str());
             last_error = node_.now();
     
            }
   
            tf_error.clear();
        }*/
        RCLCPP_INFO(node_.get_logger(), "Global CostMap Arrived");



    }

    void Costmap2DClient::costmapUpdateCallback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
    {
         
        RCLCPP_INFO(node_.get_logger(),"Inside Update costmap");
    
        if (msg->x < 0 || msg->y < 0) {
            RCLCPP_INFO(node_.get_logger(),
                 "negative coordinates, invalid update. x: %d, y: %d", msg->x,
                 msg->y);
            return;
        }

        size_t x0 = static_cast<size_t>(msg->x);
        size_t y0 = static_cast<size_t>(msg->y);
        size_t xn = msg->width + x0;
        size_t yn = msg->height + y0;

  
        auto* mutex = costmap_.getMutex();
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);
        size_t costmap_xn = costmap_.getSizeInCellsX();
        size_t costmap_yn = costmap_.getSizeInCellsY();

        if (xn > costmap_xn || x0 > costmap_xn || yn > costmap_yn ||
            y0 > costmap_yn) {
            RCLCPP_INFO(node_.get_logger(),
                "received update doesn't fully fit into existing map, "
                "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
                "map is: [0, %lu], [0, %lu]",
                x0, xn, y0, yn, costmap_xn, costmap_yn);
            }

  
        unsigned char* costmap_data = costmap_.getCharMap();
        size_t i = 0;
        for (size_t y = y0; y < yn && y < costmap_yn; ++y) {
           for (size_t x = x0; x < xn && x < costmap_xn; ++x) {
                size_t idx = costmap_.getIndex(x, y);
                unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
                costmap_data[idx] = cost_translation_table__[cell_cost];
                ++i;
            }
        }
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
            costmap_data[i] = cell_cost;
            //costmap_data[i] = cost_translation_table__[cell_cost];
        }
        /*RCLCPP_INFO(node_.get_logger(), "map updated, written %lu values",
                    costmap_size);*/

    }

    void Costmap2DClient::printRobotPose() const
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "base_link";
        pose.header.stamp = node_.now();
        try {
            pose = tf_->transform(pose, "map",
                                tf2::durationFromSec(0.3));
        }
        catch (tf2::LookupException& ex) {
            RCLCPP_ERROR(node_.get_logger(),
                          "No Transform available Error looking up robot pose: "
                          "%s\n",
                          ex.what());
   
        } 
        catch (tf2::ConnectivityException& ex) {
            RCLCPP_ERROR(node_.get_logger(),
                          "Connectivity Error looking up robot pose: %s\n",
                          ex.what());

        }
        catch (tf2::ExtrapolationException& ex) {
            RCLCPP_ERROR(node_.get_logger(),
                          "Extrapolation Error looking up robot pose: %s\n",
                          ex.what());
        } 
        catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(node_.get_logger(), "Other error: %s\n",
                          ex.what());
        }

        RCLCPP_INFO(node_.get_logger(), "Robot pose: x=%f, y=%f, theta=%f",
                    pose.pose.position.x, pose.pose.position.y,
                    tf2::getYaw(pose.pose.orientation));

    }

    nav2_costmap_2d::Costmap2D* robot_planner::Costmap2DClient::getCostmap()
    {
        while(!costmap_received)
        {
            rclcpp::spin_some(node_.get_node_base_interface());
            RCLCPP_INFO(node_.get_logger(), "Waiting for costmap for Obstacle Marking");
        }
        return &costmap_;
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

