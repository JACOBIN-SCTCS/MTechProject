
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include "robot_topological_explore/costmap_client.h"
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

namespace robot_topological_explore
{
    std::array<unsigned char, 256>  init_translation_table();
    std::array<unsigned char, 256> cost_translation_table__ = init_translation_table();

    
    Costmap2DClient::Costmap2DClient(rclcpp::Node& node, const tf2_ros::Buffer* tf)
    : tf_(tf), node_(node)
    {
        costmap_sub_ = node_.create_subscription<nav_msgs::msg::OccupancyGrid>(
            "global_costmap/costmap", rclcpp::SystemDefaultsQoS(),
            std::bind(&Costmap2DClient::costmapCallback, this, std::placeholders::_1));
       
        RCLCPP_INFO(node_.get_logger(), "Waiting for the global costmap to arrive");
        while(!costmap_received)
        {
            rclcpp::spin_some(node_.get_node_base_interface());
            usleep(100000);
        }

        costmap_update_sub_ = node_.create_subscription<map_msgs::msg::OccupancyGridUpdate>(
            "/global_costmap/costmap_updates", rclcpp::SystemDefaultsQoS(),
            std::bind(&Costmap2DClient::costmapUpdateCallback, this, std::placeholders::_1));  
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
        unsigned int num_cells_x  = msg->info.width;
        unsigned int num_cells_y = msg->info.height;
        double resolution = msg->info.resolution;
        double origin_x = msg->info.origin.position.x;
        double origin_y = msg->info.origin.position.y;

        RCLCPP_INFO(node_.get_logger(),"Costmap Width = %u Height = %u Resolution = %lf", num_cells_x,num_cells_y,resolution);
        costmap_.resizeMap(num_cells_x, num_cells_y, resolution, origin_x, origin_y);
        auto* mutex = costmap_.getMutex();
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

        unsigned char* costmap_data = costmap_.getCharMap();
        size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
        RCLCPP_INFO(node_.get_logger(), "full map update, %lu values", costmap_size);
        for (size_t i = 0; i < costmap_size && i < msg->data.size(); ++i) {
            unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
            costmap_data[i] = cell_cost;
            costmap_data[i] = cost_translation_table__[cell_cost];
        }
        costmap_received = true;
    }

    geometry_msgs::msg::PoseStamped Costmap2DClient::getRobotPose() const
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
        unsigned int mx , my;
        if(!costmap_.worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
        {
            RCLCPP_INFO(node_.get_logger(), "Robot pose is out of map bounds");
        }
        else
        {
            unsigned int index = costmap_.getIndex(mx, my);
            RCLCPP_INFO(node_.get_logger(), "Robot pose in map: x=%u, y=%u , index=%u", mx, my,index);
        }
        RCLCPP_INFO(node_.get_logger(), "Robot pose: x=%f, y=%f, theta=%f",
                    pose.pose.position.x, pose.pose.position.y,
                    tf2::getYaw(pose.pose.orientation));
        return pose;
    }

    robot_topological_explore::WorldCoord robot_topological_explore::Costmap2DClient::getGlobalGoalPose()
    {
        size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
        unsigned int mx , my;
        double wx , wy ;
        costmap_.indexToCells(costmap_size-1, mx, my);
        costmap_.mapToWorld(mx, my, wx, wy);
        return {wx,wy};
    }

    nav2_costmap_2d::Costmap2D* robot_topological_explore::Costmap2DClient::getCostmap()
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

    robot_topological_explore::WorldCoord robot_topological_explore::Costmap2DClient::convert_index_to_world(unsigned int index) 
    {
        unsigned int mx, my;
        auto* mutex = costmap_.getMutex();
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);
        costmap_.indexToCells(index, mx, my);
        double wx, wy;
        costmap_.mapToWorld(mx, my, wx, wy);
        return {wx, wy};
    }
    std::vector<unsigned int> robot_topological_explore::Costmap2DClient::getNeighbors(unsigned int index)
    {
        std::vector<unsigned int> neighbors;
        unsigned int x_size = costmap_.getSizeInCellsX();
        unsigned int y_size = costmap_.getSizeInCellsY();
        if (index > x_size * y_size - 1)
        {
            RCLCPP_INFO(rclcpp::get_logger("FrontierExploration"), "Evaluating nhood "
                                                                   "for offmap point");
            return neighbors;
        }

        if (index % x_size > 0)
        {
            neighbors.push_back(index - 1);
        }
        if (index % x_size < x_size - 1)
        {
            neighbors.push_back(index + 1);
        }
        if (index >= x_size)
        {
            neighbors.push_back(index - x_size);
        }
        if (index < x_size * (x_size - 1))
        {
            neighbors.push_back(index + x_size);
        }
        if (index % x_size > 0 && index >= x_size)
        {
            neighbors.push_back(index - 1 - x_size);
        }
        if (index % x_size > 0 && index < x_size * (y_size - 1))
        {
            neighbors.push_back(index - 1 + x_size);
        }
        if (index % x_size < x_size - 1 && index >= x_size)
        {
            neighbors.push_back(index + 1 - x_size);
        }
        if (index % x_size < x_size - 1 && index < x_size * (y_size - 1))
        {
            neighbors.push_back(index + 1 + x_size);
        }

        return neighbors;
    }


    void robot_topological_explore::Costmap2DClient::updateObstacles()
    {
        auto* mutex = costmap_.getMutex();
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

        obstacles_.clear();
        unsigned int obstacle_id = 1;
        unsigned char* costmap_data = costmap_.getCharMap();
        unsigned int size_x = costmap_.getSizeInCellsX();
        unsigned int size_y = costmap_.getSizeInCellsY();

        std::vector<int16_t> obstacle_tag(size_x*size_y , -1);
        std::vector<bool> visited(size_x*size_y, false);
    

       for(unsigned int i=0;i<(size_x * size_y);++i)
        {
            if(visited[i] == true || !(costmap_data[i]==253 || costmap_data[i]==254))
                continue;
            std::stack<unsigned int> stack;
            std::vector<std::vector<unsigned int>> points;
            unsigned int px,py;
            costmap_.indexToCells(i, px, py);
            points.push_back({px,py});
            stack.push(i);
            while(stack.size()>0)
            {
                unsigned int current_node = stack.top();
                stack.pop();
                if(visited[current_node] == true)
                    continue;
                visited[current_node] = true;
                std::vector<unsigned int> neighbors = getNeighbors(current_node);
                for(unsigned int j=0;j<neighbors.size();++j)
                {
                    if(costmap_data[neighbors[j]]==253 || costmap_data[neighbors[j]]==254)
                    {
                        stack.push(neighbors[j]);
                        unsigned int px,py;
                        costmap_.indexToCells(neighbors[j], px, py);
                        points.push_back({px,py});
                    }
                }
            }
            Obstacle obs;
            obs.obstacle_id = obstacle_id++;
            obs.rep = {points[0][0],points[0][1]};
            obs.obstacle_points = points;
            double reference_x , reference_y ; 
            costmap_.mapToWorld(obs.rep[0], obs.rep[1], reference_x, reference_y);
            obs.obstacle_pose = geometry_msgs::msg::Pose();
            obs.obstacle_pose.position.x = reference_x;
            obs.obstacle_pose.position.y = reference_y;
            obs.obstacle_pose.orientation.w = 1.0;
            obstacles_.push_back(obs);
        }
      
    }
}

