#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace robot_planner
{
    class Costmap2DClient
    {
        public:
            Costmap2DClient(rclcpp::Node& node, const tf2_ros::Buffer* tf_listener);
            nav2_costmap_2d::Costmap2D getCostmap();
            void printRobotPose() const;

        protected:
            void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
            void costmapUpdateCallback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg);
           
            const tf2_ros::Buffer* const tf_; 
            rclcpp::Node& node_; 
            nav2_costmap_2d::Costmap2D costmap_;

            bool costmap_received = false;
        private:
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
            rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_update_sub_;


    };

}  
