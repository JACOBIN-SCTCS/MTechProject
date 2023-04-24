#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace robot_topological_explore
{
    struct MapCoord
    {
        unsigned int x;
        unsigned int y;
    };

    struct WorldCoord
    {
        double x;
        double y;
    };

    struct Obstacle
    {
        unsigned int obstacle_id;
        std::vector<unsigned int> rep;
        std::vector<unsigned int> obstacle_cells;
        std::vector<std::vector<unsigned int>> obstacle_points;
        geometry_msgs::msg::Pose obstacle_pose;
    };


    class Costmap2DClient
    {
        public:
            Costmap2DClient(rclcpp::Node& node, const tf2_ros::Buffer* tf_listener);
            nav2_costmap_2d::Costmap2D* getCostmap();
            geometry_msgs::msg::PoseStamped getRobotPose() const;
            WorldCoord getGlobalGoalPose();
            WorldCoord convert_index_to_world(unsigned int index);
            void updateObstacles();
            
            std::vector<Obstacle> obstacles_;


        protected:
            void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
            void costmapUpdateCallback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg);
            std::vector<unsigned int> getNeighbors(unsigned int index);
           
            const tf2_ros::Buffer* const tf_; 
            rclcpp::Node& node_; 
            nav2_costmap_2d::Costmap2D costmap_;
            bool costmap_received = false;

        private:
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
            rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_update_sub_;


    };

}  
