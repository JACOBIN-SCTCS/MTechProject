
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot_planner
{
    struct Obstacle
    {
        int id;
        std::vector<unsigned int> rep;
        std::vector<std::vector<unsigned int>> points;
        geometry_msgs::msg::Pose rep_world;
    };

    struct Frontier
    {
        unsigned int frontier_point;
        unsigned int size;
        std::vector<unsigned int> unknown_cells;
    };

    class Utils
    {   
        public:
            Utils();
            Utils(nav2_costmap_2d::Costmap2D *costmap);
            void searchObstacles();
            void searchFrontiers(geometry_msgs::msg::PoseStamped pose);
            Frontier getFrontier();
            std::vector<unsigned int> getNeighbors(unsigned int index);
            std::vector<Obstacle> getObstacles();
            
        protected:
            nav2_costmap_2d::Costmap2D* costmap_;
            std::vector<Obstacle> obstacles_;
            std::vector<int16_t> visited_;
            Frontier current_frontier;
            unsigned int min_frontier_size_ = 20;
            
        
    };
}