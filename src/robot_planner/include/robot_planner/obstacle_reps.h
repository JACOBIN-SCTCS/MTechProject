
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace robot_planner
{
    struct Obstacle
    {
        int id;
        geometry_msgs::msg::Point rep;
        std::vector<geometry_msgs::msg::Point> points;
    };

    class ObstacleUtils
    {   
        public:
            ObstacleUtils(nav2_costmap_2d::Costmap2D costmap);
            void searchObstacles();
            std::vector<Obstacle> getObstacles();
            
        protected:
            nav2_costmap_2d::Costmap2D costmap_;
            std::vector<Obstacle> obstacles_;
            std::vector<int16_t> visited_;
            
        
    };
}