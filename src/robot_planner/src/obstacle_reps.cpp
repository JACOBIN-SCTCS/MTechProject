
#include "robot_planner/obstacle_reps.h"

namespace robot_planner
{
    ObstacleUtils::ObstacleUtils(nav2_costmap_2d::Costmap2D costmap):
        costmap_(costmap)
    {
        ;
    }

    void ObstacleUtils::searchObstacles()
    {
        
    }

    std::vector<Obstacle> ObstacleUtils::getObstacles()
    {
        return obstacles_;
    }

}