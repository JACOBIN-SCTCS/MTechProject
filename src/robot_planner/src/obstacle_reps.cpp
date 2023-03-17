
#include "robot_planner/obstacle_reps.h"
#include <mutex>

namespace robot_planner
{
    ObstacleUtils::ObstacleUtils()
    {
        ;
    }

    ObstacleUtils::ObstacleUtils(nav2_costmap_2d::Costmap2D* costmap):
        costmap_(costmap)
    {
        ;
    }

    std::vector<unsigned int> ObstacleUtils::getNeighbors(unsigned int index)
    {
        std::vector<unsigned int> neighbors;
        unsigned int x_size = costmap_ -> getSizeInCellsX();
        unsigned int y_size = costmap_ -> getSizeInCellsY();
        if (index > x_size * y_size - 1) {
            RCLCPP_INFO(rclcpp::get_logger("FrontierExploration"), "Evaluating nhood "
                                                           "for offmap point");
            return neighbors;
        }

        if (index % x_size > 0) {
            neighbors.push_back(index - 1);
        }
        if (index % x_size < x_size - 1) {
            neighbors.push_back(index + 1);
        }
        if (index >= x_size) {
            neighbors.push_back(index - x_size);
        }
        if (index < x_size * (x_size - 1)) {
            neighbors.push_back(index + x_size);
        }
        if (index % x_size > 0 && index >= x_size) {
                neighbors.push_back(index - 1 - x_size);
        }
        if (index % x_size > 0 && index < x_size * (y_size - 1)) {
            neighbors.push_back(index - 1 + x_size);
        }
        if (index % x_size < x_size - 1 && index >= x_size) {
            neighbors.push_back(index + 1 - x_size);
        }
        if (index % x_size < x_size - 1 && index < x_size * (y_size - 1)) {
            neighbors.push_back(index + 1 + x_size);
        }

        return neighbors;
    }

    void ObstacleUtils::searchObstacles()
    {
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
        *(costmap_->getMutex()));
        obstacles_.clear();
        unsigned int obstacle_id = 1;

        unsigned char* costmap_data = costmap_->getCharMap();
        unsigned int x_size = costmap_ -> getSizeInCellsX();
        unsigned int y_size = costmap_ -> getSizeInCellsY();

        std::vector<int16_t> obstacle_tag(x_size * y_size, -1);
        std::vector<bool> visited(x_size*y_size , false);

        for(unsigned int i = 0; i < (x_size*y_size);++i)
        {
            if(costmap_data[i]!=100)
            {
                visited[i] = true;
                continue;
            }
            if(visited[i]==true)
                continue;
            
            std::queue<unsigned int> bfs_queue;
            std::vector<std::vector<unsigned int>> points;
            unsigned int px,py ;
            costmap_->indexToCells(i,px,py);
            points.push_back({px,py});
            bfs_queue.push(i);
            visited[i] = true;

            while(!bfs_queue.empty())
            {
                unsigned int current_index = bfs_queue.front();
                bfs_queue.pop();
                std::vector<unsigned int> neighbors = getNeighbors(current_index);
                for(unsigned int j = 0; j < neighbors.size(); ++j)
                {
                    if(visited[neighbors[j]]==true)
                        continue;
                    if(costmap_data[neighbors[j]]!=100)
                    {
                        visited[neighbors[j]] = true;
                        continue;
                    }
                    bfs_queue.push(neighbors[j]);
                    visited[neighbors[j]] = true;
                    costmap_->indexToCells(neighbors[j],px,py);
                    points.push_back({px,py});
                }
            }
            Obstacle obstacle;
            obstacle.id = obstacle_id++;
            obstacle.rep = {0,0};
            obstacle.points = points;
            obstacles_.push_back(obstacle);
        
        }

    }

    std::vector<Obstacle> ObstacleUtils::getObstacles()
    {
        return obstacles_;
    }

}