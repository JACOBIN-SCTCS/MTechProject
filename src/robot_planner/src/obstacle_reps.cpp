
#include "robot_planner/obstacle_reps.h"
#include "geometry_msgs/msg/pose.hpp"
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
            if(visited[i]==true  || !(costmap_data[i]==253 || costmap_data[i]==254))
            {
                continue;
            }
            std::stack<unsigned int> stack;
            std::vector<std::vector<unsigned int>> points;
            unsigned int px,py ;
            costmap_->indexToCells(i,px,py);
            points.push_back({px,py});

            stack.push(i);
            while(stack.size()> 0)
            {
                unsigned int current_node = stack.top();
                stack.pop();
                if(visited[current_node]==true)
                    continue;
                visited[current_node] = true;
                std::vector<unsigned int> neighbours = getNeighbors(current_node);
                for(unsigned int j =0 ; j <neighbours.size();++j)
                {
                    auto neighbour = neighbours[j];
                    if (costmap_data[neighbour]==253 ||  costmap_data[neighbour]==254)
                    {
                        unsigned int px,py;
                        costmap_->indexToCells(neighbour,px,py);
                        points.push_back({px,py});
                        stack.push(neighbour);
                    }

                }
            }
            
            Obstacle obstacle;
            obstacle.id = obstacle_id++;
            obstacle.rep = {points[0][0],points[0][1]};
            obstacle.points = points;
            double reference_x,reference_y;
            costmap_->mapToWorld(obstacle.rep[0],obstacle.rep[1],reference_x,reference_y);
            obstacle.rep_world = geometry_msgs::msg::Pose();
            obstacle.rep_world.position.x = reference_x;
            obstacle.rep_world.position.y = reference_y;
            obstacle.rep_world.orientation.w = 1.0;
            obstacles_.push_back(obstacle);
        
        }

    }

    void ObstacleUtils::searchFrontiers(unsigned int index)
    {
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
        *(costmap_->getMutex()));
        

        unsigned int x_size = costmap_ -> getSizeInCellsX();
        unsigned int y_size = costmap_ -> getSizeInCellsY();
        unsigned char* costmap_data = costmap_->getCharMap();

        std::vector<bool> visited(x_size*y_size , false);
        std::vector<bool> frontier_tag(x_size * y_size, false);
        std::queue<unsigned int> bfs_queue;        
        
        bfs_queue.push(index);
        visited[index] = true;
        bool first_frontier = false;

        while(!bfs_queue.empty())
        {
            unsigned int front = bfs_queue.front();
            bfs_queue.pop();
            std::vector<unsigned int> neighbours = getNeighbors(front);
            for(unsigned int i = 0; i < neighbours.size();++i)
            {
                unsigned int neighbour = neighbours[i];
                if(visited[neighbour]==true || frontier_tag[neighbour]==true)
                    continue;
                
                visited[neighbour] = true;
                if(costmap_data[neighbour]==253 || costmap_data[neighbour]==254)
                    continue;
                if(costmap_data[neighbour]==255)
                {
                    std::vector<unsigned int> cell_neighbours = getNeighbors(neighbour);
                    for(unsigned int j = 0; j < cell_neighbours.size();++j)
                    {
                        unsigned int cell_neighbour = cell_neighbours[j];
                        if(costmap_data[cell_neighbour]<253)
                        {
                            frontier_tag[neighbour] = true;
                            break;
                        }
                    }
                    if(frontier_tag[neighbour]==true)
                    {
                        std::queue<unsigned int> frontier_queue;
                        frontier_queue.push(neighbour);
                        int size = 1;
                        std::vector<unsigned int> unknown_cells;
                        unknown_cells.push_back(neighbour);
                        while(!frontier_queue.empty())
                        {
                            unsigned int frontier_front = frontier_queue.front();
                            frontier_queue.pop();
                            std::vector<unsigned int> frontier_neighbours = getNeighbors(frontier_front);
                           
                            for(unsigned int k = 0; k < frontier_neighbours.size();++k)
                            {
                               if(frontier_tag[frontier_neighbours[k]]==true)
                                continue;
                               if(costmap_data[frontier_neighbours[k]]==255)
                                {
                                    frontier_tag[frontier_neighbours[k]] = true;
                                    unknown_cells.push_back(frontier_neighbours[k]);
                                    frontier_queue.push(frontier_neighbours[k]);
                                    size+=1;
                                }
                            }
                            
                        }
                        Frontier f ;
                        f.frontier_point = neighbour;
                        f.size = size;
                        f.unknown_cells = unknown_cells;
                        if(first_frontier==false)
                        {
                            first_frontier = true;
                            current_frontier = f;
                        }
                        else
                        {
                            if(f.size > current_frontier.size)
                            {
                                current_frontier = f;
                                
                            }
                        }
                    }
                }
                else
                {
                    bfs_queue.push(neighbour);
                }

            }
          
        }

    }
    Frontier ObstacleUtils::getFrontier()
    {
        return current_frontier;
    }
    
    std::vector<Obstacle> ObstacleUtils::getObstacles()
    {
        return obstacles_;
    }

}