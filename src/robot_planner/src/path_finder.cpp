
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include "robot_planner/path_finder.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <mutex>
#include <complex>
#include <Eigen/Dense>
#include <ctime>
#include <vector>
#include <queue>
#include <unordered_map>
#include <set>

namespace robot_planner
{
    Utils::Utils()
    {
        ;
    }

    Utils::Utils(nav2_costmap_2d::Costmap2D *costmap) : costmap_(costmap)
    {
        ;
    }

    std::vector<unsigned int> Utils::getNeighbors(unsigned int index)
    {
        std::vector<unsigned int> neighbors;
        unsigned int x_size = costmap_->getSizeInCellsX();
        unsigned int y_size = costmap_->getSizeInCellsY();
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

    void Utils::searchObstacles()
    {
        obstacles_.clear();
        unsigned int obstacle_id = 1;

        unsigned char *costmap_data = costmap_->getCharMap();
        unsigned int x_size = costmap_->getSizeInCellsX();
        unsigned int y_size = costmap_->getSizeInCellsY();

        std::vector<int16_t> obstacle_tag(x_size * y_size, -1);
        std::vector<bool> visited(x_size * y_size, false);

        for (unsigned int i = 0; i < (x_size * y_size); ++i)
        {
            if (visited[i] == true || !(costmap_data[i] == 253 || costmap_data[i] == 254))
            {
                continue;
            }
            std::stack<unsigned int> stack;
            std::vector<std::vector<unsigned int>> points;
            unsigned int px, py;
            costmap_->indexToCells(i, px, py);
            points.push_back({px, py});

            stack.push(i);
            while (stack.size() > 0)
            {
                unsigned int current_node = stack.top();
                stack.pop();
                if (visited[current_node] == true)
                    continue;
                visited[current_node] = true;
                std::vector<unsigned int> neighbours = getNeighbors(current_node);
                for (unsigned int j = 0; j < neighbours.size(); ++j)
                {
                    auto neighbour = neighbours[j];
                    if (costmap_data[neighbour] == 253 || costmap_data[neighbour] == 254)
                    {
                        unsigned int px, py;
                        costmap_->indexToCells(neighbour, px, py);
                        points.push_back({px, py});
                        stack.push(neighbour);
                    }
                }
            }

            Obstacle obstacle;
            obstacle.id = obstacle_id++;
            obstacle.rep = {points[0][0], points[0][1]};
            obstacle.points = points;
            double reference_x, reference_y;
            costmap_->mapToWorld(obstacle.rep[0], obstacle.rep[1], reference_x, reference_y);
            obstacle.rep_world = geometry_msgs::msg::Pose();
            obstacle.rep_world.position.x = reference_x;
            obstacle.rep_world.position.y = reference_y;
            obstacle.rep_world.orientation.w = 1.0;
            obstacles_.push_back(obstacle);
        }
    }

    void Utils::searchFrontiers(geometry_msgs::msg::PoseStamped pose)
    {

       
        unsigned int mx, my;
        costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my);
        unsigned int index = costmap_->getIndex(mx, my);
        unsigned int x_size = costmap_->getSizeInCellsX();
        unsigned int y_size = costmap_->getSizeInCellsY();
        unsigned char *costmap_data = costmap_->getCharMap();

        std::vector<bool> visited(x_size * y_size, false);
        std::vector<bool> frontier_tag(x_size * y_size, false);
        std::queue<unsigned int> bfs_queue;

        bfs_queue.push(index);
        visited[index] = true;
        bool first_frontier = false;

        while (!bfs_queue.empty())
        {
            unsigned int front = bfs_queue.front();
            bfs_queue.pop();
            std::vector<unsigned int> neighbours = getNeighbors(front);
            for (unsigned int i = 0; i < neighbours.size(); ++i)
            {
                unsigned int neighbour = neighbours[i];
                if (visited[neighbour] == true || frontier_tag[neighbour] == true)
                    continue;

                visited[neighbour] = true;
                if (costmap_data[neighbour] == 253 || costmap_data[neighbour] == 254)
                    continue;
                if (costmap_data[neighbour] == 255)
                {
                    std::vector<unsigned int> cell_neighbours = getNeighbors(neighbour);
                    for (unsigned int j = 0; j < cell_neighbours.size(); ++j)
                    {
                        unsigned int cell_neighbour = cell_neighbours[j];
                        if (costmap_data[cell_neighbour] < 253)
                        {
                            frontier_tag[neighbour] = true;
                            break;
                        }
                    }
                    if (frontier_tag[neighbour] == true)
                    {
                        std::queue<unsigned int> frontier_queue;
                        frontier_queue.push(neighbour);
                        int size = 1;
                        std::vector<unsigned int> unknown_cells;
                        unknown_cells.push_back(neighbour);
                        while (!frontier_queue.empty())
                        {
                            unsigned int frontier_front = frontier_queue.front();
                            frontier_queue.pop();
                            std::vector<unsigned int> frontier_neighbours = getNeighbors(frontier_front);

                            for (unsigned int k = 0; k < frontier_neighbours.size(); ++k)
                            {
                                if (frontier_tag[frontier_neighbours[k]] == true)
                                    continue;
                                if (costmap_data[frontier_neighbours[k]] == 255)
                                {
                                    frontier_tag[frontier_neighbours[k]] = true;
                                    unknown_cells.push_back(frontier_neighbours[k]);
                                    frontier_queue.push(frontier_neighbours[k]);
                                    size += 1;
                                }
                            }
                        }
                        Frontier f;
                        f.frontier_point = neighbour;
                        costmap_->indexToCells(neighbour, mx, my);
                        f.map_coord = {mx, my};
                        f.size = size;
                        f.unknown_cells = unknown_cells;
                        if (first_frontier == false)
                        {
                            first_frontier = true;
                            current_frontier = f;
                        }
                        else
                        {
                            if (f.size > current_frontier.size)
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

    auto customOp = [](const std::complex<double>& a, const std::complex<double>& b) -> double
    {
		double minimum_phase_difference = std::arg(b) - std::arg(a);
		for(int i=-2;i<3;++i)
		{
			for(int j = -2; j<3;++j)
			{
				double phase_difference =  (std::arg(b) +2*M_PIf64*i) - (std::arg(a) + 2*M_PIf64*j);
				if(std::abs(phase_difference) < std::abs(minimum_phase_difference))
				{
					minimum_phase_difference = phase_difference;
				}  
			}
		}
		return minimum_phase_difference;

    };

    struct DijkstraNode
    {
        std::complex<double> point;
        Eigen::VectorXd h_signature;
        double cost;
        struct DijkstraNode* parent; 
        std::vector<std::complex<double>> edge;

        DijkstraNode( std::complex<double> p , Eigen::VectorXd h , double c , struct DijkstraNode* pa ,std::vector<std::complex<double>> e) : point(p) , h_signature(h),cost(c),parent(pa) , edge(e) {}
    };

    void Utils::findPath(geometry_msgs::msg::PoseStamped pose)
    {
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
            *(costmap_->getMutex()));
        unsigned char* costmap_data = costmap_->getCharMap();
        unsigned int map_size_x = costmap_->getSizeInCellsX();
        unsigned int map_size_y = costmap_->getSizeInCellsY();
        int count_limit  = 4;
        int count =0;


        unsigned int mx, my;
        costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my);
        searchObstacles();
        searchFrontiers(pose);

        Eigen::VectorXcd obstacle_points = Eigen::VectorXcd::Zero(obstacles_.size());
        for(unsigned int i= 0 ; i < obstacles_.size();++i)
            obstacle_points(i) = std::complex<double>(obstacles_[i].rep[0],obstacles_[i].rep[1]);
        std::complex<double> start_point(mx,my);
        std::complex<double> goal_point(current_frontier.map_coord[0],current_frontier.map_coord[1]);
        RCLCPP_INFO(rclcpp::get_logger("FrontierExploration"), "Start point : %f %f",start_point.real(),start_point.imag());
        RCLCPP_INFO(rclcpp::get_logger("FrontierExploration"), "Goal point : %f %f",goal_point.real(),goal_point.imag());

        std::vector<std::complex<double>> directions = {
            std::complex<double>(1.0,0.0),
            std::complex<double>(0.0,1.0),
            std::complex<double>(-1.0,0.0),
            std::complex<double>(0.0,-1.0),
            std::complex<double>(1.0,1.0),
            std::complex<double>(-1.0,1.0),
            std::complex<double>(1.0,-1.0),
            std::complex<double>(-1.0,-1.0),
        };

	    std::vector<std::vector<std::complex<double>>> paths;	
	    std::priority_queue<DijkstraNode*, std::vector<DijkstraNode*>, std::function<bool(DijkstraNode*, DijkstraNode*)>> pq([](DijkstraNode* a, DijkstraNode* b) { return a->cost > b->cost; });
	    std::unordered_map<std::string,double> distance_count;
	    std::set<std::string> visited;

	    std::stringstream ss;
	    Eigen::VectorXd zeros = Eigen::VectorXd::Zero(obstacle_points.size());
	    ss << start_point << "-\n"<< zeros;
	    distance_count[ss.str()] = std::abs(goal_point-start_point);

	    for(unsigned int i=0;i<directions.size();++i)
	    {
            std::complex<double> new_point = start_point + directions[i];
            unsigned int new_point_index = costmap_->getIndex((unsigned int)new_point.real(),(unsigned int)new_point.imag());

            if(real(new_point)<0.0 || real(new_point)>map_size_x || imag(new_point)<0.0 || imag(new_point)>map_size_y || (costmap_data[new_point_index] == 254 || costmap_data[new_point_index] == 253))
                        continue;
            Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(),start_point) - obstacle_points;
            Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(),new_point) - obstacle_points;
            Eigen::VectorXd temp = s_vec.array().binaryExpr(e_vec.array(),customOp);
            double c = 1 + std::abs(new_point-goal_point);
            std::vector<std::complex<double>> e = {start_point,new_point};
            DijkstraNode* node = new DijkstraNode(new_point,temp,c,NULL,e);
            pq.push(node);
	    }
	
        while(!pq.empty())
        {
            DijkstraNode* node = pq.top();
            pq.pop();
            if(node->point == goal_point)
            {
                std::stringstream ss;
                ss << node->h_signature;
                std::string key = ss.str();
                
                if(visited.find(key) == visited.end())
                {
                    //std::cout<<key<<std::endl<<std::endl;
                    count +=1;


                    visited.insert(key);
                    std::vector<geometry_msgs::msg::Point> path;
                    DijkstraNode* temp = node;
                     RCLCPP_INFO(rclcpp::get_logger("FrontierExploration"), "Before path logging");
                    while(temp!=NULL)
                    {

                        double current_point_x , current_point_y;
                        try
                        {
                            costmap_->mapToWorld((unsigned int)temp->point.real(),(unsigned int)temp->point.imag(),current_point_x,current_point_y);
                        }
                        catch(const std::exception& e)
                        {
                            std::cerr << e.what() << '\n';
                        }
                        geometry_msgs::msg::Point current_point;
                        current_point.x = current_point_x;
                        current_point.y = current_point_y;
                        RCLCPP_INFO(rclcpp::get_logger("FrontierExploration"), "Appending path code");

                        path.push_back(current_point);
                        temp = temp->parent;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("FrontierExploration"), "Reversing code");

                    std::reverse(path.begin(),path.end());
                    paths_.push_back(path);
                    if(count>=count_limit)
                    {
                        //RCLCPP_INFO(rclcpp::get_logger("FrontierExploration"), "Path Size = %lu",paths[0].size());
                        return;
                    }
                }
            }
            else
            {
                for(unsigned int i=0;i<directions.size();++i)
                {
                    std::complex<double> new_point = node->point + directions[i];
                    unsigned int new_point_index = costmap_->getIndex((unsigned int)new_point.real(),(unsigned int)new_point.imag());
                    if(real(new_point)<0.0 || real(new_point)> map_size_x || imag(new_point)<0.0 || imag(new_point)>map_size_y || (costmap_data[new_point_index] == 254 || costmap_data[new_point_index] == 253))
                        continue;
                    Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(),node->point) - obstacle_points;
                    Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(),new_point) - obstacle_points;
                    Eigen::VectorXd t =   s_vec.array().binaryExpr(e_vec.array(),customOp);
                    Eigen::VectorXd temp =  node->h_signature + t;
                    double c = node->cost + 1 + std::abs(new_point-goal_point);
                    
                    std::stringstream ss;
                    ss << new_point << "-\n"<< temp;
                    std::string key = ss.str();
                    if(distance_count.find(key)==distance_count.end() || distance_count[key] > c)
                    {
                        distance_count[key] = c;
                        std::vector<std::complex<double>> edge = { node->point,new_point}; 
                        DijkstraNode* new_node = new DijkstraNode(new_point,temp,c,node,edge);
                        pq.push(new_node);
                    }
                }
            }

	    }

    }
    Frontier Utils::getFrontier()
    {
        return current_frontier;
    }

    std::vector<Obstacle> Utils::getObstacles()
    {
        return obstacles_;
    }
    

}