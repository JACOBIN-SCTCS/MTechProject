#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include "robot_topological_explore/robot.h"

namespace robot_topological_explore
{
    Robot::Robot(rclcpp::Node& node,robot_topological_explore::Costmap2DClient *costmap_client) : node_(node)
    {
        _costmap_client = costmap_client;
        auto robot_start_pose = _costmap_client->getRobotPose();
        global_start_point.x = robot_start_pose.pose.position.x;
        global_start_point.y = robot_start_pose.pose.position.y;
        robot_topological_explore::WorldCoord world_last_index = _costmap_client->getGlobalGoalPose();
        global_goal_pose.x = world_last_index.x;
        global_goal_pose.y = world_last_index.y;
        goal_pose = global_goal_pose;
        start_point = global_start_point;
    }

    double Robot::get_absolute_distance(geometry_msgs::msg::Point pose1, geometry_msgs::msg::Point pose2)
    {
        return sqrt(pow(pose1.x - pose2.x, 2) + pow(pose1.y - pose2.y, 2));
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

    void Robot::get_non_homologous_path(geometry_msgs::msg::Point current_pose, Eigen::VectorXcd obstacle_coords)
    {
        current_path.erase(current_path.begin()+current_path_index+1,current_path.end());
        
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
          *(_costmap_client->costmap_.getMutex()));
        unsigned char* costmap_data = _costmap_client->costmap_.getCharMap();
        unsigned int map_size_x = _costmap_client->costmap_.getSizeInCellsX();
        unsigned int map_size_y = _costmap_client->costmap_.getSizeInCellsY();
        //int count_limit = 5;
        //int count = 0;
      
        unsigned int mx,my;
        unsigned int gx,gy;
        _costmap_client->costmap_.worldToMap(current_pose.x, current_pose.y, mx, my);
        _costmap_client->costmap_.worldToMap(goal_pose.x,goal_pose.y,gx,gy);
        std::complex<double> start_point(mx,my);
        std::complex<double> goal_point(gx,gy);
        
        // searchObstacles();
        // searchFrontiers(pose);
        
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

        std::priority_queue<AstarNode*, std::vector<AstarNode*>, std::function<bool(AstarNode*, AstarNode*)>> pq([](AstarNode* a, AstarNode* b) { return (a->f + a->g) > (b->f + b->g); });
        std::unordered_map<std::string,double> distance_count;
        // std::set<std::string> visited;
        std::stringstream ss;
        ss << start_point << "-\n"<< partial_h_signature;
        distance_count[ss.str()] = std::abs(goal_point-start_point);
        AstarNode* start_node = new AstarNode(start_point,partial_h_signature,0,std::abs(goal_point-start_point),NULL,{});
        pq.push(start_node);
        while(!pq.empty())
        {
            AstarNode* node = pq.top();
            pq.pop();
            if(node->point == goal_point)
            {         
                Eigen::VectorXd filtered = (1.0/(2*M_PIf64))*node->h_signature;
			    if((filtered.array()> 1.0).any() || (filtered.array() < -1.0).any())
				    continue;
			
                Eigen::VectorXcd final_signature = node-> h_signature;
                if(goal_point.real()==global_start_point.x && goal_point.imag() == global_start_point.y)
                {
                    final_signature = -final_signature;
                }
                bool is_already_seen = false;
                
                for(long unsigned int i=0;i< traversed_h_signatures.size();++i)
                {
                    auto difference = traversed_h_signatures[i]-final_signature;
                    if (difference.isZero(0.0001))
                    {
                        is_already_seen = true;
                        break;
                    }
                }
                if(is_already_seen)
                    continue;

                std::vector<geometry_msgs::msg::Point> path;
                AstarNode* temp = node;
                while(temp!=NULL)
                {
                    double current_point_x , current_point_y;
                    try
                    {
                        _costmap_client->costmap_.mapToWorld((unsigned int)temp->point.real(),(unsigned int)temp->point.imag(),current_point_x,current_point_y);
                    }
                    catch(const std::exception& e)
                    {
                        std::cerr << e.what() << '\n';
                    }
                    geometry_msgs::msg::Point current_point;
                    current_point.x = current_point_x;
                    current_point.y = current_point_y;
                    path.push_back(current_point);
                    temp = temp->parent;
                }

                std::reverse(path.begin(),path.end());
                for(unsigned int j=0;j<path.size();++j)
                    current_path.push_back(path[j]);
                //current_path.push_back(path);
                
            }
            else
            {
                for(unsigned int i=0;i<directions.size();++i)
                {
                    std::complex<double> new_point = node->point + directions[i];
                    unsigned int new_point_index = _costmap_client->costmap_.getIndex((unsigned int)new_point.real(),(unsigned int)new_point.imag());
                    if(real(new_point)<0.0 || real(new_point)>= map_size_x || imag(new_point)<0.0 || imag(new_point)>=map_size_y || (costmap_data[new_point_index] == nav2_costmap_2d::LETHAL_OBSTACLE || costmap_data[new_point_index] == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
                        continue;
                    Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_coords.size(),node->point) - obstacle_coords;
                    Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_coords.size(),new_point) - obstacle_coords;
                    Eigen::VectorXd t =   s_vec.array().binaryExpr(e_vec.array(),customOp);
                    Eigen::VectorXd temp =  node->h_signature + t;
                    Eigen::VectorXd filtered = (1.0/(2*M_PIf64))*temp;
			        if((filtered.array()> 1.0).any() || (filtered.array() < -1.0).any())
				        continue;
			        double cell_cost = costmap_data[new_point_index];
                    if(cell_cost == nav2_costmap_2d::NO_INFORMATION)
                        cell_cost = 1.0;
                    double f = node->f + cell_cost;
                    double g = std::abs(new_point-goal_point);

                    // double c = node->cost + cell_cost + std::abs(new_point-goal_point);
                    
                    std::stringstream ss;
                    ss << new_point << "-\n"<< temp;
                    std::string key = ss.str();
                    if(distance_count.find(key)==distance_count.end() || distance_count[key] > (f+g))
                    {
                        distance_count[key] = f + g ;
                        std::vector<std::complex<double>> edge = { node->point,new_point}; 
                        AstarNode* new_node = new AstarNode(new_point,temp,f,g,node,edge);
                        pq.push(new_node);
                    }
                }
            }
        }
    }

    void Robot::get_exploration_path()
    {
        current_pose = _costmap_client->getRobotPose().pose.position;
        if(get_absolute_distance(current_pose,goal_pose) <= 0.001)
        {
            if(goal_pose.x == global_goal_pose.x && goal_pose.y == global_goal_pose.y)
            {
                goal_pose = global_start_point;
                start_point = current_pose;
            }
            else
            {
                goal_pose = global_goal_pose;
                start_point = current_pose;
            }
        }

        auto obstacles = _costmap_client->obstacles_;
        Eigen::VectorXcd obstacle_points = Eigen::VectorXcd::Zero(obstacles.size());
        for(long unsigned int i=0;i<obstacles.size();i++)
        {
            obstacle_points(i) = std::complex<double>(obstacles[i].obstacle_pose.position.x ,obstacles[i].obstacle_pose.position.y);
        }
        // std::stringstream ss;
        // ss << "obstacle_points: " << obstacle_points << std::endl;
        // RCLCPP_INFO(node_.get_logger(), ss.str().c_str());
        traversed_h_signatures.clear();
        for(long unsigned int i=0;i<traversed_paths.size();i++)
        {
            Eigen::VectorXd h_signature = Eigen::VectorXd::Zero(obstacles.size()); 
            for(long unsigned int j=1;j<traversed_paths[i].size();++i)
            {
                Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(traversed_paths[i][j-1].x,traversed_paths[i][j-1].y)) - obstacle_points;
                Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(traversed_paths[i][j].x,traversed_paths[i][j].y)) - obstacle_points;
                Eigen::VectorXd temp = s_vec.array().binaryExpr(e_vec.array(),customOp);
                h_signature += temp;
            }
            traversed_h_signatures.push_back(h_signature);
        }
        partial_h_signature = Eigen::VectorXd::Zero(obstacles.size());
        for(long unsigned int i = 0 ; i!=(current_path.size()-1) && i< current_path_index;++i)
        {
            Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(current_path[i].x,current_path[i].y)) - obstacle_points;
            Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(current_path[i+1].x,current_path[i+1].y)) - obstacle_points;
            Eigen::VectorXd temp = s_vec.array().binaryExpr(e_vec.array(),customOp);
            partial_h_signature += temp;
        }

    }


}