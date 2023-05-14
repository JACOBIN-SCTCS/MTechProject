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
        
        // Initialize the global start pose
        global_start_point.x = robot_start_pose.pose.position.x;
        global_start_point.y = robot_start_pose.pose.position.y;

        // Initialize the global goal pose
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
        // RCLCPP_INFO(node_.get_logger(), "Reached inside get_non_homologous path function");

        current_path.erase(current_path.begin()+current_path_index,current_path.end());

        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
          *(_costmap_client->costmap_.getMutex()));
        unsigned char* costmap_data = _costmap_client->costmap_.getCharMap();
        unsigned int map_size_x = _costmap_client->costmap_.getSizeInCellsX();
        unsigned int map_size_y = _costmap_client->costmap_.getSizeInCellsY();
        //int count_limit = 5;
        //int count = 0;
        // RCLCPP_INFO(node_.get_logger()," get_non_homologous_path : Map Size = %u x %u",map_size_x,map_size_y);

        unsigned int mx,my;
        unsigned int gx,gy;
        _costmap_client->costmap_.worldToMap(current_pose.x, current_pose.y, mx, my);
        _costmap_client->costmap_.worldToMap(goal_pose.x,goal_pose.y,gx,gy);
        std::complex<double> s_point(mx,my);
        std::complex<double> g_point(gx,gy);
        
        // RCLCPP_INFO(node_.get_logger()," get_non_homologous_path : start_point = ( %u , %u ) , goal_point = ( %u , %u)",mx,my,gx,gy);

        
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
        ss << s_point << "-\n"<< partial_h_signature;
        distance_count[ss.str()] = std::abs(g_point-s_point);
        // RCLCPP_INFO(node_.get_logger()," get_non_homologous_path : distance value = %lf",distance_count[ss.str()]);
        AstarNode* start_node = new AstarNode(s_point,partial_h_signature,0,std::abs(g_point-s_point),NULL,{});
        pq.push(start_node);
        // RCLCPP_INFO(node_.get_logger()," get_non_homologous_path : start_node pushed");
        while(!pq.empty())
        {
            AstarNode* node = pq.top();
            // RCLCPP_INFO(node_.get_logger()," get_non_homologous_path : (%lf , %lf)",node->point.real(),node->point.imag());
            pq.pop();

            if(node->point == g_point)
            {         
                Eigen::VectorXd filtered = (1.0/(2*M_PIf64))*node->h_signature;
			    if((filtered.array()> 1.0).any() || (filtered.array() < -1.0).any())
				    continue;
			
                Eigen::VectorXcd final_signature = node-> h_signature;
                double real_world_goal_x, real_world_goal_y;
                _costmap_client->costmap_.mapToWorld((unsigned int)g_point.real(),(unsigned int)g_point.imag(),real_world_goal_x,real_world_goal_y);

                if(real_world_goal_x==global_start_point.x && real_world_goal_y == global_start_point.y)
                {
                    final_signature = -final_signature;
                }
                // RCLCPP_INFO(node_.get_logger()," get_non_homologous_path : Final Signature %lf",real(final_signature(0)));
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
                    // RCLCPP_INFO(node_.get_logger(),"(%f,%f)",temp->point.real(),temp->point.imag());
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
                    if(path.size() > 0)
                    {
                        if( (get_absolute_distance(current_point,path[path.size()-1]) >= 0.5) || (current_point.x==current_pose.x && current_point.y == current_pose.y))
                            path.push_back(current_point);
                    }
                    else
                    {
                        path.push_back(current_point);
                    }
                    temp = temp->parent;
                }
            

                std::reverse(path.begin(),path.end());
            
                std::vector<geometry_msgs::msg::Point> new_path;
                for(unsigned int j=0;j<current_path_index;++j)
                    new_path.push_back(current_path[j]);
                for(unsigned int j=0;j<path.size();++j)
                    new_path.push_back(path[j]);

                current_path = new_path;
                // for(unsigned int j=0;j<current_path.size();++j)
                //     RCLCPP_INFO(node_.get_logger()," get_non_homologous_path : (%lf , %lf)",current_path[j].x,current_path[j].y);
   
                 break; 
            }
            else
            {
                for(unsigned int i=0;i<directions.size();++i)
                {
                    std::complex<double> new_point = node->point + directions[i];
                    unsigned int new_point_index = _costmap_client->costmap_.getIndex((unsigned int)new_point.real(),(unsigned int)new_point.imag());
                    if( ((long)real(new_point))<0 || ((long)real(new_point))>= map_size_x || ((long)imag(new_point))<0 || ((long)imag(new_point))>=map_size_y || (costmap_data[new_point_index] == nav2_costmap_2d::LETHAL_OBSTACLE || costmap_data[new_point_index] == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
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
                    double g = std::abs(new_point-g_point);

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
        // Adopt either Frontier Based Exploration or Topological Exploration here    
        current_pose = _costmap_client->getRobotPose().pose.position;

        auto obstacles = _costmap_client->obstacles_;
        Eigen::VectorXcd obstacle_points = Eigen::VectorXcd::Zero(obstacles.size());
        for(long unsigned int i=0;i<obstacles.size();i++)
        {
            unsigned int current_obstacle_x, current_obstacle_y;
            try
            {
                _costmap_client->costmap_.worldToMap(obstacles[i].obstacle_pose.position.x,obstacles[i].obstacle_pose.position.y,current_obstacle_x,current_obstacle_y);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                exit(0);
            }
            obstacle_points(i) = std::complex<double>(current_obstacle_x,current_obstacle_y);
        }
        // RCLCPP_INFO(node_.get_logger(), "Completed getting new obstacle points");

        traversed_h_signatures.clear();
        for(long unsigned int i=0;i<traversed_paths.size();i++)
        {
            Eigen::VectorXd h_signature = Eigen::VectorXd::Zero(obstacles.size()); 
            for(long unsigned int j=1;j<traversed_paths[i].size();++j)
            {
                unsigned int sx,sy,gx,gy;
                try
                {
                    _costmap_client->costmap_.worldToMap(traversed_paths[i][j-1].x,traversed_paths[i][j-1].y,sx,sy);
                    _costmap_client->costmap_.worldToMap(traversed_paths[i][j].x,traversed_paths[i][j].y,gx,gy);
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                    exit(0);
                }
                Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(sx,sy)) - obstacle_points;
                Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(gx,gy)) - obstacle_points;
                Eigen::VectorXd temp = s_vec.array().binaryExpr(e_vec.array(),customOp);
                h_signature += temp;
            }
            traversed_h_signatures.push_back(h_signature);
        }
        RCLCPP_INFO(node_.get_logger(), "Completed computing h signatures traversed");
        partial_h_signature = Eigen::VectorXd::Zero(obstacles.size());
        
        for(long i=1; i <= current_path_index;++i)
        {
            unsigned int sx,sy,gx,gy;
            try
            {
                _costmap_client->costmap_.worldToMap(current_path[i-1].x,current_path[i-1].y,sx,sy);
                _costmap_client->costmap_.worldToMap(current_path[i].x,current_path[i].y,gx,gy);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                exit(0);
            }
            Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(sx,sy)) - obstacle_points;
            Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(gx,gy)) - obstacle_points;
            Eigen::VectorXd temp = s_vec.array().binaryExpr(e_vec.array(),customOp);
            partial_h_signature += temp;
        }
        
        RCLCPP_INFO(node_.get_logger(), "partial_h_signature: %f", partial_h_signature.norm());
        get_non_homologous_path(current_pose,obstacle_points);
    }

     void Robot::get_non_homologous_PRM(geometry_msgs::msg::Point current_pose,Eigen::VectorXcd obstacle_coords)
    {
           // Adopt either Frontier Based Exploration or Topological Exploration here
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
          *(_costmap_client->costmap_.getMutex()));
        
        
        unsigned char* costmap_data = _costmap_client->costmap_.getCharMap();
        unsigned int temp_grid_width = _costmap_client->costmap_.getSizeInCellsX();
        unsigned int temp_grid_height = _costmap_client->costmap_.getSizeInCellsY();
        
        std::vector<std::vector<unsigned char>> temp_grid_data(temp_grid_width,std::vector<unsigned char>(temp_grid_height));
        for(unsigned int i=0;i<temp_grid_width*temp_grid_height;++i)
        {
            unsigned int mx , my;
            _costmap_client->costmap_.indexToCells(i,mx,my);
            temp_grid_data[mx][my] = costmap_data[i];
        }
        grid_data = temp_grid_data;

        
        current_pose = _costmap_client->getRobotPose().pose.position;

        auto obstacles = _costmap_client->obstacles_;
        Eigen::VectorXcd obstacle_points = Eigen::VectorXcd::Zero(obstacles.size());
        for(long unsigned int i=0;i<obstacles.size();i++)
        {
            unsigned int current_obstacle_x, current_obstacle_y;
            try
            {
                _costmap_client->costmap_.worldToMap(obstacles[i].obstacle_pose.position.x,obstacles[i].obstacle_pose.position.y,current_obstacle_x,current_obstacle_y);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                exit(0);
            }
            obstacle_points(i) = std::complex<double>(current_obstacle_x,current_obstacle_y);
        }
        // RCLCPP_INFO(node_.get_logger(), "Completed getting new obstacle points");

        traversed_h_signatures.clear();
        for(long unsigned int i=0;i<traversed_paths.size();i++)
        {
            Eigen::VectorXd h_signature = Eigen::VectorXd::Zero(obstacles.size()); 
            for(long unsigned int j=1;j<traversed_paths[i].size();++j)
            {
                unsigned int sx,sy,gx,gy;
                try
                {
                    _costmap_client->costmap_.worldToMap(traversed_paths[i][j-1].x,traversed_paths[i][j-1].y,sx,sy);
                    _costmap_client->costmap_.worldToMap(traversed_paths[i][j].x,traversed_paths[i][j].y,gx,gy);
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                    exit(0);
                }
                Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(sx,sy)) - obstacle_points;
                Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(gx,gy)) - obstacle_points;
                Eigen::VectorXd temp = s_vec.array().binaryExpr(e_vec.array(),customOp);
                h_signature += temp;
            }
            traversed_h_signatures.push_back(h_signature);
        }
        RCLCPP_INFO(node_.get_logger(), "Completed computing h signatures traversed");
        partial_h_signature = Eigen::VectorXd::Zero(obstacles.size());
        
        for(long i=1; i <= current_path_index;++i)
        {
            unsigned int sx,sy,gx,gy;
            try
            {
                _costmap_client->costmap_.worldToMap(current_path[i-1].x,current_path[i-1].y,sx,sy);
                _costmap_client->costmap_.worldToMap(current_path[i].x,current_path[i].y,gx,gy);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                exit(0);
            }
            Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(sx,sy)) - obstacle_points;
            Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(gx,gy)) - obstacle_points;
            Eigen::VectorXd temp = s_vec.array().binaryExpr(e_vec.array(),customOp);
            partial_h_signature += temp;
        }
        
        RCLCPP_INFO(node_.get_logger(), "partial_h_signature: %f", partial_h_signature.norm());
        current_path.erase(current_path.begin()+current_path_index,current_path.end());


       

        unsigned int map_size_x = _costmap_client->costmap_.getSizeInCellsX();
        unsigned int map_size_y = _costmap_client->costmap_.getSizeInCellsY();
        unsigned int mx,my;
        unsigned int gx,gy;
        _costmap_client->costmap_.worldToMap(current_pose.x, current_pose.y, mx, my);
        _costmap_client->costmap_.worldToMap(goal_pose.x,goal_pose.y,gx,gy);

        // Insert PRM from here

        ompl::base::StateSpacePtr dim1(new ompl::base::DiscreteStateSpace(0,temp_grid_width-1));
        ompl::base::StateSpacePtr dim2(new ompl::base::DiscreteStateSpace(0,temp_grid_height-1));
        ompl::base::StateSpacePtr space = dim1 + dim2;
        auto si(std::make_shared<ompl::base::SpaceInformation>(space));
        si -> setStateValidityChecker(std::make_shared<robot_topological_explore::gridStateValidityCheckerClass>(si,temp_grid_data));

        ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(space);
        start->as<ompl::base::DiscreteStateSpace::StateType>(0)->value=mx;
        start->as<ompl::base::DiscreteStateSpace::StateType>(1)->value=my;

        ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(space);
        goal->as<ompl::base::DiscreteStateSpace::StateType>(0)->value=gx;
        goal->as<ompl::base::DiscreteStateSpace::StateType>(1)->value=gy;

        auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start,goal);

        auto planner(std::make_shared<ompl::geometric::PRM>(si));
        planner->setProblemDefinition(pdef);
        planner->setup();

        // planner->constructRoadmap(PlannerTerminationClass(NULL));
        planner->constructRoadmap(ompl::base::IterationTerminationCondition(1000));
        auto graph = planner->getRoadmap();
        std::cout << "No of vertices before  = " << graph.m_vertices.size()<<std::endl;
        // planner->constructRoadmap(ompl::base::CostConvergenceTerminationCondition(pdef));
        // planner->setConnectionFilter(customConnectionFilter);
        ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(1.0);
    

        if(solved)
        {
            std::cout<<"Solved the problem"<<std::endl;
            ompl::geometric::PathGeometric path( dynamic_cast< const ompl::geometric::PathGeometric& >( *pdef->getSolutionPath()));
            for(unsigned int i=0;i<path.getStateCount();++i)
            {
                const ompl::base::State* state = path.getState(i);
                auto coord1 = state->as<ompl::base::CompoundState>()->as<ompl::base::DiscreteStateSpace::StateType>(0)->value;
                auto coord2 = state->as<ompl::base::CompoundState>()->as<ompl::base::DiscreteStateSpace::StateType>(1)->value;
                std::cout<<"( " <<coord1 << "," << coord2 <<" )"<<std::endl; 
            }
            std::cout<<"Done printing path"<<std::endl;
        }
        else
        {
            std::cout<<"Couldnt find the path";
        }
        
        graph = planner->getRoadmap();
        std::cout << "No of vertices  after = " << graph.m_vertices.size()<<std::endl;
       

        //PRM code ends here

        std::complex<double> s_point(mx,my);
        std::complex<double> g_point(gx,gy);
        
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
        ss << s_point << "-\n"<< partial_h_signature;
        distance_count[ss.str()] = std::abs(g_point-s_point);
        // RCLCPP_INFO(node_.get_logger()," get_non_homologous_path : distance value = %lf",distance_count[ss.str()]);
        AstarNode* start_node = new AstarNode(s_point,partial_h_signature,0,std::abs(g_point-s_point),NULL,{});
        pq.push(start_node);
        // RCLCPP_INFO(node_.get_logger()," get_non_homologous_path : start_node pushed");
        while(!pq.empty())
        {
            AstarNode* node = pq.top();
            // RCLCPP_INFO(node_.get_logger()," get_non_homologous_path : (%lf , %lf)",node->point.real(),node->point.imag());
            pq.pop();

            if(node->point == g_point)
            {         
                Eigen::VectorXd filtered = (1.0/(2*M_PIf64))*node->h_signature;
                if((filtered.array()> 1.0).any() || (filtered.array() < -1.0).any())
                    continue;
            
                Eigen::VectorXcd final_signature = node-> h_signature;
                double real_world_goal_x, real_world_goal_y;
                _costmap_client->costmap_.mapToWorld((unsigned int)g_point.real(),(unsigned int)g_point.imag(),real_world_goal_x,real_world_goal_y);

                if(real_world_goal_x==global_start_point.x && real_world_goal_y == global_start_point.y)
                {
                    final_signature = -final_signature;
                }
                // RCLCPP_INFO(node_.get_logger()," get_non_homologous_path : Final Signature %lf",real(final_signature(0)));
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
                    // RCLCPP_INFO(node_.get_logger(),"(%f,%f)",temp->point.real(),temp->point.imag());
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
                    if(path.size() > 0)
                    {
                        if( (get_absolute_distance(current_point,path[path.size()-1]) >= 0.5) || (current_point.x==current_pose.x && current_point.y == current_pose.y))
                            path.push_back(current_point);
                    }
                    else
                    {
                        path.push_back(current_point);
                    }
                    temp = temp->parent;
                }
            

                std::reverse(path.begin(),path.end());
            
                std::vector<geometry_msgs::msg::Point> new_path;
                for(unsigned int j=0;j<current_path_index;++j)
                    new_path.push_back(current_path[j]);
                for(unsigned int j=0;j<path.size();++j)
                    new_path.push_back(path[j]);

                current_path = new_path;
             
                 break; 
            }
            else
            {
                for(unsigned int i=0;i<directions.size();++i)
                {
                    std::complex<double> new_point = node->point + directions[i];
                    unsigned int new_point_index = _costmap_client->costmap_.getIndex((unsigned int)new_point.real(),(unsigned int)new_point.imag());
                    if( ((long)real(new_point))<0 || ((long)real(new_point))>= map_size_x || ((long)imag(new_point))<0 || ((long)imag(new_point))>=map_size_y || (costmap_data[new_point_index] == nav2_costmap_2d::LETHAL_OBSTACLE || costmap_data[new_point_index] == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
                        continue;
                    Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(),node->point) - obstacle_points;
                    Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(),new_point) - obstacle_points;
                    Eigen::VectorXd t =   s_vec.array().binaryExpr(e_vec.array(),customOp);
                    Eigen::VectorXd temp =  node->h_signature + t;
                    Eigen::VectorXd filtered = (1.0/(2*M_PIf64))*temp;
                    if((filtered.array()> 1.0).any() || (filtered.array() < -1.0).any())
                        continue;
                    double cell_cost = costmap_data[new_point_index];
                    if(cell_cost == nav2_costmap_2d::NO_INFORMATION)
                        cell_cost = 1.0;
                    double f = node->f + cell_cost;
                    double g = std::abs(new_point-g_point);

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
    
    void Robot::set_global_goal_pose(geometry_msgs::msg::Point new_global_goal)
    {
        if(global_goal_pose.x == goal_pose.x && global_goal_pose.y == goal_pose.y)
        {
            global_goal_pose = new_global_goal;
            goal_pose = new_global_goal;
        }
        else
        {   
            global_goal_pose = new_global_goal;
        }
    }

   
    void Robot::current_goal_succeeded()
    {
        geometry_msgs::msg::Point to_add_source;
        geometry_msgs::msg::Point to_add_goal;

        if(goal_pose.x == global_goal_pose.x && goal_pose.y == global_goal_pose.y)
        {
                to_add_source.x = start_point.x;
                to_add_source.y = start_point.y;
                to_add_source.z = start_point.z;

                to_add_goal.x = goal_pose.x;
                to_add_goal.y = goal_pose.y;
                to_add_goal.z = goal_pose.z;
                current_path.push_back(to_add_goal);
                current_path.insert(current_path.begin(), to_add_source);

                goal_pose = global_start_point;
                start_point = global_goal_pose;
        }
        else
        {
                to_add_source.x = start_point.x;
                to_add_source.y = start_point.y;
                to_add_source.z = start_point.z;

                to_add_goal.x = goal_pose.x;
                to_add_goal.y = goal_pose.y;
                to_add_goal.z = goal_pose.z;

                current_path.push_back(to_add_goal);
                current_path.insert(current_path.begin(), to_add_source);         
                goal_pose = global_goal_pose;
                start_point = global_start_point;
                std::reverse(current_path.begin(),current_path.end());
        }   

        std::vector<geometry_msgs::msg::Point> current_path_copy;
            
        for(long unsigned int i=0;i<current_path.size();++i)
            current_path_copy.push_back(current_path[i]);
               
        traversed_paths.push_back(current_path_copy);
        current_path.clear();
        current_path_index = 0;
        
        RCLCPP_INFO(node_.get_logger(),"Current Path");
        for(unsigned int i = 0 ; i< current_path_copy.size();++i)
        {
            RCLCPP_INFO(node_.get_logger(),"(%lf,%lf)",current_path_copy[i].x,current_path_copy[i].y);
        }
    }

}