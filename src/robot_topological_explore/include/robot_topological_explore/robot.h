

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif


#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <boost/graph/adjacency_list.hpp>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>

#include <iostream>
#include <thread>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "robot_topological_explore/costmap_client.h"
#include "Eigen/Dense"

namespace robot_topological_explore
{
    class gridStateValidityCheckerClass : public ompl::base::StateValidityChecker
    {
        public:
            std::vector<std::vector<unsigned char>> grid_data;

            gridStateValidityCheckerClass(const ompl::base::SpaceInformationPtr &si, std::vector<std::vector<unsigned char>> &g) :
            ompl::base::StateValidityChecker(si), grid_data(g)
                {
            }
        
            virtual bool isValid(const ompl::base::State *state) const
            {
                unsigned int x  = state->as<ompl::base::CompoundState>()->as<ompl::base::DiscreteStateSpace::StateType>(0)->value;
                unsigned int y = state->as<ompl::base::CompoundState>()->as<ompl::base::DiscreteStateSpace::StateType>(1)->value;
                if(grid_data[x][y]==253 || grid_data[x][y] == 254)
                    return false;
                return true;
            }
    };

    struct AstarNode
    {
        unsigned int vertex_id;
        std::complex<double> point;
        Eigen::VectorXd h_signature;
        double f;
        double g;
        double cost;
        struct AstarNode* parent; 
        std::vector<std::complex<double>> edge;

        AstarNode(std::complex<double> p,
        Eigen::VectorXd h,
        double f,
        double g,
        struct AstarNode* pa,
        std::vector<std::complex<double>> e) : point(p) , h_signature(h),f(f),g(g),parent(pa) , edge(e) {}

        AstarNode(std::complex<double> p,
        Eigen::VectorXd h,
        double c,
        struct AstarNode* pa,
        std::vector<std::complex<double>> e) : point(p) , h_signature(h),cost(c),parent(pa) , edge(e) {}
        
        AstarNode(
        unsigned int v,
        std::complex<double> p,
        Eigen::VectorXd h,
        double f,
        double g,
        struct AstarNode* pa,
        std::vector<std::complex<double>> e) : vertex_id(v) , point(p) , h_signature(h),f(f),g(g),parent(pa) , edge(e) {}
  
    };

    class Robot
    {
        public:
            Robot(rclcpp::Node& node,robot_topological_explore::Costmap2DClient *costmap_client);

            void get_exploration_path();
            void goto_frontier();
            double get_absolute_distance(geometry_msgs::msg::Point pose1, geometry_msgs::msg::Point pose2);
            void get_non_homologous_path(geometry_msgs::msg::Point current_pose,Eigen::VectorXcd obstacle_coords);
            void get_non_homologous_PRM(geometry_msgs::msg::Point current_pose,Eigen::VectorXcd obstacle_coords);
            void set_global_goal_pose(geometry_msgs::msg::Point new_global_goal);
            void current_goal_succeeded();

            rclcpp::Node& node_;

            std::vector<std::vector<geometry_msgs::msg::Point>> traversed_paths;
            std::vector<Eigen::VectorXd> traversed_h_signatures;
            
            geometry_msgs::msg::Point start_point;
            geometry_msgs::msg::Point current_pose;
            geometry_msgs::msg::Point goal_pose;
            geometry_msgs::msg::Point global_start_point;
            geometry_msgs::msg::Point global_goal_pose;

            std::vector<geometry_msgs::msg::Point> current_path;
            unsigned int current_path_index;
            Eigen::VectorXd partial_h_signature;
    
            robot_topological_explore::Costmap2DClient *_costmap_client;
            std::vector<std::vector<unsigned char>> grid_data;
            

    };
}