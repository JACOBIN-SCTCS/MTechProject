

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif


#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "Eigen/Dense"
#include "robot_topological_explore/costmap_client.h"

namespace robot_topological_explore
{
    class Robot
    {
        public:
            Robot(robot_topological_explore::Costmap2DClient *costmap_client);

            void get_exploration_path();
            void goto_frontier();
            double get_absolute_distance(geometry_msgs::msg::PoseStamped pose1, geometry_msgs::msg::PoseStamped pose2);

            std::vector<std::vector<geometry_msgs::msg::PoseStamped>> traversed_paths;
            std::vector<Eigen::VectorXd> traversed_h_signatures;
            
            geometry_msgs::msg::Point start_point;
            geometry_msgs::msg::Point current_pose;
            geometry_msgs::msg::Point goal_pose;
            geometry_msgs::msg::Point global_start_point;
            geometry_msgs::msg::Point global_goal_pose;

            std::vector<geometry_msgs::msg::PoseStamped> current_path;
            Eigen::VectorXd partial_h_signature;
    
            robot_topological_explore::Costmap2DClient *_costmap_client;

    };
}