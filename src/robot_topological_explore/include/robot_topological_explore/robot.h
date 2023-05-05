

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif


#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "robot_topological_explore/costmap_client.h"
#include "Eigen/Dense"

namespace robot_topological_explore
{
    struct AstarNode
    {
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
  
    };

    class Robot
    {
        public:
            Robot(rclcpp::Node& node,robot_topological_explore::Costmap2DClient *costmap_client);

            void get_exploration_path();
            void goto_frontier();
            double get_absolute_distance(geometry_msgs::msg::Point pose1, geometry_msgs::msg::Point pose2);
            void get_non_homologous_path(geometry_msgs::msg::Point current_pose,Eigen::VectorXcd obstacle_coords);
            void set_global_goal_pose(geometry_msgs::msg::Point new_global_goal);

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

    };
}