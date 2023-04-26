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
        std::stringstream ss;
        ss << "obstacle_points: " << obstacle_points << std::endl;
        RCLCPP_INFO(node_.get_logger(), ss.str().c_str());
        traversed_h_signatures.clear();
        for(long unsigned int i=0;i<traversed_paths.size();i++)
        {
            Eigen::VectorXd h_signature = Eigen::VectorXd::Zero(obstacles.size()); 
            for(long unsigned int j=1;j<traversed_paths[i].size();++i)
            {
               ; 
            }
        }

    }


}