#include "robot_topological_explore/robot.h"

namespace robot_topological_explore
{
    Robot::Robot(robot_topological_explore::Costmap2DClient *costmap_client)
    {
        _costmap_client = costmap_client;
        auto robot_start_pose = _costmap_client->getRobotPose();
        global_start_point.x = robot_start_pose.pose.position.x;
        global_start_point.y = robot_start_pose.pose.position.y;
        robot_topological_explore::WorldCoord world_last_index = _costmap_client->getGlobalGoalPose();
        global_goal_pose.x = world_last_index.x;
        global_goal_pose.y = world_last_index.y;
    }


    void Robot::get_exploration_path()
    {

    }

}