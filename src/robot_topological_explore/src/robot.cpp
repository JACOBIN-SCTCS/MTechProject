#include "robot_topological_explore/robot.h"

namespace robot_topological_explore
{
    Robot::Robot(robot_topological_explore::Costmap2DClient *costmap_client)
    {
        auto robot_start_pose = costmap_client->getRobotPose();
        start_point.x = robot_start_pose.pose.position.x;
        start_point.y = robot_start_pose.pose.position.y;
        robot_topological_explore::WorldCoord world_last_index = costmap_client->getGlobalGoalPose();
        

        _costmap_client = costmap_client;
    }
    void Robot::get_exploration_path()
    {

    }

}