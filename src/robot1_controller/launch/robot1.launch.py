import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription

package_name = 'robot1_controller'

MAX_ROBOTS = 1
robot_names = ['robot_'+str(i) for i in range(1,MAX_ROBOTS+1)]
robot_world_sizes  = [20 for i in range(MAX_ROBOTS)]

def generate_launch_description():
    
    
    robot_instances = [
        Node(package=package_name, executable='robot_1_map_processor',
                            arguments=[robot_names[i],str(robot_world_sizes[i])],
            )
       for i in range(len(robot_names))
    ]
    '''robot_1_motion = Node(
        package=package_name,
        executable='robot_1_motion_planner'
    )'''
    return LaunchDescription(robot_instances)