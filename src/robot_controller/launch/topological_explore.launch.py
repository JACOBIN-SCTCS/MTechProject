import os
import pathlib
import launch
from launch_ros.actions import Node
from launch.actions import GroupAction,DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


package_name = 'robot_controller'

MAX_ROBOTS = 1
robot_names = ['robot_'+str(i) for i in range(1,MAX_ROBOTS+1)]

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time')

    bringup_cmd_group = GroupAction([
        Node(package=package_name, executable='topological_explore',
                            arguments=[robot_names[0]],
            )     
    ])

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(bringup_cmd_group)
    return ld