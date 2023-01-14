import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription

package_name = 'robot1_controller'

def generate_launch_description():
    robot1_map = Node(
        package=package_name,
        executable='robot_1_map_processor'
    )

    robot_1_motion = Node(
        package=package_name,
        executable='robot_1_motion_planner'
    )

    '''robot1_map_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["0", "0", "0", "0", "0", "0", "world_frame", "robot1_map"]

    )'''
    return LaunchDescription([
        robot1_map,
        robot_1_motion
        #robot1_map_frame
    ])