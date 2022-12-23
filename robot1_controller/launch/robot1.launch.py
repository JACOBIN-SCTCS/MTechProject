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
    return LaunchDescription([
        robot1_map
    ])