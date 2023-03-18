import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,Command
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import Node



def generate_launch_description():
    package_name = 'robot_world_package'

    package_dir = get_package_share_directory(package_name=package_name)
    launch_dir = os.path.join(package_dir,'launch')
    
    world_file_name = 'maze.world'
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(package_dir,'models')

    world = LaunchConfiguration('world') 
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(package_dir, 'worlds', world_file_name),
        description='Full path to world model file to load'
    )

  
    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd = ['gzserver','-s','libgazebo_ros_init.so',
               '-s','libgazebo_ros_factory.so',world
               ],
        cwd = [launch_dir],output='screen'
    )    

    start_gazebo_client_cmd = ExecuteProcess(
        cmd = ['gzclient'],
        cwd = [launch_dir],output='screen')
    

   

    
    # Create the launch description and populate the necessary fields
    ld  = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)

    # Add the actions to launch decsription
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd),
    return ld
    





    