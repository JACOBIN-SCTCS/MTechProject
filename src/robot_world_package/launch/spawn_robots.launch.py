


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
    
    robot_urdf = 'model.urdf'
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(package_dir,'models')
    urdf_file = os.path.join(package_dir,'models','my_robot',robot_urdf)

    # All Launch Configurations will be defined here
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    
    

    # Remappings
    remappings = [ ('/tf','tf'),('/tf_static','tf_static') ]


    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top Level namespace'
    )


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
        description='Name of the robot'
    )

    declare_x_cmd = DeclareLaunchArgument(
        'x',
        default_value=str(0.0),
        description='x coordinate of the robot'
    )

    declare_y_cmd = DeclareLaunchArgument(
        'y',
        default_value=str(0.0),
        description='y coordinate of the robot'
    )

    declare_z_cmd = DeclareLaunchArgument(
        'z',
        default_value=str(0.0),
        description='z coordinate of the robot'
    )

    # Specify the actions

    start_robot_state_publisher_cmd  = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
            'robot_description' :   Command(['xacro ', urdf_file]) }
        ],
        remappings=remappings
    )

    start_gazebo_spawner_cmd =  Node(
                            package=package_name, executable='spawn_robot',
                            arguments=['TurtleBot', x, y, z,namespace],
                            output='screen')
                        
    
    # Create the launch description and populate the necessary fields
    ld  = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)


    # Add the actions to launch decsription
    ld.add_action(start_robot_state_publisher_cmd),
    ld.add_action(start_gazebo_spawner_cmd)
    return ld
    





    