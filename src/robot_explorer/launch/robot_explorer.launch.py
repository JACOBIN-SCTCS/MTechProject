

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'robot_explorer'

    package_dir = get_package_share_directory(package_name=package_name)
    launch_dir = os.path.join(package_dir,'launch')


    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')

    #Input parameters declaration
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    param_substitutions = {
        'use_sim_time': use_sim_time,
    }

    lifecycle_nodes = ['map_saver']

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_dir, 'config', 'robot_slam.yaml'),
        description='Full path to the ROS2 parameters file to use for all SLAM')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # start_map_saver_server_cmd = Node(
    #     package='nav2_map_server',
    #     executable='map_saver_server',
    #     output='screen',
    #     parameters=[configured_params])

    # start_life_cycle_manager_cmd = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_slam',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #         {'node_names': lifecycle_nodes}
    #     ])

    # start_slam_toolbox = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(slam_launch_file),
    #     launch_arguments={'use_sim_time': use_sim_time,
    #                         'slam_params_file': params_file
    #                       }.items(),
        
    # )

    start_slam_toolbox = Node(
        parameters=[
          params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=namespace,
        remappings=[
                    ("/scan", "scan"),
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/odom", "odom"),
                    ("/map", "map"),
                    ("/map_metadata", "map_metadata"),
                    ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
                    ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization")
                ],
    )


    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # ld.add_action(start_map_saver_server_cmd)
    # ld.add_action(start_life_cycle_manager_cmd)
    ld.add_action(start_slam_toolbox)
    return ld


