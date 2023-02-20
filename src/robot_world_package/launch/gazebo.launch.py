import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration,Command
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro
 
package_name = 'robot_world_package'
world_file_name = 'maze.world'

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time',default=True)
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    pkg_dir = get_package_share_directory(package_name)
    
    sdf_file_path = os.path.join(
        get_package_share_directory("robot_world_package"), "models",
        "my_robot", "model.urdf")
    
    #sdf_text = xacro.process_file(sdf_file_path)


    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir,'models')
    
    world = os.path.join(pkg_dir,'worlds',world_file_name)
    #launch_file_dir = os.path.join(pkg_dir,'launch')

    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so'],
            output='screen')
    
    robot_coordinates = [[0.0,0.0,0.0]]

    robot_state_publishers = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name = 'robot_state_publisher',
            namespace='robot_' + str(i+1),
            parameters=[{'frame_prefix': 'robot_'+str(i+1)+'/','use_sim_time':True, 'robot_description': Command(['xacro ', sdf_file_path,' robot_number:=',str(i+1)]) }],
            output='screen'
        )

        for i in range(len(robot_coordinates))
    ]

    joint_state_publishers = [
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace='robot_' + str(i+1),
            parameters=[{'frame_prefix': 'robot_'+str(i+1)+'/','use_sim_time':True, 'robot_description': Command(['xacro ', sdf_file_path,' robot_number:=',str(i+1)]) }],
            output='screen'
        )
        for i in range(len(robot_coordinates))
    ]
    

    robot_instances = [ 
                        Node(package=package_name, executable='spawn_robot',
                            arguments=['WarehouseBot'+str(i+1), 'robot_'+str(i+1), str(robot_coordinates[i][0]), str(robot_coordinates[i][1]), str(robot_coordinates[i][2])],
                            output='screen')
                        
                        for i in range(len(robot_coordinates))
                    ]
    nodes = [gazebo] + robot_state_publishers+ joint_state_publishers + robot_instances 
    ld = LaunchDescription(nodes)
    ld.add_action(declare_use_sim_time_argument)
    return ld