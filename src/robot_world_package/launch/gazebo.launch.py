import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
 
package_name = 'robot_world_package'
world_file_name = 'maze.world'

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time',default=True)
    pkg_dir = get_package_share_directory(package_name)

    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir,'models')
    
    world = os.path.join(pkg_dir,'worlds',world_file_name)
    #launch_file_dir = os.path.join(pkg_dir,'launch')

    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so'],
            output='screen')
    
    robot_coordinates = [[0.0,0.0,0.0]]
    robot_instances = [ 
                        Node(package=package_name, executable='spawn_robot',
                            arguments=['WarehouseBot'+str(i+1), 'robot_'+str(i+1), str(robot_coordinates[i][0]), str(robot_coordinates[i][1]), str(robot_coordinates[i][2])],
                            output='screen')
                        
                        for i in range(len(robot_coordinates))
                    ]
    nodes = [gazebo] + robot_instances
    
    return LaunchDescription(nodes)