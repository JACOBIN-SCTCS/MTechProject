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
    robot_urdf = 'model.urdf'

    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(package_dir,'models')
    urdf_file = os.path.join(package_dir,'models','my_robot',robot_urdf)



    # All Launch Configurations will be defined here
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')

    # pose = {
    #     'x': LaunchConfiguration('x_pose', default=0.0),
    #     'y': LaunchConfiguration('y_pose', default=0.5),
    #     'z': LaunchConfiguration('z_pose', default=0.0),
    #     'R': LaunchConfiguration('roll', default=0.0),
    #     'P': LaunchConfiguration('pitch', default=0.0),
    #     'Y': LaunchConfiguration('yaw', default=0.0)
    # }
    poses  = [{
        'x' : 0.0,
        'y' : 0.0,
        'z' : 0.0,
    },
    {
        'x' : -3.0,
        'y' : 0.0,
        'z' : 0.0,
    },
    
    ]
    
    # Remappings
    remappings = [ ('/tf','tf'),('/tf_static','tf_static') ]


    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top Level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Whether to apply a namespace to the node'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='true',
        description='Whether to run the simulator'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(package_dir, 'worlds', world_file_name),
        description='Full path to world model file to load'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
        description='Name of the robot'
    )

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition = IfCondition(use_simulator),
        cmd = ['gzserver','-s','libgazebo_ros_init.so',
               '-s','libgazebo_ros_factory.so',world
               ],
        cwd = [launch_dir],output='screen'
    )    

    start_gazebo_client_cmd = ExecuteProcess(
        condition = IfCondition(use_simulator),
        cmd = ['gzclient'],
        cwd = [launch_dir],output='screen')
    

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
                            arguments=['WarehouseBot', str(poses[0]['x']), str(poses[0]['y']), str(poses[0]['z']),namespace],
                            output='screen')
                        
    # robot_localization_node = Node(
    #      package='robot_localization',
    #      executable='ekf_node',
    #      name='ekf_filter_node',
    #      output='screen',
    #      parameters=[os.path.join(package_dir, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )

    # Create the launch description and populate the necessary fields
    ld  = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)


    # Add the actions to launch decsription
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    # ld.add_action(robot_localization_node)
    return ld
    
    