import os 
import sys 
import rclpy 
from ament_index_python.packages import get_package_share_directory 
from gazebo_msgs.srv import SpawnEntity

def main():
    argv = sys.argv[1:]
    rclpy.init()
    robot_number = int(sys.argv[2].split('_')[1])
    sdf_file_path = os.path.join(
        get_package_share_directory("robot_world_package"), "models",
        "my_robot"+str(robot_number), "model.sdf")
        

    node = rclpy.create_node("entity_spawner")
    client = node.create_client(SpawnEntity, "/spawn_entity")


    if not client.service_is_ready():
        client.wait_for_service()


    print(f"robot_sdf={sdf_file_path}")
    
    request = SpawnEntity.Request()
    request.name = argv[0]
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = argv[1]
    request.initial_pose.position.x = float(argv[2])
    request.initial_pose.position.y = float(argv[3])
    request.initial_pose.position.z = float(argv[4])
    future = client.call_async(request)
    
    
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()