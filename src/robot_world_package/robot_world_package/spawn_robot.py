import os 
import sys 
import rclpy 
from ament_index_python.packages import get_package_share_directory 
from gazebo_msgs.srv import SpawnEntity
import xml.etree.ElementTree as ET
import subprocess
import xacro

def generate_custom_sdf(robot_number,sdf_tree):

    new_chassis_name = 'chassis_' + str(robot_number)
    
    new_right_wheel_name = 'right_wheel_' + str(robot_number)
    new_right_wheel_hinge_name = 'right_wheel_hinge_'+str(robot_number)  

    new_left_wheel_name = 'left_wheel_' + str(robot_number)
    new_left_wheel_hinge_name = 'left_wheel_hinge_'+str(robot_number)  
    
    new_laser_name = 'laser_' + str(robot_number)
    new_laser_link_name = 'laser_link_' + str(robot_number)
    new_laser_joint_name = 'laser_joint_' + str(robot_number)

    new_ultra_name = 'ultrasonic_' + str(robot_number)
    new_ultra_link_name = 'ultrasonic_link_' + str(robot_number)
    new_ultra_joint_name = 'chassis2joint_' + str(robot_number)

    sdf_tree[0][0].set('name',new_chassis_name)
    sdf_tree[0][1].set('name',new_right_wheel_name)
    sdf_tree[0][2].set('name',new_left_wheel_name)
    
    sdf_tree[0][3].set('name',new_left_wheel_hinge_name)
    sdf_tree[0][3][2].text = new_chassis_name
    sdf_tree[0][3][1].text = new_left_wheel_name
   
    sdf_tree[0][4].set('name',new_right_wheel_hinge_name)
    sdf_tree[0][4][2].text = new_chassis_name
    sdf_tree[0][4][1].text = new_right_wheel_name

    sdf_tree[0][5].set('name',new_laser_link_name)
    sdf_tree[0][5][-1].set('name',new_laser_name)

    sdf_tree[0][6].set('name',new_laser_joint_name)
    sdf_tree[0][6][0].text = new_laser_link_name
    sdf_tree[0][6][1].text = new_chassis_name

  
    sdf_tree[0][7].set('name',new_ultra_link_name)
    sdf_tree[0][7][0].set('name',new_ultra_name)

    sdf_tree[0][8].set('name',new_ultra_joint_name)
    sdf_tree[0][8][0].text = new_ultra_link_name
    sdf_tree[0][8][1].text = new_chassis_name


    sdf_tree[0][9][1].text = new_left_wheel_hinge_name
    sdf_tree[0][9][2].text = new_right_wheel_hinge_name

    sdf_tree[0][-1][-1].text = new_chassis_name
    sdf_tree[0][-1][0][0].text = '/robot_'+str(robot_number)
    sdf_tree[0][-3][-1][-1][0][0].text = '/robot_'+str(robot_number)
    sdf_tree[0][-5][-1][-1][0][0].text = '/robot_'+str(robot_number)
    new_sdf_text = ET.tostring(sdf_tree,encoding='unicode',method="xml")   

    return new_sdf_text

def main():
    argv = sys.argv[1:]
    rclpy.init()
    robot_number = int(sys.argv[2].split('_')[1])
    sdf_file_path = os.path.join(
        get_package_share_directory("robot_world_package"), "models",
        "my_robot", "model.urdf")

    #sdf_text = open(sdf_file_path,'r').read()
    sdf_text = xacro.process_file(sdf_file_path)
    xml = sdf_text.toxml()
    #sdf_text = subprocess.check_output(['xacro',sdf_file_path]).decode()
    #sdf_tree = ET.fromstring(sdf_text)
    #new_sdf_text = generate_custom_sdf(robot_number,sdf_tree)

    node = rclpy.create_node("entity_spawner")
    client = node.create_client(SpawnEntity, "/spawn_entity")


    if not client.service_is_ready():
        client.wait_for_service()


    print(f"robot_sdf={sdf_file_path}")
    
    request = SpawnEntity.Request()
    request.name = argv[0]
    #request.xml = open(sdf_file_path, 'r').read()
    #request.xml = new_sdf_text
    request.xml = xml
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