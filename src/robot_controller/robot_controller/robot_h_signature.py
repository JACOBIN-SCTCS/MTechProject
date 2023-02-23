import rclpy
from rclpy.node import Node
from message_filters import Subscriber
from nav_msgs.msg import OccupancyGrid
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import math
from PIL import Image
import cv2
import matplotlib.pyplot as pyplot
import robot_controller.robot_helper
from nav_msgs.msg import Path

class TopologicalExplore(Node):

    def __init__(self,robot_name='robot_1'):
        super().__init__(robot_name+'_explore')
        self.navigating = False

        self.robot_name = robot_name
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.count = 0

        #map = Subscriber(self,OccupancyGrid,'/global_costmap/costmap')
        self.map_subscriber = self.create_subscription(OccupancyGrid,
            '/global_costmap/costmap',
            self.map_callback,
            10
        )
        self.map_subscriber

        self.path_publisher = self.create_publisher(Path,'h_signature_path',10)
        self.exploration_client = ActionClient(self,NavigateToPose,'navigate_to_pose')


    def goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Exploration goal rejected')
            return
        #self.get_logger().info('Navigation goal rejected')
        self._get_result_future  = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self,future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Arrived at destination')
        else:
            self.get_logger().info('Goal Failed')
        
    def send_goal(self):
        self.exploration_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id='map'
        goal_msg.pose.pose.position.x = 0.3
        goal_msg.pose.pose.position.y = -0.3
        
        self._send_goal_future = self.exploration_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        #rclpy.spin_until_future_complete(self,self._send_goal_future)
        #goal_handle = self._send_goal_future.result()
        #get_result_future = goal_handle.get_result_async()
        #rclpy.spin_until_future_complete(self,get_result_future)

    def imshow_components(self,labels):
    # Map component labels to hue val
        label_hue = np.uint8(179*labels/np.max(labels))
        blank_ch = 255*np.ones_like(label_hue)
        labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])

        # cvt to BGR for display
        labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)

        # set bg label to black
        labeled_img[label_hue==0] = 0

        cv2.imshow('labeled.png', labeled_img)
        pyplot.imshow(labeled_img)
        pyplot.show()

    def map_callback(self,msg:OccupancyGrid):
        if not self.navigating:
            self.send_goal()
            self.navigating = True
        self.get_logger().info("Map Origin = ("+str(msg.info.origin.position.x)+"," + str(msg.info.origin.position.y)+")")
            
        np_map = np.array(msg.data,dtype=np.float32)
        np_map = np.reshape(np_map,(-1,msg.info.width))
        

        t = None
        try:
            t = self.tf_buffer.lookup_transform('map','robot_1/base_link',rclpy.time.Time())
        except Exception as ex:
            self.get_logger().info('Could not find the transformation')
        if t is None:
            return

        map_x = t.transform.translation.x - msg.info.origin.position.x 
        map_y = t.transform.translation.y - msg.info.origin.position.y 
        map_resolution = msg.info.resolution
        map_width = msg.info.width
   

        map_index = int(math.floor(map_y/map_resolution)*map_width + math.floor(map_x/map_resolution))
        self.get_logger().info("The Map Index = " + str(map_index))
        
        row , col = divmod(map_index,map_width)
        f = open("/home/depressedcoder/sample.txt","w")
        np.savetxt(f,np_map)
        f.close()

        if self.count == 1:
            return

        binary_t = (np_map == 100)
        image = Image.fromarray(np.uint8(binary_t*255)).convert('RGB')
        open_cv_image = np.array(image)
        gray = cv2.cvtColor(open_cv_image,cv2.COLOR_BGR2GRAY)
        binary_image = cv2.threshold(gray,0,255,cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
        ret,labels,stats,centroids  = cv2.connectedComponentsWithStatsWithAlgorithm(binary_image,connectivity=8,ltype=cv2.CV_32S,ccltype=cv2.CCL_GRANA)
        
        obstacle_coordinates = []
        for i in range(1,len(centroids)):
            self.get_logger().info("Obstacle id = " + str(i))
            current_obstacle = (labels==i)
            current_obstacle_num_cells = np.count_nonzero(current_obstacle)
            random_point = np.random.randint(current_obstacle_num_cells)
            obstacle_cells = np.nonzero(current_obstacle)
            obstacle_coordinates.append([obstacle_cells[0][random_point],obstacle_cells[1][random_point]])
            #self.get_logger().info("Representative point = (" + str(obstalcle_cells[0][random_point]) + "," + str(obstalcle_cells[1][random_point])+")")
            #labels[obstalcle_cells[0][random_point]][obstalcle_cells[1][random_point]] = len(centroids)
            

        obstacles = np.array(obstacle_coordinates)
        self.get_logger().info(str(obstacles))
        self.count = 1
        '''try:
            t  = self.tf_buffer.lookup_transform('map','robot_1/base_link',rclpy.time.Time())
            self.get_logger().info(str(t.transform.translation.x) + "," + str(t.transform.translation.y))
        except Exception as ex:
            self.get_logger().info('Could not find transform')
        self.navigating = True
        self.get_logger().info("Logging" + str(msg.header.frame_id))
        if self.count == 0 :
            f = open("/home/depressedcoder/sample.txt","w")
            np_array = np.array(msg.data)
            np_array = np.reshape(np_array,(-1,msg.info.width))
            self.get_logger().info(str(np.any(np_array>=100)))
            #print(np.any(np_array>=255))
            np.savetxt(f,np_array)
            f.close()
            self.count = 1'''
            

def main(args = None):
        rclpy.init(args=args)
        exploration_node = TopologicalExplore()
        rclpy.spin(exploration_node)
        exploration_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    
