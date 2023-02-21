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

        self.exploration_client = ActionClient(self,NavigateToPose,'navigate_to_pose')


    def goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Exploration goal rejected')
            return
        self.get_logger().info('Navigation goal rejected')
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
        goal_msg.pose.pose.position.x = 0.0
        goal_msg.pose.pose.position.y = -1.0
        
        self._send_goal_future = self.exploration_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        rclpy.spin_until_future_complete(self,self._send_goal_future)
        goal_handle = self._send_goal_future.result()
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self,get_result_future)



    def map_callback(self,msg:OccupancyGrid):
        if not self.navigating:
            self.send_goal()
            self.navigating = True
        try:
            t  = self.tf_buffer.lookup_transform('robot_1/base_footprint','map',rclpy.time.Time())
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
            self.count = 1
            

def main(args = None):
        rclpy.init(args=args)
        exploration_node = TopologicalExplore()
        rclpy.spin(exploration_node)
        exploration_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    
