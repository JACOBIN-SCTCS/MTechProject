import rclpy
from rclpy.node import Node
from message_filters import Subscriber
from nav_msgs.msg import OccupancyGrid
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np

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

    def map_callback(self,msg:OccupancyGrid):
        #if self.navigating:
        #    return
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

    
