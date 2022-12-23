import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import message_filters
from message_filters import Subscriber


class MapProcessor(Node):
    
    def __init__(self):
        super().__init__('robot_1_map_processor')

        laser_sub = Subscriber('robot_1/laser/out',LaserScan)
        odom_sub  = Subscriber('robot_1/odom',Odometry)

        ts = message_filters.TimeSynchronizer([laser_sub,odom_sub],10)
        ts.registerCallback(self.map_callback)

    def map_callback(self,laser_scan,odom):
        self.get_logger().info(str(odom.pose.pose.position.x))
    

def main(args=None):
    rclpy.init(args=args)
    processor = MapProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()


if __name__== "__main__":
    main()
