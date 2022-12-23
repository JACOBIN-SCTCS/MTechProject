import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import message_filters


class MapProcessor(Node):
    
    def __init__(self):
        super().__init__('robot_1_map_processor')
        self.subscriber =  self.create_subscription(
            Odometry,
            'robot_1/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self,msg):
        self.get_logger().info(str(msg.pose.pose.position.x))
    

def main(args=None):
    rclpy.init(args=args)
    processor = MapProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()


if __name__== "__main__":
    main()
