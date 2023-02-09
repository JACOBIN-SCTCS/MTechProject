import rclpy
from rclpy.node import Node
from message_filters import Subscriber
from nav_msgs.msg import OccupancyGrid

class TopologicalExplore(Node):

    def __init__(self,robot_name='robot_1'):
        super().__init__(robot_name+'_explore')
        self.navigating = False

        self.robot_name = robot_name
        #map = Subscriber(self,OccupancyGrid,'/global_costmap/costmap')
        self.map_subscriber = self.create_subscription(OccupancyGrid,
            '/global_costmap/costmap',
            self.map_callback,
            10
        )
        self.map_subscriber

    def map_callback(self,msg:OccupancyGrid):
        if self.navigating:
            return

        self.navigating = True
        self.get_logger().info("Logging" + str(msg.header.frame_id))

def main(args = None):
        rclpy.init(args=args)
        exploration_node = TopologicalExplore()
        rclpy.spin(exploration_node)
        exploration_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    
