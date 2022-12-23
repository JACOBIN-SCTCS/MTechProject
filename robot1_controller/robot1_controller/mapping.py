import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
import message_filters
from message_filters import Subscriber
from rclpy.qos import QoSProfile,DurabilityPolicy,HistoryPolicy
import math
import numpy as np


class MapProcessor(Node):
    
    def __init__(self):
        super().__init__('robot_1_map_processor')

        self.GRID_SIZE = 20 * 100
        self.NON_OCC_PROB = 0.2
        self.OCC_PROB = 0.8
        self.PRIOR_PROB = 0.5

        self.CONVERSION_FACTOR = 100
        self.RANGE_MAX = 1.0

        self.l_occ = self.log_prob(self.OCC_PROB)
        self.l_prior = self.log_prob(self.PRIOR_PROB)
        self.l_non_occ = self.log_prob(self.NON_OCC_PROB)


        laser_sub = Subscriber(self,LaserScan,'robot_1/laser/out')
        odom_sub  = Subscriber(self,Odometry,'robot_1/odom')

        ts = message_filters.TimeSynchronizer([laser_sub,odom_sub],10)
        ts.registerCallback(self.map_callback)

        self.map_publisher = self.create_publisher(OccupancyGrid,'robot1_map',qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            ))

        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.info.width = self.GRID_SIZE
        self.occupancy_grid.info.height = self.GRID_SIZE
        self.occupancy_grid.info.resolution = 0.01
        self.occupancy_grid.header.frame_id = 'robot1_map'
        self.occupancy_grid.data = [-1 for i in range(self.GRID_SIZE*self.GRID_SIZE)]
        self.log_map = np.array([[self.l_prior for i in range(self.GRID_SIZE)] for j in range(self.GRID_SIZE)])

    def log_prob(self,p_x):
        return math.log(p_x / (1-p_x))
    
    def map_callback(self,laser_scan : LaserScan,odom : Odometry):
        
        self.get_logger().info(str(odom.pose.pose.position.x))
        
        MIN_ANGLE = laser_scan.angle_min
        ANGLE_INCREMENT = laser_scan.angle_increment
        current_angle = MIN_ANGLE

        try:
            grid_x_position = 0

        except:
            print(" ")
        


def main(args=None):
    rclpy.init(args=args)
    processor = MapProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()


if __name__== "__main__":
    main()
