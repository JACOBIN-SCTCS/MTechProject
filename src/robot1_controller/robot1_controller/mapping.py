import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
import message_filters
from message_filters import Subscriber
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile,DurabilityPolicy,HistoryPolicy,ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster,TransformListener, Buffer
import tf2_ros
import math
import numpy as np
import sys
from rclpy.task import Future

class MapProcessor(Node):
    
    def __init__(self,robot_name='robot_1',world_size=20):
        super().__init__(robot_name+'_map_processor')

        self.robot_name = robot_name
        self.robot_number = int(self.robot_name[6:])
        self.WORLD_SIZE = world_size
        self.RESOLUTION = 0.01
        self.GRID_SIZE = int(self.WORLD_SIZE/self.RESOLUTION)

        self.NON_OCC_PROB = 0.2
        self.OCC_PROB = 0.8
        self.PRIOR_PROB = 0.5

        self.CONVERSION_FACTOR = 100
        self.RANGE_MAX = 1.0

        self.l_occ = self.log_prob(self.OCC_PROB)
        self.l_prior = self.log_prob(self.PRIOR_PROB)
        self.l_non_occ = self.log_prob(self.NON_OCC_PROB)


        laser_sub = Subscriber(self,LaserScan,self.robot_name+'/laser/out')
        odom_sub  = Subscriber(self,Odometry,self.robot_name+'/odom')

        ts = message_filters.TimeSynchronizer([laser_sub,odom_sub],10)
        ts = message_filters.ApproximateTimeSynchronizer([laser_sub,odom_sub],20,0.5)
        ts.registerCallback(self.map_callback)

        self.map_publisher = self.create_publisher(OccupancyGrid,self.robot_name+'_map',qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            ))
        
        self.tf_publisher = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'world_frame'
        tf.child_frame_id = self.robot_name+'_map'
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self.tf_publisher.sendTransform(tf)

        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.info.width = self.GRID_SIZE
        self.occupancy_grid.info.height = self.GRID_SIZE
        self.occupancy_grid.info.resolution = 0.01
        self.occupancy_grid.header.frame_id = self.robot_name+'_map'
        self.occupancy_grid.data = [-1 for i in range(self.GRID_SIZE*self.GRID_SIZE)]
        self.occupancy_grid.info.origin.position.x  = -self.WORLD_SIZE/2
        self.occupancy_grid.info.origin.position.y  = -self.WORLD_SIZE/2

        self.log_map = np.array([[self.l_prior for i in range(self.GRID_SIZE)] for j in range(self.GRID_SIZE)])


        #self.tf_buffer = Buffer()
        #self.tf_listener =  TransformListener(self.tf_buffer,self)
        #self.laser_subscriber = self.create_subscription(LaserScan,self.robot_name+'/laser/out',self.map_callback_no_odom,10)

    def log_prob(self,p_x):
        return math.log(p_x / (1.0-p_x))
    
    def prob_value(self,log_prob):
        try:
            prob_value = 1.0 - 1.0/(1.0+math.exp(log_prob))
        except:
            prob_value = 0.0 
        return prob_value

    def map_callback(self,laser_scan : LaserScan,odom : Odometry):
                
        MIN_ANGLE = laser_scan.angle_min
        ANGLE_INCREMENT = laser_scan.angle_increment
        current_angle = MIN_ANGLE

        try:
            grid_x_position = int(self.CONVERSION_FACTOR*odom.pose.pose.position.x)
            grid_y_poisition = int(self.CONVERSION_FACTOR*odom.pose.pose.position.y)
            q = odom.pose.pose.orientation
            yaw = math.atan2(+2.0 * (q.w * q.z + q.x * q.y),+1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
            for i in range(len(laser_scan.ranges)):
                measurement = self.CONVERSION_FACTOR*min(laser_scan.range_max,laser_scan.ranges[i])
                reflected_x_position = int(grid_x_position + measurement*math.cos(current_angle + yaw))
                reflected_y_position = int(grid_y_poisition + measurement*math.sin(current_angle + yaw))
                current_angle+= ANGLE_INCREMENT
                self.bressenhams_line_drawing_algorithm((grid_x_position,grid_y_poisition),(reflected_x_position,reflected_y_position),measurement)
                now = self.get_clock().now()
                self.occupancy_grid.header.stamp = now.to_msg()
                self.map_publisher.publish(self.occupancy_grid)

        except Exception as ex:
            template = "An exception of type {0} occurred. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            self.get_logger().info(message)
        
    def bressenhams_line_drawing_algorithm(self,coord0,coord1,sensor_measurement):
        
        x0 = coord0[0]
        y0 = coord0[1]
        x1 = coord1[0]
        y1 = coord1[1]
        dx = x1 - x0
        dy = y1 - y0

        is_steep = abs(dy) > abs(dx)
        if is_steep:
            x0,y0  = y0,x0
            x1,y1 = y1,x1
        swapped = False
        if(x0 > x1):
            x0 , x1 = x1 , x0 
            y0 , y1 = y1 , y0
            swapped = True
        dx = x1 - x0
        dy = y1 - y0
        error = int(dx / 2.0)
        ystep = 1 if y0 < y1 else -1
        y = y0
        points = []
        for x in range(x0,x1+1):
            coord = (y,x) if is_steep else (x,y)
            points.append(coord)
            
            error -= abs(dy)
            if error < 0 :
                y += ystep
                error += dx
        if swapped:
            points.reverse()
        

        last_point = ((self.GRID_SIZE>>1)-1 + points[len(points)-1][0],(self.GRID_SIZE>>1)-1 + points[len(points)-1][1])
        last_point_flattened = ((self.GRID_SIZE>>1)-1 + points[len(points)-1][0])*self.GRID_SIZE + ((self.GRID_SIZE>>1) -1 + points[len(points)-1][1])
        for i in range(len(points)-1):
            current_point = ((self.GRID_SIZE>>1)-1 + points[i][0] , (self.GRID_SIZE>>1)-1+ points[i][1] )
            flattened_point = ((self.GRID_SIZE>>1)-1 + points[i][0])*self.GRID_SIZE + ((self.GRID_SIZE>>1) -1 + points[i][1]) 
            self.log_map[current_point] += self.l_non_occ
        
            probability_value  = self.prob_value(self.log_map[current_point])
            if(probability_value > 0.5):
                self.occupancy_grid.data[flattened_point] = 100
            else:
                self.occupancy_grid.data[flattened_point] = 0
        
        if sensor_measurement < (self.CONVERSION_FACTOR*self.RANGE_MAX):
            self.log_map[last_point] += self.l_occ
        else:
            self.log_map[last_point] += (self.l_prior)

        probability_value = max(float('-inf'), self.prob_value(self.log_map[last_point]))
        if(probability_value >= 0.8):
            self.occupancy_grid.data[last_point_flattened] = 100
        else:
            self.occupancy_grid.data[last_point_flattened] = 0


def main():
    args = sys.argv[1:]
    rclpy.init()
    processor = MapProcessor(robot_name=args[0],world_size=int(args[1]))
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()


if __name__== "__main__":
    main()