import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
import message_filters
from message_filters import Subscriber
from rclpy.qos import QoSProfile,DurabilityPolicy,HistoryPolicy,ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster,Buffer,TransformListener
import math
import numpy as np
import threading


class MapProcessor(Node):
    
    def __init__(self,robot_name='robot_1',world_size=20):
        
        self.robot_name = robot_name
        self.robot_number = int(self.robot_name[6:])
        super().__init__(self.robot_name+'_map_processor')
        
        self.RANGE_MAX = 1.0

 
        self.RESOLUTION = 0.01
        self.WORLD_SIZE = world_size
        self.GRID_SIZE = self.WORLD_SIZE / self.RESOLUTION

        # Initialize the probability values for the cell being an obstacle,
        # free or belonging to the unknown region.
        self.NON_OCC_PROB = 0.2
        self.OCC_PROB = 0.8
        self.PRIOR_PROB = 0.5
        
        self.l_occ = self.log_prob(self.OCC_PROB)
        self.l_prior = self.log_prob(self.PRIOR_PROB)
        self.l_non_occ = self.log_prob(self.NON_OCC_PROB)
        
        self.log_map = np.array([[self.l_prior for i in range(self.GRID_SIZE)] for j in range(self.GRID_SIZE)])
        self.map_publisher = self.create_publisher(OccupancyGrid,self.robot_name + '_map',qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            ))

        self._map_lock = threading.Lock()
        
        self.tf_publisher = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'world_frame'
        tf.child_frame_id = self.robot_name+'_map'
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self.tf_publisher.sendTransform(tf)


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.scanner_subscriber = self.create_subscription(LaserScan,self.robot_name+'/laser/out')
        self.publish_map()
        self.create_timer(1,self.publish_map)




        self.CONVERSION_FACTOR = 50

   



        # NO NEED FOR APPROPIRATE TIME SYNCHRONIZER
        laser_sub = Subscriber(self,LaserScan,'robot_1/laser/out')
        odom_sub  = Subscriber(self,Odometry,'robot_1/odom')

        #ts = message_filters.TimeSynchronizer([laser_sub,odom_sub],10)
        ts = message_filters.ApproximateTimeSynchronizer([laser_sub,odom_sub],20,0.5)
        ts.registerCallback(self.map_callback)        
        # TO DELETE
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.info.width = self.GRID_SIZE
        self.occupancy_grid.info.height = self.GRID_SIZE
        self.occupancy_grid.info.resolution = 0.01
        self.occupancy_grid.header.frame_id = 'robot1_map'
        self.occupancy_grid.data = [-1 for i in range(self.GRID_SIZE*self.GRID_SIZE)]



    def log_prob(self,p_x):
        return np.log(p_x / (1.0-p_x))

    def prob(self,log_prob):
        result = np.exp(log_prob) / (1.0 + np.exp(log_prob))
        if math.isnan(result):
            result = 0.0
        return result

    def publish_map(self):
        self._map_lock.acquire()
        now = self.get_clock().now()
        prob_map = [-1]*self.GRID_SIZE*self.GRID_SIZE
        idx = 0

        for i in range(self.GRID_SIZE):
            for j in range(self.GRID_SIZE):
                cell_prob= round(self.prob(self.log_map[i,j]),2)
                prob_map[idx] = int(cell_prob*100)
                idx+=1
        msg = OccupancyGrid()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.robot_name+'_map'
        msg.info.height = self.GRID_SIZE
        msg.info.width =  self.GRID_SIZE
        msg.info.origin.position.x = -(self.WORLD_SIZE/2.0)
        msg.info.origin.position.y = -(self.WORLD_SIZE/2.0)
        msg.data = prob_map
        self.map_publisher.publish(msg)
        self._map_lock.release()


    def update_map(self, msg: LaserScan):
        if self._map_lock.locked():
            return
        else:
            self._map_lock.acquire()
        
        robot_rotation = None
        robot_translation = None

        try : 
            transformation = self.tf_buffer.lookup_transform(target_frame='chassis'+str(self.robot_number),source_frame='odom',time=msg.header.stamp)
            q = transformation.transform.rotation
            robot_rotation = math.atan2(
                                +2.0 * (q.w * q.z + q.x * q.y),
                                +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                            )
            robot_translation = transformation.transform.translation
        except:
            self._map_lock.release()
            return 


    def quarternion_to_euler(self,quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


    def map_callback(self,laser_scan : LaserScan,odom : Odometry):
                
        MIN_ANGLE = laser_scan.angle_min
        ANGLE_INCREMENT = laser_scan.angle_increment
        current_angle = MIN_ANGLE

        try:
            grid_x_position = int(self.CONVERSION_FACTOR*odom.pose.pose.position.x)
            grid_y_poisition = int(self.CONVERSION_FACTOR*odom.pose.pose.position.y)
            orientation = self.quarternion_to_euler(odom.pose.pose.orientation)
        
            for i in range(len(laser_scan.ranges)):
                measurement = self.CONVERSION_FACTOR*min(laser_scan.range_max,laser_scan.ranges[i])
                reflected_x_position = int(grid_x_position + measurement*math.cos(current_angle + orientation[2]))
                reflected_y_position = int(grid_y_poisition + measurement*math.sin(current_angle + orientation[2]))
                current_angle+= ANGLE_INCREMENT
                self.bressenhams_line_drawing_algorithm((grid_x_position,grid_y_poisition),(reflected_x_position,reflected_y_position),measurement)
                self.map_publisher.publish(self.occupancy_grid)

        except:
            print(" ")
        
    # Reference Taken from a repository
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
        
        for i in range(len(points)-1):
            self.log_map[(self.GRID_SIZE>>1)-1 + points[i][0]][(self.GRID_SIZE>>1)-1+ points[i][1]] += self.l_non_occ
            
            try:
                probability_value  = 1 - 1/(1+math.exp(self.log_map[(self.GRID_SIZE>>1)-1 + points[i][0]][(self.GRID_SIZE>>1)-1+ points[i][1]]))
            except OverflowError:
                probability_value = float('-inf')
                self.get_logger().info("Overflow error")



            if(probability_value > 0.5):
                self.occupancy_grid.data[((self.GRID_SIZE>>1)-1 + points[i][0])*self.GRID_SIZE + ((self.GRID_SIZE>>1) -1 + points[i][1])] = 100
            else:
                self.occupancy_grid.data[((self.GRID_SIZE>>1)-1 + points[i][0])*self.GRID_SIZE + ((self.GRID_SIZE>>1) -1 + points[i][1])] = 0
        

        if sensor_measurement < (self.CONVERSION_FACTOR*self.RANGE_MAX):
            self.log_map[(self.GRID_SIZE>>1)-1 + points[len(points)-1][0]][(self.GRID_SIZE>>1)-1 + points[len(points)-1][1]] += self.l_occ
        else:
            self.log_map[(self.GRID_SIZE>>1)-1 + points[len(points)-1][0]][(self.GRID_SIZE>>1)-1 + points[len(points)-1][1]] += (self.l_prior)


        try:
            probability_value = max(float('-inf'), 1 - float(1/(1 + math.exp(self.log_map[(self.GRID_SIZE>>1)-1 + points[len(points)-1][0]][(self.GRID_SIZE>>1)-1+ points[len(points)-1][1]]))))
        except OverflowError:
            probability_value = float('-inf')
            self.get_logger().info("Overflow error")


        if(probability_value >= 0.8):
            self.occupancy_grid.data[((self.GRID_SIZE>>1)-1 + points[len(points)-1][0])*self.GRID_SIZE + ((self.GRID_SIZE>>1) -1 + points[len(points)-1][1])] = 100
        else:
            self.occupancy_grid.data[((self.GRID_SIZE>>1)-1 + points[len(points)-1][0])*self.GRID_SIZE + ((self.GRID_SIZE>>1) -1 + points[len(points)-1][1])] = 0

        return points

def main(args=None):
    rclpy.init(args=args)
    processor = MapProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()


if __name__== "__main__":
    main()
