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


class MapProcessor(Node):
    
    def __init__(self,robot_name='robot_1',world_size=20):
    
        super().__init__(robot_name+'_map_processor')
        self.robot_name = robot_name
        self.robot_number = int(self.robot_name[6:])

        self.RESOLUTION = 0.01
        self.WORLD_SIZE = world_size
        self.GRID_SIZE = int(world_size/self.RESOLUTION) + 1
        self.WORLD_ORIGIN_X = -(self.WORLD_SIZE/2.0)
        self.WORLD_ORIGIN_Y = -(self.WORLD_SIZE/2.0)


        self.NON_OCC_PROB = 0.2
        self.OCC_PROB = 0.8
        self.PRIOR_PROB = 0.5
        self.P0 = 0.1


        self.l_occ = self.log_prob(self.OCC_PROB)
        self.l_prior = self.log_prob(self.PRIOR_PROB)
        self.l_non_occ = self.log_prob(self.NON_OCC_PROB)
        self.l0 = self.log_prob(self.P0)


        self.CONVERSION_FACTOR = 50
        self.RANGE_MAX = 1.0

        self.tf_publisher = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'world_frame'
        tf.child_frame_id = self.robot_name+'_map'
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self.tf_publisher.sendTransform(tf)
        
        
        cell_prob = round(self.actual_prob(self.l_prior),2)
        cell_value = int(100*cell_prob)
        self.occupancy_grid_data = np.array([cell_value for i in range(self.GRID_SIZE*self.GRID_SIZE)])

        now = self.get_clock().now()
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header.stamp = now.to_msg()
        self.occupancy_grid.info.width = self.GRID_SIZE
        self.occupancy_grid.info.height = self.GRID_SIZE
        self.occupancy_grid.info.resolution = self.RESOLUTION
        self.occupancy_grid.header.frame_id = self.robot_name+'_map'
        self.occupancy_grid.info.origin.position.x = self.WORLD_ORIGIN_X
        self.occupancy_grid.info.origin.position.y = self.WORLD_ORIGIN_Y
        self.occupancy_grid.data = self.occupancy_grid_data.tolist()
        

        self.log_map = np.array([[self.l_prior for i in range(self.GRID_SIZE)] for j in range(self.GRID_SIZE)])

        self.tf_buffer = Buffer()
        self.tf_listener =  TransformListener(self.tf_buffer,self)


        self.map_publisher = self.create_publisher(OccupancyGrid,'robot1_map',qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            ))
        self.publish_map()
        self.scanner_subscriber = self.create_subscription(LaserScan,self.robot_name+'/laser/out',self.update_map,10)
        self.scanner_subscriber


        #self.create_timer(1,self.publish_map)

      


        

    def log_prob(self,p_x):
        return math.log(p_x / (1-p_x))

    def actual_prob(self,log_prob):
        result = float(1.0 - (1.0/(1+np.exp(log_prob))))
        if np.isnan(result):
            result = 0.0
        return result


    def publish_map(self):
        now = self.get_clock().now()
        self.occupancy_grid.header.stamp = now.to_msg()
        self.occupancy_grid.data = self.occupancy_grid_data.tolist()
        self.map_publisher.publish(self.occupancy_grid)


    def update_map(self, msg: LaserScan):
        robot_rotation = None
        robot_translation = None

        try:
            transformation = self.tf_buffer.lookup_transform(target_frame='chassis'+str(self.robot_number),source_frame='odom',time=msg.header.stamp)
            q = transformation.transform.rotation
            robot_rotation = math.atan2(+2.0 * (q.w * q.z + q.x * q.y),+1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            robot_translation = transformation.transform.translation
            #self.get_logger().info(str(robot_translation))
        except:
            return
        
        lidar_coord_values = []
        laser_range_angle = robot_rotation + msg.angle_min
        for laser_range in msg.ranges:
            if laser_range < msg.range_min or laser_range > msg.range_max:
                continue
            world_laser_x = robot_translation.x + laser_range * math.cos(laser_range_angle)
            world_laser_y = robot_translation.y + laser_range * math.sin(laser_range_angle)
            lidar_sensor_value = self.l_occ
            if(laser_range == msg.range_max):
                lidar_sensor_value = self.l_non_occ
            lidar_coord_values.append((world_laser_x,world_laser_y,lidar_sensor_value))
            laser_range_angle = laser_range_angle + msg.angle_increment
        
        robot_position_grid = (int(robot_translation.x/self.RESOLUTION), int(robot_translation.y/self.RESOLUTION))
        for lidar_vals in lidar_coord_values:
            laser_position_grid = (int(lidar_vals[0]/self.RESOLUTION),int(lidar_vals[1]/self.RESOLUTION))
            points = self.bressenhams_algorithm(robot_position_grid,laser_position_grid)[:-1]

            for i in range(len(points)):
                self.log_map[(self.GRID_SIZE>>1) + points[i][0],(self.GRID_SIZE>>1) + points[i][1]] += (self.l_non_occ - self.l0)
                cell_prob = self.actual_prob(self.log_map[(self.GRID_SIZE>>1) + points[i][0],(self.GRID_SIZE>>1) + points[i][1]])
                cell_value = int(100*cell_prob)
                self.occupancy_grid_data[(points[i][0]-int(self.WORLD_ORIGIN_X/self.RESOLUTION))*(int(self.GRID_SIZE)) + (points[i][1]-int(self.WORLD_ORIGIN_Y/self.RESOLUTION))] = cell_value
            
            self.log_map[(self.GRID_SIZE>>1) + laser_position_grid[0],(self.GRID_SIZE>>1) +  laser_position_grid[1]] += ( lidar_vals[2] - self.l0)
            cell_prob = self.actual_prob(self.log_map[(self.GRID_SIZE>>1) + laser_position_grid[0],(self.GRID_SIZE>>1) + laser_position_grid[1]])
            cell_value = int(100*cell_prob)
            self.occupancy_grid_data[(laser_position_grid[0]-int(self.WORLD_ORIGIN_X/self.RESOLUTION))*(int(self.GRID_SIZE)) + (laser_position_grid[1]-int(self.WORLD_ORIGIN_Y/self.RESOLUTION))] = cell_value
        
        for i in range(self.GRID_SIZE):
            self.occupancy_grid_data[i*self.GRID_SIZE + i] = 0
        self.get_logger().info("new data has been added")   
        self.publish_map()

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


    def bressenhams_algorithm(self,coord0,coord1):
        x0 = coord0[0]
        y0 = coord0[1]
        x1 = coord1[0]
        y1 = coord1[1]

        dx = abs(x1-x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1-y0)
        sy = 1 if y0 < y1 else -1

        error = dx + dy

        tmp_x = x0
        tmp_y = y0
        coordinate_points  = []
        while True:
            if(tmp_x==x1 and tmp_y == y1 ):
                break
            e2 = 2*error
            if e2 >= dy:
                if tmp_x==x1:
                    break
                error = error + dy
                tmp_x = tmp_x + sx
            if e2 <= dx:
                if tmp_y == y1:
                    break
                error = error + dx
                tmp_y = tmp_y + sy
            coordinate_points.append((tmp_x,tmp_y))
        return coordinate_points



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