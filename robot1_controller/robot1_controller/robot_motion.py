import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class MotionRobot1(Node):

    def __init__(self):
        super().__init__('robot_1_motion_handler')
        
        self.waypoints_to_follow = [ 
                (0,-1),
                (0,-2), (0,-3) , (0,-4),
                (-1,-5),
                (-2,-6),
                (-3,-6),
        ]
        self.absolute_difference_threshhold = 0.1


        self.current_waypoint_index = 0
        self.count = 0
        self.range_threshhold = 0.6
        self.twist_publisher = self.create_publisher(Twist,'/robot_1/cmd_vel',10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/robot_1/odom',
            self.odom_callback,
            10
        )
        

        '''self.range_subscription = self.create_subscription(
            LaserScan,
            '/robot_1/ultrasonic_sensor_1',
            self.range_callback,
            10
        )
        self.range_subscription'''
        self.odom_subscription


    '''def range_callback(self,msg : LaserScan):
        twist_message = Twist()
        range_values = msg.ranges

        if(range_values[3] <= self.range_threshhold or range_values[4] <= self.range_threshhold or range_values[5] <= 0.4):
            twist_message.angular.z = -0.3
        elif (range_values[3]<= self.range_threshhold or range_values[2] <= self.range_threshhold or range_values[1] <= 0.4 ):
            twist_message.angular.z = 0.3
        else:
            twist_message.linear.x = 0.3
        
        self.twist_publisher.publish(twist_message)'''


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

    def odom_callback(self,msg : Odometry):
        
        twist_message = Twist()
        position = msg.pose.pose.position
        orientation  = self.quarternion_to_euler(msg.pose.pose.orientation)
        destination = self.waypoints_to_follow[self.current_waypoint_index]

        inc_x = destination[0] - position.x
        inc_y = destination[1] - position.y
        if((inc_x*inc_x + inc_y*inc_y) <= 0.001):
            self.twist_publisher.publish(twist_message)
            return
            

        angle_to_goal = math.atan2(inc_y,inc_x)
        if(abs(angle_to_goal - orientation[2]) > self.absolute_difference_threshhold):
            self.get_logger().info(str(abs(angle_to_goal - orientation[2])))
            twist_message.angular.z = -0.1
            twist_message.linear.x = 0.0
        else:
            twist_message.linear.x = 0.1
            twist_message.angular.z = 0.0

        self.twist_publisher.publish(twist_message)



def main(args=None):
    rclpy.init(args=args)
    motion_robot1 = MotionRobot1()
    rclpy.spin(motion_robot1)
    motion_robot1.destroy_node()
    rclpy.shutdown()


if __name__== "__main__":
    main()
