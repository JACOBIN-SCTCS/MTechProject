import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class MotionRobot1(Node):

    def __init__(self,robot_name='robot_1'):
        super().__init__(robot_name+'_motion_handler')
        self.robot_name =  robot_name
        self.waypoints_to_follow = [ 
                (0,1),
                (0,2),
                (0,3)
        ]
        self.absolute_difference_threshhold = 0.1


        self.current_waypoint_index = 0
        self.count = 0
        self.range_threshhold = 0.6
        self.twist_publisher = self.create_publisher(Twist,'/'+self.robot_name+'/cmd_vel',10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/'+self.robot_name+'/odom',
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


    def odom_callback(self,msg : Odometry):
        
        twist_message = Twist()
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation    
        t1 = +2.0 * (q.w * q.z + q.x * q.y)
        t2 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    
        orientation  = math.atan2(t1,t2)
        if(self.current_waypoint_index == len(self.waypoints_to_follow)):
                self.get_logger().info("completed all waypoints")
                return twist_message
        destination = self.waypoints_to_follow[self.current_waypoint_index]

        inc_x = destination[0] - position.x
        inc_y = destination[1] - position.y
        if((inc_x*inc_x + inc_y*inc_y) <= 0.01):
            self.twist_publisher.publish(twist_message)
            self.current_waypoint_index +=1 
          

        angle_to_goal = math.atan2(inc_y,inc_x)
        if(abs(angle_to_goal - orientation) > self.absolute_difference_threshhold):
            #self.get_logger().info(str(abs(angle_to_goal - orientation[2])))
            twist_message.angular.z = 0.3
            twist_message.linear.x = 0.0
        else:
            twist_message.linear.x = 0.3
            twist_message.angular.z = 0.0

        self.twist_publisher.publish(twist_message)



def main(args=None):
    rclpy.init(args=args)
    motion_robot = MotionRobot1(robot_name='robot_1')
    rclpy.spin(motion_robot)
    motion_robot.destroy_node()
    rclpy.shutdown()


if __name__== "__main__":
    main()
