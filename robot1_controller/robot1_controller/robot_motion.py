import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MotionRobot1(Node):

    def __init__(self):
        super().__init__('robot_1_motion_handler')
        
        self.count = 0
        self.range_threshhold = 0.6

        self.twist_publisher = self.create_publisher(Twist,'/robot_1/cmd_vel',10)

        self.range_subscription = self.create_subscription(
            LaserScan,
            '/robot_1/ultrasonic_sensor_1',
            self.range_callback,
            10
        )
        self.range_subscription


    def range_callback(self,msg : LaserScan):
        twist_message = Twist()
        range_values = msg.ranges

        if(range_values[3] <= self.range_threshhold or range_values[4] <= self.range_threshhold or range_values[5] <= 0.4):
            twist_message.angular.z = -0.3
        elif (range_values[3]<= self.range_threshhold or range_values[2] <= self.range_threshhold or range_values[1] <= 0.4 ):
            twist_message.angular.z = 0.3
        else:
            twist_message.linear.x = 0.3
        
        self.twist_publisher.publish(twist_message)


def main(args=None):
    rclpy.init(args=args)
    motion_robot1 = MotionRobot1()
    rclpy.spin(motion_robot1)
    motion_robot1.destroy_node()
    rclpy.shutdown()


if __name__== "__main__":
    main()
