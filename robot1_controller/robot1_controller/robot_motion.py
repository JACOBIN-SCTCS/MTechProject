import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotionRobot1(Node):

    def __init__(self):
        super().__init__('robot_1_motion_handler')
        self.publisher =  self.create_publisher(Twist,'/robot_1/cmd_vel',10)
        self.count = 0
        self.timer = self.create_timer(10,self.timer_callback)
   
    def timer_callback(self):
        msg = Twist()
        if self.count%2==0:
            msg.angular.z = -3.14
        else:
            msg.angular.z = 3.14
        self.publisher.publish(msg)
        self.count = 1 - self.count



def main(args=None):
    rclpy.init(args=args)
    motion_robot1 = MotionRobot1()
    rclpy.spin(motion_robot1)
    motion_robot1.destroy_node()
    rclpy.shutdown()


if __name__== "__main__":
    main()
