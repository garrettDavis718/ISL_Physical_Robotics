import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Vector3
import time

qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

class MovementCommands(Node):
 def __init__(self):
    #Super init
        super().__init__("movement_command_node")
        self.sub = self.create_subscription(LaserScan, "/scan",
         self.subscriber_callback, qos_policy)
        #init pub for movement
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

def subscriber_callback(self, move_cmd=Twist()):

    def turn_right(self, move_cmd = Twist()):
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = -0.2
        self.pub.publish(move_cmd)
        print('turning right')    

    def turn_left(self, move_cmd = Twist()):
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.2
        self.pub.publish(move_cmd)
        print('turning left')
        
    def move_forward(self, move_cmd = Twist()):
        move_cmd.linear.x = 0.2
        move_cmd.angular.z = 0.0
        print('moving forward')
        #publish command
        self.pub.publish(move_cmd)
        
    def move_backward(self, move_cmd = Twist()):
        move_cmd.linear.x = -0.2
        move_cmd.angular.z = 0.0
        print('moving backward')
        #publish command
        self.pub.publish(move_cmd)