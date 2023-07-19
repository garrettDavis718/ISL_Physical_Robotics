import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  #/cmd_vel publisher

class MovementCommands(Node):
    def __init__(self):
        #Super init
        super().__init__("movement_command_node")
        #init pub for movement
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
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
