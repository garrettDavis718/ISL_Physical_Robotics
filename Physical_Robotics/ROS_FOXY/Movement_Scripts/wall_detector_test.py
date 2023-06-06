#simple detection

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Vector3
import time

#Setup our Node


qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
class WallAvoider(Node):
    def __init__(self):
    #Super init
        super().__init__("wall_avoider_node")
        #init sub attribute for Lidar Scan
        self.sub = self.create_subscription(LaserScan, "/scan",
         self.subscriber_callback, qos_policy)
        #init velocity pub
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
 
    #Callback Method for lidar scan data		
    def subscriber_callback(self, msg : LaserScan, move_cmd = Twist()):
    	#display distances

        '''GAZEBO VALUES FOR msg.ranges: 0, 360
        REAL TIME VALUES FOR msg.ranges: 0, 250'''

        front_left_val= msg.ranges[340:359]
        front_right_val= msg.ranges[0:20]
        front_vals = list([front_left_val, front_right_val])
        print(f'front left val: {front_left_val}')
        print(f'front right val: {front_right_val}')

        if min(front_right_val) < 0.8 and min(front_left_val) > min(front_right_val):
            self.turn_right()
            time.sleep(6)
        elif min(front_right_val) < 0.8:
            self.turn_left()
            time.sleep(6)
        else:
            self.move_forward()

    '''Turtlebot motor cannot handle speeds >= 0.3
       Positive angular.z values == right turn
       Negative angular.z values == left turn '''
    
    #turn_right and turn_left are potentially inversed, will fix soon
    
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



        





#Main
def main():
    #init node
    rclpy.init()
    #Instantiante our created node
    my_node = WallAvoider()
    print("Waiting for data to be published over the topic...")
    #try catch for "spinning" our node
    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        #publish move_cmd with no movement to have it stop
        stop = Twist()
        my_node.pub.publish(stop)
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()