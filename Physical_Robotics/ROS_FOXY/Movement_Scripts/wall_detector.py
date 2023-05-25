#simple detection

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
#Setup our Node
class WallAvoider(Node):
    def __init__(self):
    #Super init
        super().__init__("wall_avoider_node")
        #init sub attribute for Lidar Scan
        self.sub = self.create_subscription(LaserScan, "/scan",
         self.subscriber_callback, 10)
        #init velocity pub
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
 
    #Callback Method for lidar scan data		
    def subscriber_callback(self, msg : LaserScan, move_cmd = Twist()):
    	#display distances
        print(f"Front Distance : {msg.ranges[0]}")
        print(f"Back Distance : {msg.ranges[180]}")
        #if statement, check for distance in front
        #publish movement(straight or turn) dependant on distances
        if msg.ranges[0] > 1.5:
            #build command
            move_cmd.linear.x = 0.3
            move_cmd.angular.z = 0.0
        else:
            #build command
            move_cmd.linear.x = 0.1
            move_cmd.angular.z = 0.3\
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
