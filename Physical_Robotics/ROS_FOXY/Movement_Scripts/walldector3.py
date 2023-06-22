#simple detection

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
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
        front_left_val= msg.ranges[0:30]
        front_right_val= msg.ranges[220:250]
        print(f'front left val: {front_left_val}')
        print(f'front right val: {front_right_val}')
        #if statement, check for distance in front
        #publish movement(straight or turn) dependant on distances
        #Turtlebot motor cannot handle speeds >= 0.3
        if min(front_left_val) < 1 or min(front_right_val) < 1:
            #build command
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.2
            print('moving forward')
            self.pub.publish(move_cmd)
        else:
            #build command
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0.0
            #move_cmd.angular.x = 0.1
            print('wall found')
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
