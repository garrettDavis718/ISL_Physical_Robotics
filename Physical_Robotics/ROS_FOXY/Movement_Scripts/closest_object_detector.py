import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import LaserScan # LaserScan is another subscriber
from geometry_msgs.msg import Twist # Twist data to move robot
import cv2 # OpenCV library

qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
#Create Node Class
class LidarNode(Node):
    '''Node that finds closest LIDAR scan distance & turns to that distance'''
    def __init__(self):
        super().__init__("laser_subscriber_node")
        self.lidarsub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, qos_policy)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
    def lidar_callback(self, msg : LaserScan, move_cmd = Twist(), object_found = False, min = 3.5):
        print('test1')
        front_val = msg.ranges[0]
        print(msg.ranges[0])
        for val in msg.ranges:
            if val <= min and val != "inf":
                min = val
        print(min)
        if msg.ranges[0] - 0.01 > min:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.2
            self.pub.publish(move_cmd)
        else:
            move_cmd = Twist()
            self.pub.publish(move_cmd)
            print("object found")
            self.destroy_node()



def main():
    rclpy.init()
    my_node = LidarNode()
    print("Waiting for data to be published over the topic...")
    try:
        print('test3')
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
