import csv 
import rclpy
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import LaserScan # LaserScan is another subscriber
import time
from geometry_msgs.msg import Twist
import cv2 # OpenCV library
from cv_bridge import CvBridge
import numpy as np
import sys
from sensor_msgs.msg import Image #image to publish image with green
from object_class import Object

qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

class WallAvoider(Node):
    def __init__(self):
        self.object_list = read_csv()
        self.current_degree = 0
    #Super init
        super().__init__("wall_avoider_node")
        #init sub attribute for Lidar Scan
        self.sub = self.create_subscription(LaserScan, "/scan",
         self.suscriber_callback, qos_policy)
        #init velocity pub
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def suscriber_callback(self, msg: LaserScan, move_cmd = Twist()):
        self.turn_left()
        #time.sleep(32.52/4)
        #ime.sleep((len(msg.ranges)/32.54)*self.object_list[0].location)
        time.sleep(self.object_list[0].location/260*32.5)
        self.pub.publish(Twist())
        sys.exit()

    def turn_right(self, move_cmd = Twist()):
       
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = -0.2
        #publish command
        self.pub.publish(move_cmd)
        print('Turning right')
        
    
    def turn_left(self, move_cmd = Twist()):
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.2
        #publish command
        self.pub.publish(move_cmd)
        print('Turning left')
        
    
    def move_forward(self, move_cmd = Twist()):
        move_cmd.linear.x = 0.2
        move_cmd.angular.z = 0.0
        #publish command
        self.pub.publish(move_cmd)
        print('Moving forward')

    def move_backward(self, move_cmd = Twist()):
        move_cmd.linear.x = -0.2
        move_cmd.angular.z = 0.0
        #publish command
        self.pub.publish(move_cmd)
        print('Moving backward')

def read_csv():
    new_list = []
    with open('/home/gardongo/physical_robotics/nearby_objects.csv', 'r') as f:
        reader = csv.reader(f)
        for lines in reader:
            new_obj = Object(lines[0],int(lines[1]),float(lines[2]),lines[3])
            new_list.append(new_obj)
    return new_list

def main():
    rclpy.init(args=None)
    my_node = WallAvoider()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()