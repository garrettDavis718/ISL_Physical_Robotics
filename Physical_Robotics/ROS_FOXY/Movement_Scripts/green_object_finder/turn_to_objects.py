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
from std_msgs.msg import String #string for whether green found


qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
path_to_csv = '/home/ubuntu/ISL_Physical_Robotics/Physical_Robotics/ROS_FOXY/Movement_Scripts/green_object_finder/nearby_objects.csv'
lower_green = np.array([48,63,63])
upper_green = np.array([98,255,255])

class WallAvoider(Node):
    def __init__(self):
        self.object_list = read_csv()
        self.current_degree = 0
        self.found_objects = []
    #Super init
        super().__init__("wall_avoider_node")
        #init sub attribute for Lidar Scan
        self.sub = self.create_subscription(LaserScan, "/scan",
         self.suscriber_callback, qos_policy)
        #init velocity pub
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        #self.green_img_sub = self.create_subscription(Image, "image_mask", self.green_sub_callback, 10)
        self.br = CvBridge()
        #self.cap = cv2.VideoCapture(0)
        #self.find_green = False
        self.green_found = False
    
    def suscriber_callback(self, msg: LaserScan, move_cmd = Twist()):
        #if not self.find_green:
        for obj in self.object_list:
            turn_loc = obj.location - self.current_degree
            self.turn_left()
            time.sleep(turn_loc/260*32.5)
            self.current_degree = obj.location
            self.pub.publish(Twist())
            #self.find_green = True
            obj.is_green = self.green_finder()
            self.found_objects.append(obj)
        write_csv(self.found_objects)
        print('all objects checked')
        sys.exit()

    '''def green_sub_callback(self, data: Image):
        #if self.find_green:
        #current_frame = self.br.imgmsg_to_cv2(data)
        ret, frame = self.cap.read()
        #wide = int(self.cap.get(3))
        #height = int(self.cap.get(4))
        #convert frame to hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #create masks
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        #results after checking mask against frame
        
        green_result = cv2.bitwise_and(frame, frame, mask=green_mask)
        #check if green exists
        if cv2.countNonZero(green_mask) > 0:
            self.green_found = True
            self.get_logger().info("Green Found!")
        else:
            self.green_found = False
            self.get_logger().info('No green found')
        #self.find_green = False
        #Display Image
        #cv2.imshow("image mask", current_frame)
    
        #cv2.waitKey(1)'''


    def green_finder(self):
        green_found = False
        cap = cv2.VideoCapture(0)
        #if self.find_green:
        #current_frame = self.br.imgmsg_to_cv2(data)
        ret, frame = cap.read()
        #wide = int(self.cap.get(3))
        #height = int(self.cap.get(4))
        #convert frame to hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #create masks
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        #results after checking mask against frame
        
        #green_result = cv2.bitwise_and(frame, frame, mask=green_mask)
        #check if green exists
        if cv2.countNonZero(green_mask) > 0:
            self.get_logger().info("Green Found!")
            green_found = True
        else:
            self.get_logger().info('No green found')
        return green_found
        #self.find_green = False
        #Display Image
        #cv2.imshow("image mask", current_frame)

        #cv2.waitKey(1)

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
    with open(path_to_csv, 'r') as f:
        reader = csv.reader(f)
        for lines in reader:
            new_obj = Object(lines[0],int(lines[1]),float(lines[2]),lines[3])
            new_list.append(new_obj)
    return new_list

def write_csv(objects):
    with open(path_to_csv, 'w') as f:
        writer = csv.writer(f)
        for obj in objects:
            writer.writerow(obj.object_to_tuple())

def main():
    rclpy.init(args=None)
    my_node = WallAvoider()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
