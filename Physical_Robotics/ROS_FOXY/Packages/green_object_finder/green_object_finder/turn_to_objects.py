import csv 
import rclpy
from datetime import datetime
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import LaserScan # LaserScan is another subscriber
import time
from geometry_msgs.msg import Twist
import cv2 # OpenCV library
from cv_bridge import CvBridge
import numpy as np
import sys
from sensor_msgs.msg import Image #image to publish image with green
from .submodules.object_class import Object
from std_msgs.msg import String #string for whether green found


qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
path_to_csv = '/home/ubuntu/ros2_ws/src/green_object_finder/green_object_finder/nearby_objects.csv'
lower_green = np.array([48,63,63])
upper_green = np.array([98,255,255])
time_stamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
output_csv_path = f'/media/external/nearby_csv{time_stamp}.csv'
path_to_photo = f'/media/external/nearest_image{time_stamp}.png'
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
        self.br = CvBridge()
        self.green_found = False
        self.closest_object = Object(0,0,0.0,False,0)
    
    def suscriber_callback(self, msg: LaserScan, move_cmd = Twist()):
        for obj in self.object_list:
            turn_loc = obj.location - self.current_degree
            self.turn_left()
            time.sleep(turn_loc/obj.lidar_angles*32.5)
            self.current_degree = obj.location
            self.pub.publish(Twist())
            obj.is_green = self.green_finder()
            if obj.is_green:
                if self.closest_object.distance == 0.0 :
                    self.closest_object = obj
                elif obj.distance <= self.closest_object.distance:
                    self.closest_object = obj
            self.found_objects.append(obj)
        print('all objects checked')
        
        try:
            self.turn_right()
            time.sleep((self.current_degree - self.closest_object.location)/self.closest_object.lidar_angles*32.5)
        except ZeroDivisionError:
            print('No Green Objects.')
            sys.exit()
            
        print("Nearest Object in front")
        self.pub.publish(Twist())
        self.take_photo()
        write_csv(self.found_objects)
        sys.exit()



    def green_finder(self):
        """This function looks at truncated image returned by the turtlebot's camera and determines
        if the object it is looking at is green or not.

        Returns:
            bool: Returns a True or False value depending on whether or not green was detected.
        """
        green_found = False
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        ret, frame = cap.read()
        #convert frame to hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #create masks
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        #check if green exists
        if cv2.countNonZero(green_mask) > 0:
            self.get_logger().info("Green Found!")
            green_found = True
        else:
            self.get_logger().info('No green found')
        return green_found

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

    def take_photo(self):
        """This function is called once the turtlebot has found the closest green object. It will take
        a photo and save it as a png named 'nearest_image' followed by a time stamp.
        """
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        if ret:
            contours, _ = cv2.findContours (frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
            cv2.imwrite(path_to_photo, frame)
            print("Photo Taken")


def read_csv():
    """This function will read the csv generated by closest_objects.py and add them to a list to be
    iterated over.

    Returns:
        list: Returns a list of objects of the object class.
    """
    new_list = []
    with open(path_to_csv, 'r') as f:
        reader = csv.reader(f)
        for lines in reader:
            new_obj = Object(lines[0],int(lines[1]),float(lines[2]),lines[3], int(lines[4]))
            new_list.append(new_obj)
    return new_list

def write_csv(objects):
    """This function will write a new csv with updated values for the is_green attribute of our objects.

    Args:
        objects (list): objects is a list of objects of the object class.

    Output:
        The output is a csv containing the id, center index, distance from the turtlebot, and is_green status
        of each of our objects.
    """
    with open(output_csv_path, 'w') as f:
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
