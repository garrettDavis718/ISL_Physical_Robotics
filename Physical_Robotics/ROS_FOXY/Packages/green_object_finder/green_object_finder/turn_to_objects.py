import csv #For reading and writing data to csv
import rclpy
from datetime import datetime #Used to create a timestamp
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import LaserScan # LaserScan is another subscriber
import time #Used to make precise turns 
from geometry_msgs.msg import Twist #/cmd_vel publisher
import cv2 # OpenCV library
from cv_bridge import CvBridge
import numpy as np
import sys #Used to kill the program
from sensor_msgs.msg import Image #image to publish image with green
from .submodules.object_class import Object #Object class that hold the data for each object in the tb's range
from .submodules.movement import MovementCommands as mc  #Movement to turn to each object
from std_msgs.msg import String #string for whether green found


qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

#Path to read the csv output by closest_objects.py
path_to_csv = '/home/ubuntu/ros2_ws/src/green_object_finder/green_object_finder/nearby_objects.csv'

#The lower and upper bound values for the color green
lower_green = np.array([48,63,63])
upper_green = np.array([98,255,255])

time_stamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")  #Time stamp to accurately keep track of the csv's output by this script

#The path to the usb that this script will write the updated csv and save our photo of the green object
output_csv_path = '/home/ubuntu/ros2_ws/src/green_object_finder/green_object_finder/nearby_objects.csv'
path_to_photo = f'/media/external/nearest_image{time_stamp}.png'

class WallAvoider(Node):
    def __init__(self):
        self.object_list = read_csv()  #Read from the csv outputted by closest_objects.py to create a list of objects
        self.current_degree = 0  #The current degree that the tb is facing
        self.found_objects = []  #List of objects after being checked for green

        super().__init__("wall_avoider_node")  #Super init

        self.sub = self.create_subscription(LaserScan, "/scan",
         self.suscriber_callback, qos_policy)  #init sub attribute for Lidar Scan
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)  #init velocity pub
        self.br = CvBridge()
        self.green_found = False  #Attribute to hold whether or not a object is green
        self.closest_object = Object(0,0,0.0,False,0) #Placeholder attribute to save the closest green object
    
    def suscriber_callback(self, msg: LaserScan, move_cmd = Twist()):

        for obj in self.object_list:  #Iterate ove the list of objects, turn to each one and determine if they are green.

            turn_loc = obj.location - self.current_degree  #Calculate the position of the object we currently want to check
            mc.turn_left(self)
            time.sleep(turn_loc/obj.lidar_angles*32.5)
            self.pub.publish(Twist())  #Halt turning

            self.current_degree = obj.location  #Update the degree at which the tb is currently facing
            obj.is_green = self.green_finder()  #Call green_finder function to determine if the object is green

            if obj.is_green:

                if self.closest_object.distance == 0.0 :  #If self.closest_object hasn't been updated, do so.
                    self.closest_object = obj

                elif obj.distance <= self.closest_object.distance:  #If current green object is closer, update self.closest_object
                    self.closest_object = obj

            self.found_objects.append(obj) #Append to the final list of objects

        print('all objects checked')
        
        if self.closest_object.is_green: #Turns back to the closest green object

            mc.turn_right(self) 
            time.sleep((self.current_degree - self.closest_object.location)/self.closest_object.lidar_angles*32.5)
            
            print("Nearest Object in front")
            self.pub.publish(Twist())
            self.take_photo()  #Take photo of the closest green object
            write_csv(self.closest_object)  #writing the updated objects to a csv
            sys.exit()
            
        else:  #No green objects were found
            
            print('No green objects.')
            sys.exit()



    def green_finder(self):
        """This function looks at truncated image returned by the turtlebot's camera and determines
        if the object it is looking at is green or not.

        Returns:
            bool: Returns a True or False value depending on whether or not green was detected.
        """
        green_found = False  #Placeholder variable to update the status of green
        cap = cv2.VideoCapture(0)

        ret, frame = cap.read()
        frame = frame[0:-1, 213:427]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  #convert frame to hsv
        green_mask = cv2.inRange(hsv, lower_green, upper_green)  #create masks

        if cv2.countNonZero(green_mask) > 0:  #check if green exists
            self.get_logger().info("Green Found!")
            green_found = True

        else:  #report if no green was found
            self.get_logger().info('No green found')

        return green_found



    def take_photo(self):
        """This function is called once the turtlebot has found the closest green object. It will take
        a photo and save it as a png named 'nearest_image' followed by a time stamp.
        """
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  #convert frame to hsv
        green_mask = cv2.inRange(hsv, lower_green, upper_green)  #create masks

        if ret:
            contours, _ = cv2.findContours (green_mask , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)

            #frame = frame[185:-1, 213:427]
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

def write_csv(object):
    """This function will write a new csv with updated values for the is_green attribute of our objects.

    Args:
        objects (list): objects is a list of objects of the object class.

    Output:
        The output is a csv containing the id, center index, distance from the turtlebot, and is_green status
        of each of our objects.
    """
    with open(output_csv_path, 'w') as f:
        writer = csv.writer(f)
        writer.writerow(object.object_to_tuple())

def main():
    rclpy.init(args=None)
    my_node = WallAvoider()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
