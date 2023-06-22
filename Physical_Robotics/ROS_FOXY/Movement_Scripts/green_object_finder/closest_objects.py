import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import LaserScan # LaserScan is another subscriber
from geometry_msgs.msg import Twist # Twist data to move robot
import cv2 # OpenCV library
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String #string for whether green found
from sensor_msgs.msg import Image #image to publish image with green
import time
import sys
from object_class import Object
import csv 


qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
path_to_csv = '/home/ubuntu/ISL_Physical_Robotics/Physical_Robotics/ROS_FOXY/Movement_Scripts/green_object_finder/nearby_objects.csv'

    
class GreenObjectFinder(Node):
    def __init__(self):
        self.closest_objects = []
        super().__init__('green_finder_node')
        #init sub attribute for Lidar Scan
        self.sub = self.create_subscription(LaserScan, "/scan",
         self.subscriber_callback, qos_policy)
        #init velocity pub
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.counter = 1
        self.max = 1.5
        self.len_of_ranges = 250

    def get_unique_objects(self):
        """_summary_:This function parses through the self.closest_objects list of tuples and sorts them into
        a list of lists. The elements of the unique_objects list are a list of tuples grouped by the likelihood that the slice
        belongs to the same object.

        Returns:
            _type_: list
        """        ''''''

        new_lst = [x for x in sorted(self.closest_objects)]

        last_object = None
        similar_objects = []
        unique_objects = []

        for item in new_lst:
            if last_object == None:
                similar_objects.append(item)
                last_object = item
            elif abs(item[0] - last_object[0]) <= 9 and abs(item[1] - last_object[1]) <= 9:
                similar_objects.append(item)
                last_object = item
            elif abs(item[0] - last_object[0]) > 9 and abs(item[1] - last_object[1]) > 9:
                    unique_objects.append(similar_objects[:])
                    similar_objects.clear()
                    similar_objects.append(item)
                    last_object = item
            else:
                similar_objects.clear()
                similar_objects.append(item)
                last_object = item
        unique_objects.append(similar_objects)
        print(unique_objects)
        return unique_objects
    
    def subscriber_callback(self, msg: LaserScan, move_cmd = Twist()):
        self.len_of_ranges = len(msg.ranges)
        lidar_vals = msg.ranges[0:len(msg.ranges)]
        object = []

        def get_distance(loc):
            if not np.isnan(msg.ranges[loc]):
                return msg.ranges[loc]
            else:
                get_distance(loc+1)
        
        def avg_distance(value_1, value_2):
            total = 0
            for x in msg.ranges[value_1: value_2]:
                total += x
                return total/len(msg.ranges[value_1:value_2])
            
        

        def detect_next_nan(idx):
            """_summary_: This function is a recursive call to find the next valid index that is not NaN.

            Args:
                idx (_type_): int

            Returns:
                _type_: bool, int
            """            ''''''

            if np.isnan(lidar_vals[idx]):
                return detect_next_nan(idx+1)
            else:
                return lidar_vals[idx] > 1.0, idx
            
        def detect_last_nan(idx):
            """_summary_: This function is a recursive call to find the last valid index that is not NaN.

            Args:
                idx (_type_): int

            Returns:
                _type_: bool, int
            """            

            if np.isnan(lidar_vals[idx]):
                return detect_last_nan(idx-1)
            else:
                return lidar_vals[idx] > 1.0, idx

        for idx in range(len(lidar_vals)):

            next_idx = idx + 1
            previous_idx = idx - 1

        
            if lidar_vals[idx] < self.max:


                if idx == 0 or idx == (len(lidar_vals)-1):

                    object.append(idx)

                else:
                    is_next_nan = np.isnan(lidar_vals[next_idx])
                    is_previous_nan = np.isnan(lidar_vals[previous_idx])

                    if (lidar_vals[previous_idx] > self.max or lidar_vals[next_idx] > self.max) or is_next_nan or is_previous_nan:
                    
                        if is_next_nan:
                            next_idx_bool, next_valid_idx = detect_next_nan(next_idx)

                            if next_idx_bool:
                                object.append(next_valid_idx)

                            if lidar_vals[next_idx] > 1.0:
                                object.append(idx)

                        elif is_previous_nan:
                            previous_idx_bool, previous_valid_idx = detect_last_nan(previous_idx)

                            if previous_idx_bool:
                                object.append(previous_valid_idx)

                            if lidar_vals[previous_idx] > 1.0:
                                object.append(idx)

                        else:
                            object.append(idx)

                if not (object in self.closest_objects) and (len(object) > 1 and len(object) < 3):

                    self.closest_objects.append(object[:])
                    object.clear()
                    

        def write_objects(final_list):
            counter = 1
            writing_objects_list = []
            temp_objects = []
            #sorted_list = []
            #min = 0 
            if abs(avg_distance(final_list[0][0],final_list[0][1])- avg_distance(final_list[-1][0],final_list[-1][1]))< 0.03:
                print(abs(avg_distance(final_list[0][0],final_list[0][1]) - avg_distance(final_list[-1][0],final_list[-1][1])))
                if final_list[0][0] <= 5 and final_list[-1][1] >= self.len_of_ranges - 5:
                    final_list.pop(-1)
                    final_list[0] = [0,1]
                    
            
            for x in final_list:
                center = int((x[0]+x[1])/2)
                new_obj = Object(counter, center, avg_distance(x[0],x[1]), False)
                writing_objects_list.append(new_obj)
                print(new_obj.object_to_tuple())
                counter+=1
            '''#bubble sort
            for x in writing_objects_list:
                for i in range(len(writing_objects_list)-1):
                    if writing_objects_list[i].distance > writing_objects_list[i+1].distance:
                        writing_objects_list[i], writing_objects_list[i+1] = writing_objects_list[i+1], writing_objects_list[i]'''
            with open(path_to_csv, 'w') as f:
                writer = csv.writer(f)
                for obj in writing_objects_list:
                    writer.writerow(obj.object_to_tuple())
                    print(f'Objects found: {len(writing_objects_list)}\nObject data written to file.')
            


        if self.counter == 1:

            final_objects = [min(x) for x in self.get_unique_objects()]  
            write_objects(final_objects) 
            sys.exit()

          
        else:
            self.counter += 1

def main():
    rclpy.init(args=None)
    my_node = GreenObjectFinder()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()