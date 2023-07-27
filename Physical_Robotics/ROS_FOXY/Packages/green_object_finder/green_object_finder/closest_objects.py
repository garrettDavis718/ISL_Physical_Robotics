import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import LaserScan # LaserScan is another subscriber
import numpy as np
import matplotlib.pyplot as plt #Making diagram of lidar 
import sys
from .submodules.object_class import Object
import csv 

qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
path_to_csv = '/home/ubuntu/ros2_ws/src/green_object_finder/green_object_finder/nearby_objects.csv'
path_to_lidar_diagram = '/media/external/lidar_diagram.png'
    
class GreenObjectFinder(Node):
    def __init__(self):
        self.closest_objects = []
        super().__init__('green_finder_node')
        #init sub attribute for Lidar Scan
        self.sub = self.create_subscription(LaserScan, "/scan",
        self.subscriber_callback, qos_policy)
        self.counter = 1
        self.max = 1.5
        self.lidar_vals = None
        self.len_of_lidar = 0

    def subscriber_callback(self, msg: LaserScan):
        """Callback function that operates on /scan topic with LASERSCAN data type
        will find all nearby objects' properties and write them to a csv

    Args:
        self : node
        msg : LASERSCAN data
    
    Output:
        The output is a csv containing the id, center index, distance from the turtlebot, and is_green status
        of each of our objects. (using write_objects function)
    """
        self.len_of_lidar = len(msg.ranges)
        #Create a list of our lidar distances that does not contain NaNs
        self.lidar_vals = [x for x in msg.ranges if not np.isnan(x)] 
        #init temp_obj
        object = []
        #walk through lidar values, finding object edges within self.max
        for idx in range(len(self.lidar_vals)-1):
            #check next/prev indx
            next_idx = idx + 1
            previous_idx = idx - 1
            if self.lidar_vals[idx] < self.max:
                if idx == 0 or idx == (len(self.lidar_vals)-1):
                    object.append(idx)
                else:
                    if self.lidar_vals[next_idx] >= self.max:
                            object.append(idx)

                    elif self.lidar_vals[previous_idx] >= self.max:
                            object.append(idx)
                #check if object found previously, then write to self.closest_objects list - clear temp_obj
                if not (object in self.closest_objects) and (len(object) > 1 and len(object) < 3):
                    self.closest_objects.append(object[:])
                    object.clear()
        #if objects found, write
        if self.closest_objects:
            write_objects(self.closest_objects, self.lidar_vals) 
        #no objects, no write
        else:
            print('no objects')
        #exit node
        sys.exit()


    
def write_objects(final_list, lidar_scan_no_nan):
    """This function takes in a list of unique objects within the turtlebot's declared range
    and instantiates each item of the list as an object of the object class. Each object is appended to 
    a list that will be iterated over in order to create a csv.

    Args:
        final_list (list): This list contains the slices that reference each unique object within the turtlebot's
        declared range.
    
    Output:
        The output is a csv containing the id, center index, distance from the turtlebot, and is_green status
        of each of our objects.
    """
    #id for each object
    counter = 1
    writing_objects_list = []
    #check that front values are consisten distance to ensure only one object 'in front'
    #maybe recheck logic here to get better center of object 'in front' 
    if abs(avg_distance(final_list[0][0],final_list[0][1], lidar_scan_no_nan)- avg_distance(final_list[-1][0],final_list[-1][1], lidar_scan_no_nan))< 0.03:
        if final_list[0][0] <= 5 and final_list[-1][1] >= len(lidar_scan_no_nan) - 5:
            final_list.pop(-1)
            final_list[0] = [0,1]
    #check each inx in final temp list and instant objects -> write objects
    for x in final_list:
        #calc center
        center = int((x[0]+x[1])/2)
        #instant object
        new_obj = Object(counter, center, avg_distance(x[0],x[1], lidar_scan_no_nan), False, len(lidar_scan_no_nan))
        writing_objects_list.append(new_obj)
        #print object for sanity
        print(new_obj.object_to_tuple())
        counter+=1

    #write out lidar scan diagram
    lidar_diagram(lidar_list=lidar_scan_no_nan, obj_lst=writing_objects_list[:])


    #write out writing_objects_list of objects
    with open(path_to_csv, 'w') as f:
        writer = csv.writer(f)
        for obj in writing_objects_list:
            writer.writerow(obj.object_to_tuple())
            print(f'Objects found: {len(writing_objects_list)}\nObject data written to file.')

#edit this function to better visualize lidar scan
def lidar_diagram(lidar_list, obj_lst):
    """This function ouputs a diagram of the most recent msg.ranges scan and places objects at their location.

    Args:
        lidar_list (array): This function takes in msg.ranges or any variable holding the array of msg.ranges as an argument.

    Output:
        The output is a circular diagram of the lidar scan saved as a png.
    """
    lidar_angles = np.array([1 for x in range(0,len(lidar_list)+1,5)])
    my_labels = []
    for x in range(0, len(lidar_list)+1,5):
        if obj_lst:
            if (obj_lst[0].location == x) or (obj_lst[0].location > x and obj_lst[0].location < x+5):
                my_labels.append(f'Object {obj_lst[0].id}')
                obj_lst.pop(0)
            else:
                my_labels.append(x)
        else:
            my_labels.append(x)
    plt.pie(lidar_angles, labels=my_labels,startangle=90, radius=1.4)
    plt.savefig(path_to_lidar_diagram)
            
def avg_distance(value_1, value_2, lidar_scan_no_nan):
    """This function determines the average distance of the values within an objects slice.

    Args:
        value_1 (int): The beginning index of the slice that references an object of interest.
        value_2 (int): The ending index of the slice that references an object of interest.

    Returns:
        float: Returns a float that will later be rounded once its passed as an attribute to the object class.
    """
    counter = 0
    total = 0
    #check each distance in slice, average them
    for x in lidar_scan_no_nan[value_1: value_2]:    
        total += x
        counter += 1
    return total/counter
       
def main():
    rclpy.init(args=None)
    my_node = GreenObjectFinder()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()    
