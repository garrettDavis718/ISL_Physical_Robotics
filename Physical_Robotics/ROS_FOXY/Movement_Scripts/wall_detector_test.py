#simple detection

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Vector3
import time

#Setup our Node


qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
class WallAvoider(Node):
    def __init__(self, last_turn=None):
    #last_turn attribute keeps track of the last turn that was made('Right','Left', or None)
        self.last_turn = None 
    #Super init
        super().__init__("wall_avoider_node")
        #init sub attribute for Lidar Scan
        self.sub = self.create_subscription(LaserScan, "/scan",
         self.subscriber_callback, qos_policy)
        #init velocity pub
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
 
    #Callback Method for lidar scan data		
    def subscriber_callback(self, msg : LaserScan, move_cmd = Twist()):

        """Callback function to detect objects in front of the turtlebot and avoid them.

        msg.ranges returns an array of values representing the distance of an object from the robot.
        
        We slice the array returned from msg.ranges to grab the distances directly in front of the robot.

        If closest object is on the left side and if our last_turn was 'Right' or None, we turn right 
        and update self.last_turn to 'Right'.

        If closest object is on the right side and if our last_turn was 'Left' or None, we turn left 
        and update self.last_turn to 'Left'.

        If no obstacles, we proceed forward and update self.last_turn to None.

        time.sleep() prevents the next line of code from running, the time is specified by the value passed as 
        an argument. We use time.sleep to control the size of the turning radius.


        GAZEBO VALUES FOR msg.ranges: 0, 360

        REAL TIME VALUES FOR msg.ranges: 0, 250

        Lidar returns values couter-clockwise

        Args:
            msg (LaserScan): _description_
            move_cmd (_type_, optional): _description_. Defaults to Twist().
        """

        front_right_val= msg.ranges[220:250]
        front_left_val= msg.ranges[0:30]
        front_vals = list([front_left_val, front_right_val])
        #display distances
        print(f'Front Left values: {front_left_val}')
        print(f'Front Right values: {front_right_val}')


        if min(front_left_val) < 0.8 and min(front_left_val) < min(front_right_val):
            if self.last_turn == 'Right' or self.last_turn == None:
                self.turn_right()
                time.sleep(3)
                self.last_turn = 'Right'
                print(self.last_turn)

        elif min(front_right_val) < 0.8:
            if self.last_turn == 'Left' or self.last_turn == None:
                self.turn_left()
                time.sleep(3)
                self.last_turn = 'Left'
                print(self.last_turn)

        else:
            self.move_forward()
            self.last_turn = None
            print(self.last_turn)




    """Functions to execute the directons of movement.

       Turtlebot motor cannot handle speeds >= 0.3
       
       Positive angular.z values == left turn
       
       Negative angular.z values == right turn 

       Positive.linear.x values == forward
       
       Negative linear.x values == backward

       After updating the angular.z or linear.x we publish the move_cmd.


        Args:
            move_cmd (_type_, optional): _description_. Defaults to Twist().
    """
    
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