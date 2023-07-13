import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys
import cv2 # OpenCV library
from cv_bridge import CvBridge
from datetime import datetime

turtlebot_name_1 = 'R2TB_O1'
turtlebot_name_2 = 'R2TB_O2'
time_stamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
path_to_photo = f'/media/external/nearest_image{time_stamp}.png'

class DomainMergeSub(Node):
    
    def __init__(self):
        self.current_distance = 1.5
        self.closest_turtlebot = None
        self.counter = 1
        super().__init__('domain merger')
        self.sub_1 = self.create_subscription(Float32, turtlebot_name_1, self.subscriber_callback_1, 10)
        self.sub_2 = self.create_subscription(Float32, turtlebot_name_2, self.subscriber_callback_2, 10)

    def subscriber_callback_1(self, msg: Float32):
        if msg <= self.current_distance:
            self.current_distance = msg
            self.closest_turtlebot = turtlebot_name_1

    def subscriber_callback_2(self, msg: Float32):
        if msg <= self.current_distance:
            self.current_distance = msg
            self.closest_turtlebot = turtlebot_name_2

        if self.counter == 1:
            sys.exit()
        else:
            self.counter += 1

    def take_photo(self):
        """This function is called once the turtlebot has found the closest green object. It will take
        a photo and save it as a png named 'nearest_image' followed by a time stamp.
        """
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        if ret:
            cv2.imwrite(path_to_photo, frame)
            print("Photo Taken")


    
def main():
    rclpy.init(args=None)
    my_node = DomainMergeSub()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()