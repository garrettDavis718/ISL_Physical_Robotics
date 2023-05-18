import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String #string for whether green found
from sensor_msgs.msg import Image #image to publish image with green

#color values
#upper/lower hsv values of red&green
lower_red = np.array([164, 100, 100])
upper_red = np.array([184,255,255])
lower_green = np.array([68,100,100])
upper_green = np.array([88,255,255])

class GreenPublisher(Node):
    def __init__(self):
        super().__init__('green_finder_pub')
        self.green_publisher_ = self.create_publisher(String, 'green_finder', 10)
        self.green_img_publisher_ = self.create_publisher(Image, 'image_mask', 10)
        self.img_publisher_ = self.create_publisher(Image, 'raw_image', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

    def timer_callback(self):
        green_found = String()
        green_found.data = "False"
        ret, frame = self.cap.read()
        wide = int(self.cap.get(3))
        height = int(self.cap.get(4))

        #convert frame to hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #create masks
        red_mask = cv2.inRange(hsv, lower_red, upper_red)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        #results after checking mask against frame
        red_result = cv2.bitwise_and(frame, frame, mask=red_mask)
        green_result = cv2.bitwise_and(frame, frame, mask=green_mask)
        #check if green exists
        #only pubishing color mask if color is found
        if cv2.countNonZero(green_mask) > 0:#if green is found
            green_found.data = "True"
            self.get_logger().info("Green Found!")
            self.green_img_publisher_.publish(self.br.cv2_to_imgmsg(green_result))
            self.green_publisher_.publish(green_found)
        else:
            self.get_logger().info('No green found')
            self.green_publisher_.publish(green_found)
        #always publishing the raw image
        self.img_publisher_.publish(self.br.cv2_to_imgmsg(frame))
            

def main():
    rclpy.init(args=None)
    image_publisher = GreenPublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
