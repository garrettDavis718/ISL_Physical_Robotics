import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
import apriltag
import argparse

ap = argparse.ArgumentParser()

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__("apriltag_detector")
        self.img_publisher = self.create_publisher(Image, "raw_image", 10)
        self.found_apriltag = self.create_publisher(String, "apriltag_finder", 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()
        
    def timer_callback(self):
        apriltag_found = String()
        apriltag_found.data = "False"
        ret, frame = self.cap.read()
        width = int(self.cap.get(3))
        height = int(self.cap.get(4))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        options = apriltag.DetectorOptions(families="tag36_11_00018")
        detector = apriltag.Detector(options)
        results = detector.detect(gray)
        
        for r in results:
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            
            cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
            cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
            cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
            cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
            
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(frame, (cX, cY), 5, (0, 255, 0), -1)
            
            tagFamily = r.tag_family.decode("utf-8")
            cv2.putText(frame, tagFamily, (ptA[0], ptA[1] -15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
        self.img_publisher_.publish(self.br.cv2_to_imgmsg(frame))
            
def main():
    rclpy.init(args=None)
    image_publisher = AprilTagDetector()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()