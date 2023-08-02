import rclpy
from std_msgs.msg import String
from rclpy.node import Node

class Talker(Node):
    def __init__(self):
        super().__init__('talker_node')
        self.pub = self.create_publisher(String, 'topic', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Im talking')
        self.pub.publish()

def main():
    rclpy.init(args=None)
    my_node = Talker()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()