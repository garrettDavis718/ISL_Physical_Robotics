import rclpy
from std_msgs.msg import String
from rclpy.node import Node

class Listener(Node):
    def __init__(self):
        super().__init__('listener_node')
        self.sub = self.create_subscription(String, 'topic', self.subcriber_callback, 10)

    def subscriber_callback(self, msg: String):
        print('Im listening')

def main():
    rclpy.init(args=None)
    my_node = Listener()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
