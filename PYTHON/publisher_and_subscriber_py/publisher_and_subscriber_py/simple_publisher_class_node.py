#  *
#  * @file simple_publisher_class_node.py
#  *
#  * @brief A basic publisher class node
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#  *

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('simple_publisher_class_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.msg_ = String()
        self.counter_ = 0
        self.timer_ = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info("ROS2 class node first test")
    
    def timer_callback(self):
        self.msg_.data = "Hello, world! " + str(self.counter_)
        self.get_logger().info("Publishing -> " + self.msg_.data)
        self.publisher_.publish(self.msg_)
        self.counter_ = self.counter_ + 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()