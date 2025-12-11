#  *
#  * @file simple_subscriber_class_node.py
#  *
#  * @brief A basic subscriber class node
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#  *

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('simple_subscriber_class_node')
        self.subscriber_ = self.create_subscription(String, 'topic', self.topic_callback, 10)
    
    def topic_callback(self, msg):
        self.get_logger().info("I heard the message : " + str(msg.data))

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
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