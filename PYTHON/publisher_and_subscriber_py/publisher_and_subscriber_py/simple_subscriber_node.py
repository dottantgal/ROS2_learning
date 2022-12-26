#  *
#  * @file simple_subscriber_node.py
#  *
#  * @brief A basic subscriber node
#  *        Not recommend this style, example purpose
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#  *

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

global node

def topic_callback(msg):
    node.get_logger().info("I heard the message : " + str(msg))

def main(args=None):
    global node
    rclpy.init(args=args)
    node = Node('simple_subscriber_node')
    node.subscription = node.create_subscription(
          String,
          'topic',
          topic_callback,
          10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()