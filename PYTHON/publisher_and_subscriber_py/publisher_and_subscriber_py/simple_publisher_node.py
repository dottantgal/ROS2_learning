#  *
#  * @file simple_publisher_node.py
#  *
#  * @brief A basic publisher node
#  *        Example purpose, useful to look how to use rate and exceptions
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#  *


import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)
    node = Node('simple_publisher_node')
    msg = String()
    publisher = node.create_publisher(String, 'topic', 10)
    counter = 0
    rate = rclpy.create_rate(2, node.get_clock())

    while rclpy.ok():
        msg.data = "Hello, world! " + str(counter)
        node.get_logger().info("Publishing -> " + msg.data)
        try:
            publisher.publish(msg)
            counter = counter + 1
            rclpy.spin_once(node)
        except Exception as e:
            node.get_logger().info("Error type: " + str(e))
        rate.sleep()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
