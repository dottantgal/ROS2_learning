#  *
#  * @file node_timer_without_class.py
#  *
#  * @brief A basic ROS2 node that spins and printouts a string
#  *        based on a timer. Not good practice, just for instance
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#  *

import rclpy
from rclpy.node import Node

global node

def timer_callback():
    node.get_logger().info("Echo line from a ROS2 timer callback")

def main(args=None):
    global node
    rclpy.init(args=args)
    node = Node('my_first_node_timer')
    timer = node.create_timer(0.2, timer_callback)
    rclpy.spin(node)
    node.destroy_timer(timer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()