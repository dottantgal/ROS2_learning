#  *
#  * @file node_with_class.py
#  *
#  * @brief A basic ROS2 node implemented with class that spins
#  *        and printouts a string waiting for a killing request
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#  *

import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node_class')
        self.get_logger().info("ROS2 class node first test")

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()