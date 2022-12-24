#  *
#  * @file node_timer_with_class.py
#  *
#  * @brief A basic ROS2 node implemented with class that spins
#  *        and printouts a string based on a timer
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#  *


import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node_timer_class')
        self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Echo line from a ROS2 class timer callback ")

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()