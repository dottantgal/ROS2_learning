#  *
#  * @file my_first_node.py
#  *
#  * @brief A basic ROS2 node that spins and printouts a string
#  *        waiting for a killing request
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#  *

# import of rclpy library and modules to retrieve many of the ROS2 core functionalities: nodes, topics, services
import rclpy
from rclpy.node import Node

def main(args=None):
    # initialize ROS communications
    rclpy.init(args=args)
    # creation of the ROS2 node with at least one parameter (the node name for ex)
    node = Node('my_first_node')
    # use the get_logger method to printout indormation
    node.get_logger().info("ROS2 node first test")
    
    # Pause the program execution here, but any created thread/timer  
    # in the node will continue to be executed as well as the calling to any callback 
    # function defined for the node, allowing the node to communicate with other nodes.
    rclpy.spin(node)
    # kill the node, the spin function will exit, and any callback won’t be callable anymore.
    rclpy.shutdown()

if __name__ == '__main__':
    main()
