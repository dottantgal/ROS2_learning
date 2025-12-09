#  *
#  * @file service_node_class.py
#  *
#  * @brief A basic ROS2 service server node with class implementation that gets two 
#  *        strings as request and answer with a capitalized full string as response.
#  *        It's necessary to use the custom message defined in the external
#  *        package "custom_msg_and_srv_py"
#  *        To call the service from a terminal use on a single line:
#  *        ros2 service call /create_cap_full_name 
#  *        custom_msg_and_srv_py/srv/CapitalFullName "{name: x, surname: y}"
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#

import rclpy
from rclpy.node import Node
from custom_msg_and_srv_py.srv import CapitalFullName


class MyServiceNode(Node):
    """Service server node implemented as a class."""

    def __init__(self, passed_node_name="VOID"):
        super().__init__(passed_node_name)
        self.get_logger().info('I am ready to capitalize your full name')
        
        # Create the service with the callback method
        self.service_ = self.create_service(
            CapitalFullName,
            'create_cap_full_name',
            self.compose_full_name
        )
    
    def compose_full_name(self, request, response):
        """
        Method to handle the client request and give back a response.
        The service gets the name and surname and responses with a capitalized full name.
        """
        full_name = request.name + " " + request.surname
        capital_full_name = full_name.upper()  # Capitalize using Python's built-in method
        response.capitalfullname = capital_full_name
        
        self.get_logger().info(
            f'Incoming name: {request.name}\tIncoming surname: {request.surname}'
        )
        self.get_logger().info(
            f'Sending back the capitalize full name: [{response.capitalfullname}]'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MyServiceNode("service_node")
    rclpy.spin(node)  # The service starts to wait and manage requests
    rclpy.shutdown()


if __name__ == '__main__':
    main()

