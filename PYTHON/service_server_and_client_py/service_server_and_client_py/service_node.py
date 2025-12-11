#  *
#  * @file service_node.py
#  *
#  * @brief A basic ROS2 service server node that gets two strings as request
#  *        and answer with a capitalized full string as response.
#  *        It's necessary to use the custom message defined in the external
#  *        package "custom_msg_and_srv" (C++ package)
#  *        To call the service from a terminal use on a single line:
#  *        ros2 service call /create_cap_full_name 
#  *        custom_msg_and_srv/srv/CapitalFullName "{name: x, surname: y}"
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#

import rclpy
from rclpy.node import Node
from custom_msg_and_srv.srv import CapitalFullName

# Global logger for the callback function
_logger = None


def compose_full_name(request, response):
    """
    Method to handle the client request and give back a response.
    The service gets the name and surname and responses with a capitalized full name.
    """
    global _logger
    full_name = request.name + " " + request.surname  # Creation of the full name
    capital_full_name = full_name.upper()  # Capitalize the full name using Python's built-in method
    response.capitalfullname = capital_full_name
    
    _logger.info(
        f'Incoming name: {request.name}\tIncoming surname: {request.surname}'
    )
    _logger.info(
        f'Sending back the capitalize full name: [{response.capitalfullname}]'
    )
    return response


def main(args=None):
    global _logger
    rclpy.init(args=args)
    node = Node('service_node')  # Creation of the node
    _logger = node.get_logger()  # Set the global logger
    
    # Declaration of the service called "create_cap_full_name", it waits a request from the client
    # that will be handled by the compose_full_name function
    service = node.create_service(
        CapitalFullName, 
        'create_cap_full_name', 
        compose_full_name
    )
    node.get_logger().info('I am ready to capitalize your full name')
    try:
        rclpy.spin(node)  # The service starts to wait and manage requests
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Shutting down service node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

