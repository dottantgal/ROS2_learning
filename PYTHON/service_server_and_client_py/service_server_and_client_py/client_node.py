#  *
#  * @file client_node.py
#  *
#  * @brief A basic ROS2 service client node that asks the user to input two strings
#  *        and gets back a capitalized full string from the server service.
#  *        It's necessary to use the custom message defined in the external
#  *        package "custom_msg_and_srv_py" 
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#

import rclpy
from rclpy.node import Node
from custom_msg_and_srv_py.srv import CapitalFullName


def main(args=None):
    rclpy.init(args=args)
    
    # The user inserts name and surname
    name = input("Insert the name -> ")
    surname = input("Insert the surname -> ")
    
    # Declaration of the client node
    node = Node('client_node')
    
    # Declaration of the service client of type "create_cap_full_name"
    # using the custom srv "CapitalFullName"
    client = node.create_client(CapitalFullName, 'create_cap_full_name')
    
    # Create the request
    request = CapitalFullName.Request()
    request.name = name
    request.surname = surname
    
    # Check if the server is online and everything goes well otherwise printout
    while not client.wait_for_service(timeout_sec=1.0):
        if not rclpy.ok():
            node.get_logger().error('Interrupted while waiting for the service. Exiting.')
            return
        node.get_logger().info('SERVICE NOT AVAILABLE, waiting again...')
    
    # Send the request asynchronously
    future = client.call_async(request)
    
    # Spin waiting for the SUCCESS result
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info(
            f'Capitalized full name: {future.result().capitalfullname}'
        )
    else:
        node.get_logger().error('Failed to call service create_cap_full_name')
    
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()

