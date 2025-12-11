#  *
#  * @file client_node_class.py
#  *
#  * @brief A basic ROS2 service client node with class implementation that asks the user 
#  *        to input two strings and gets back a capitalized full string from the server service.
#  *        It's necessary to use the custom message defined in the external 
#  *        package "custom_msg_and_srv" (C++ package)
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#

import rclpy
from rclpy.node import Node
from custom_msg_and_srv.srv import CapitalFullName


class MyClientNode(Node):
    """Service client node implemented as a class."""

    def __init__(self, passed_node_name="VOID", passed_name="VOID", passed_surname="VOID"):
        super().__init__(passed_node_name)
        self.name_ = passed_name
        self.surname_ = passed_surname
        
        # Create the service client
        self.client_ = self.create_client(CapitalFullName, 'create_cap_full_name')
        
        # Call the server response method
        self.server_response()
    
    def server_response(self):
        """Handle the server response."""
        # Create the request
        request = CapitalFullName.Request()
        request.name = self.name_
        request.surname = self.surname_
        
        # Wait for the service to be available
        while not self.client_.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error(
                    'Interrupted while waiting for the service. Exiting.'
                )
                return
            self.get_logger().info('SERVICE NOT AVAILABLE, waiting again...')
        
        # Send the request asynchronously
        future = self.client_.call_async(request)
        
        # Spin until the future is complete
        # Note: In Python, we can use the node directly with spin_until_future_complete
        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            if rclpy.ok():
                self.get_logger().info("Interrupted while waiting for service response...")
            return
        
        if future.result() is not None:
            self.get_logger().info(
                f'Capitalized full name: {future.result().capitalfullname}'
            )
        else:
            self.get_logger().error(
                'Failed to call service create_cap_full_name'
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        # Get user input
        name = input("Insert the name -> ")
        surname = input("Insert the surname -> ")
        
        # Create the client node
        node = MyClientNode("client_node", name, surname)
    except KeyboardInterrupt:
        if node is not None and rclpy.ok():
            node.get_logger().info("Interrupted while waiting for service...")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

