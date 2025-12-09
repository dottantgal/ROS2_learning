#  *
#  * @file parameters_node.py
#  *
#  * @brief A node to declare and get parameters
#  *        Here the parameters are the message data for two publisher
#  *        It's possible to change them at run time using the command line
#  *        "ros2 param set /set_parameter_node vehicle_speed 100"
#  *        "ros2 param set /set_parameter_node vehicle_type car"
#  *        or using a launch file
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16


class MySetParameterClass(Node):
    """Node that demonstrates parameter declaration and retrieval."""

    def __init__(self):
        super().__init__('set_parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('vehicle_type', 'bike')
        self.declare_parameter('vehicle_speed', 10)
        
        # Create publishers
        self.pub_string_ = self.create_publisher(String, '/vehicle_type', 10)
        self.pub_int_ = self.create_publisher(Int16, '/vehicle_speed', 10)
        
        # Create timer to periodically publish parameter values
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        """Timer callback that reads parameters and publishes them."""
        # Get parameter values (they can be changed at runtime via command line or launch file)
        vehicle_type_param = self.get_parameter('vehicle_type').value
        vehicle_speed_param = self.get_parameter('vehicle_speed').value
        
        # Create and publish messages
        message_string = String()
        message_string.data = vehicle_type_param
        
        message_int = Int16()
        message_int.data = vehicle_speed_param
        
        self.get_logger().info(
            f'Publishing two messages -> vehicle type {vehicle_type_param}\tVehicle speed {vehicle_speed_param}'
        )
        
        self.pub_int_.publish(message_int)
        self.pub_string_.publish(message_string)


def main(args=None):
    rclpy.init(args=args)
    node = MySetParameterClass()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down parameters node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

