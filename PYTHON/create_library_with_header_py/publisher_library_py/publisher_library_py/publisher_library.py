#  *
#  * @file publisher_library.py
#  *
#  * @brief A basic publisher library made with Python module
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PublisherLibrary(Node):
    """Publisher library class that publishes messages periodically."""

    def __init__(self):
        super().__init__('ros2_publisher_library')
        self.counter_ = 0
        self.pub_ = self.create_publisher(String, '/my_published_msg', 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        """Timer callback that publishes a message."""
        message = String()
        message.data = f'HELLO WORLD number {self.counter_}'
        self.get_logger().info(f"Publishing: '{message.data}'")
        self.pub_.publish(message)
        self.counter_ += 1

