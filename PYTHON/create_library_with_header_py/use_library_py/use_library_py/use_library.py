#  *
#  * @file use_library.py
#  *
#  * @brief A basic node to use the publisher library
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#

import rclpy
from publisher_library_py import PublisherLibrary


def main(args=None):
    rclpy.init(args=args)
    node = PublisherLibrary()
    node.get_logger().info("My Use Publisher Library ROS2 node started")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down use_library node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()

