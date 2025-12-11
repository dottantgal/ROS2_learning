#  *
#  * @file tf2_publisher.py
#  *
#  * @brief ROS2 node to publish TF2 transformation with run time adjustments through
#  *        parameters setting
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#

import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


def quaternion_from_euler(ai, aj, ak):
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion (x, y, z, w).
    Based on the ROS 2 Jazzy tutorial implementation.
    
    Args:
        ai: Roll (rotation around x-axis) in radians
        aj: Pitch (rotation around y-axis) in radians
        ak: Yaw (rotation around z-axis) in radians
    
    Returns:
        Tuple (x, y, z, w) representing the quaternion
    """
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    qx = cj * sc - sj * cs
    qy = cj * ss + sj * cc
    qz = cj * cs - sj * sc
    qw = cj * cc + sj * ss

    return (qx, qy, qz, qw)


class Tf2Publisher(Node):
    """Node to publish TF2 transformations with runtime parameter adjustments."""

    def __init__(self):
        super().__init__('tf2_dynamic_pub_node')
        self.get_logger().info('Starting the dynamic TF2 publisher node')
        
        self._tf_broadcaster = TransformBroadcaster(self)
        self._transform = TransformStamped()
        
        self.declare_params()
        loop_rate = self.get_parameter('loop_rate').value
        self._timer = self.create_timer(loop_rate / 1000.0, self.timer_callback)

    def declare_params(self):
        """Declare all parameters with default values."""
        self.declare_parameter('parent_frame', '')
        self.declare_parameter('child_frame', '')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('loop_rate', 50)

    def timer_callback(self):
        """Timer callback that publishes the transform."""
        # Get parameter values (can be changed at runtime)
        parent_frame = self.get_parameter('parent_frame').value
        child_frame = self.get_parameter('child_frame').value
        x_pos = self.get_parameter('x').value
        y_pos = self.get_parameter('y').value
        z_pos = self.get_parameter('z').value
        roll = self.get_parameter('roll').value
        pitch = self.get_parameter('pitch').value
        yaw = self.get_parameter('yaw').value

        # Check if frame IDs are set
        if not parent_frame or not child_frame:
            # Only log warning once to avoid spam
            if not hasattr(self, '_frame_warning_logged'):
                self.get_logger().warn(
                    'parent_frame and child_frame parameters must be set. '
                    'Use --ros-args -p parent_frame:=<frame> -p child_frame:=<frame> '
                    'or use the launch file with parameters.'
                )
                self._frame_warning_logged = True
            return

        # Set transform header and frame IDs
        self._transform.header.frame_id = parent_frame
        self._transform.child_frame_id = child_frame
        
        # Set translation
        self._transform.transform.translation.x = x_pos
        self._transform.transform.translation.y = y_pos
        self._transform.transform.translation.z = z_pos

        # Convert RPY to quaternion
        quat = quaternion_from_euler(roll, pitch, yaw)
        self._transform.transform.rotation.x = quat[0]
        self._transform.transform.rotation.y = quat[1]
        self._transform.transform.rotation.z = quat[2]
        self._transform.transform.rotation.w = quat[3]

        try:
            # Set timestamp and send transform
            self._transform.header.stamp = self.get_clock().now().to_msg()
            self._tf_broadcaster.sendTransform(self._transform)
        except Exception as ex:
            self.get_logger().info(f'Transformation error = {ex}')


def main(args=None):
    rclpy.init(args=args)
    node = Tf2Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info('Stopped the dynamic TF2 publisher node')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

