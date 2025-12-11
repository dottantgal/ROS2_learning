#  *
#  * @file message_sync.py
#  *
#  * @brief ROS2 node to synchronize messages
#  *        In this case I chose a PCL and a pure camera info messages, but it can be used
#  *        for every type of message and more than two
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer


class MessageSync(Node):
    """Node to synchronize messages from multiple topics."""

    def __init__(self):
        super().__init__('ros2_message_sync')
        
        # Publisher for modified point cloud
        self.pub_pcl_mod_ = self.create_publisher(PointCloud2, '/laser/pcl_mod', 10)
        
        # Regular subscription for point cloud (to align timestamps from bag files)
        self.sub_pcl_ = self.create_subscription(
            PointCloud2,
            '/laser/pcl',
            self.pcl_topic_callback,
            10
        )
        
        # Message filter subscribers for synchronization
        # A best practice would be to look at the QoS of the topics and make them consistent
        # These lines are just an example
        # qos = rclpy.qos.QoSProfile(depth=10)
        # self.pcl_sub_filter_ = Subscriber(self, PointCloud2, '/livox/lidar_mod', qos_profile=qos)
        
        # The two subscribers for the topics that must be synchronized
        self.pcl_sub_filter_ = Subscriber(self, PointCloud2, '/laser/pcl_mod')
        self.camera_info_sub_filter_ = Subscriber(self, CameraInfo, '/camera/camera_info')
        
        # Initialize the synchronizer with approximate time policy
        # The queue size is 10, slop is 0.1 seconds (time tolerance for synchronization)
        self.sync = ApproximateTimeSynchronizer(
            [self.pcl_sub_filter_, self.camera_info_sub_filter_],
            10,  # queue_size
            0.1  # slop: time tolerance in seconds for approximate time synchronization
        )
        self.sync.registerCallback(self.sync_message_callback)
    
    def pcl_topic_callback(self, msg):
        """
        Callback to subscribe the topic from the bag file and align the time stamp.
        """
        new_msg = PointCloud2()
        new_msg = msg
        new_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info('Received pcl and published with clock().now()')
        self.pub_pcl_mod_.publish(new_msg)
    
    def sync_message_callback(self, pcl, cam_info):
        """
        Synchronization callback that is called when messages from both topics are synchronized.
        """
        self.get_logger().info(
            f'Message sync time pcl={pcl.header.stamp.sec} camera_info={cam_info.header.stamp.sec}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MessageSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info('Shutting down message sync node.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

