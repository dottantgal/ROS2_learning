from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamic_tf2_publisher',
            executable='tf2_publisher_node',
            name='camera_frame_to_camera_mount',
            output='screen',
            parameters=[
                {'parent_frame' : 'camera_mount'},
                {'child_frame' : 'camera_frame'},
                {'x' : 0.0},
                {'y' : 0.0},
                {'z' : 0.0},
                {'roll' : 1.5707},
                {'pitch' : 3.14159},
                {'yaw' : 1.5707}
            ]
        ),
        Node(
            package='dynamic_tf2_publisher',
            executable='tf2_publisher_node',
            name='camera_mount_to_map',
            output='screen',
            parameters=[
                {'parent_frame' : 'map'},
                {'child_frame' : 'camera_mount'},
                {'x' : -1.0},
                {'y' : 0.0},
                {'z' : 3.0},
                {'roll' : 0.0},
                {'pitch' : 0.0},
                {'yaw' : 3.14159}
            ]
        ),
        Node(
            package='dynamic_tf2_publisher',
            executable='tf2_publisher_node',
            name='velodyne_frame_to_map',
            output='screen',
            parameters=[
                {'parent_frame' : 'map'},
                {'child_frame' : 'velodyne_laser'},
                {'x' : 0.0},
                {'y' : 0.0},
                {'z' : 3.0},
                {'roll' : 0.0},
                {'pitch' : 0.0},
                {'yaw' : 3.14159}
            ]
        )
    ])
