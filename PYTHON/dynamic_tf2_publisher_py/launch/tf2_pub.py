from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments with default values
    parent_frame_arg = DeclareLaunchArgument(
        'parent_frame',
        default_value='map',
        description='Parent frame name'
    )
    
    child_frame_arg = DeclareLaunchArgument(
        'child_frame',
        default_value='base_link',
        description='Child frame name'
    )
    
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='X translation'
    )
    
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Y translation'
    )
    
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.0',
        description='Z translation'
    )
    
    roll_arg = DeclareLaunchArgument(
        'roll',
        default_value='0.0',
        description='Roll rotation (radians)'
    )
    
    pitch_arg = DeclareLaunchArgument(
        'pitch',
        default_value='0.0',
        description='Pitch rotation (radians)'
    )
    
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Yaw rotation (radians)'
    )
    
    # Create node with launch configuration substitutions
    # ROS 2 will automatically convert string LaunchConfiguration values to the correct types
    # based on the parameter declarations in the node (double for x, y, z, roll, pitch, yaw)
    tf2_publisher_node = Node(
        package='dynamic_tf2_publisher_py',
        executable='tf2_publisher_node',
        name='tf2_dynamic_pub_node',
        output='screen',
        parameters=[{
            'parent_frame': LaunchConfiguration('parent_frame'),
            'child_frame': LaunchConfiguration('child_frame'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'roll': LaunchConfiguration('roll'),
            'pitch': LaunchConfiguration('pitch'),
            'yaw': LaunchConfiguration('yaw')
        }]
    )
    
    return LaunchDescription([
        parent_frame_arg,
        child_frame_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        tf2_publisher_node
    ])

