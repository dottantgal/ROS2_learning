# dynamic_tf2_publisher

## Overview
This package demonstrates dynamic TF2 transformation publishing in C++ using `rclcpp` and `tf2_ros`. It allows runtime reconfiguration of transformation parameters without rebuilding.

## Purpose
Learn TF2 transformation publishing:
- Publishing static/dynamic transformations
- Using `tf2_ros::TransformBroadcaster`
- Runtime parameter reconfiguration
- Converting Euler angles (RPY) to quaternions
- Using launch files with parameters

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- C++14 or higher compiler
- `tf2_ros` and `tf2_geometry_msgs` packages

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select dynamic_tf2_publisher
source install/setup.bash
```

## Usage

### Running with Launch File
```bash
ros2 launch dynamic_tf2_publisher tf2_pub.py \
  parent_frame:=map child_frame:=base_link x:=1.0 y:=0.0 z:=0.5 yaw:=1.57
```

### Running the Node Directly
```bash
ros2 run dynamic_tf2_publisher tf2_publisher_node
```

### Setting Parameters at Runtime
```bash
ros2 param set /tf2_dynamic_pub_node parent_frame map
ros2 param set /tf2_dynamic_pub_node child_frame base_link
ros2 param set /tf2_dynamic_pub_node x 1.0
ros2 param set /tf2_dynamic_pub_node y 0.0
ros2 param set /tf2_dynamic_pub_node z 0.5
ros2 param set /tf2_dynamic_pub_node roll 0.0
ros2 param set /tf2_dynamic_pub_node pitch 0.0
ros2 param set /tf2_dynamic_pub_node yaw 1.57
```

### Viewing Transformations
```bash
ros2 run tf2_ros tf2_echo map base_link
```

## Parameters
- `parent_frame` (string) - Parent frame ID
- `child_frame` (string) - Child frame ID
- `x`, `y`, `z` (float) - Translation components
- `roll`, `pitch`, `yaw` (float) - Rotation in Euler angles (radians)
- `loop_rate` (int) - Publishing rate in Hz (default: 50)

## Key Concepts
- **Transform Broadcaster**: `tf2_ros::TransformBroadcaster` for publishing transforms
- **Transform Stamped**: `geometry_msgs::msg::TransformStamped` message type
- **Quaternion Conversion**: Converting RPY to quaternion using `tf2::Quaternion`
- **Dynamic Parameters**: Parameters can be changed at runtime
- **Launch Files**: Using Python launch files for parameter configuration

## Files
- `tf2_publisher.cpp` - TF2 publisher implementation
- `tf2_publisher_node.cpp` - Node entry point
- `include/dynamic_tf2_publisher/tf2_publisher.h` - Header file
- `launch/tf2_pub.py` - Launch file with examples
- `dyn_tf2_pub.rviz` - RViz configuration file

