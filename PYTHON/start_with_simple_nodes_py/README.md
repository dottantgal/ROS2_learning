# start_with_simple_nodes_py

## Overview
This package provides introductory examples for creating ROS2 nodes using `rclpy`. It demonstrates the fundamental concepts of node creation, logging, and timers in both functional and class-based approaches.

## Purpose
Learn the basics of ROS2 node development in Python:
- Creating ROS2 nodes
- Using loggers for output
- Implementing timers
- Class-based vs functional node design

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- Python 3.8 or higher

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select start_with_simple_nodes_py --symlink-install
source install/setup.bash
```

## Usage

### my_first_node
A minimal ROS2 node that demonstrates basic node creation:
```bash
ros2 run start_with_simple_nodes_py my_first_node
```

### node_with_class
Demonstrates class-based node implementation:
```bash
ros2 run start_with_simple_nodes_py node_with_class
```

### node_timer_without_class
Shows timer implementation using free functions:
```bash
ros2 run start_with_simple_nodes_py node_timer_without_class
```

### node_timer_with_class
Demonstrates timer implementation in a class-based node:
```bash
ros2 run start_with_simple_nodes_py node_timer_with_class
```

## Key Concepts
- **Node Creation**: Using `rclpy.node.Node`
- **Logging**: `node.get_logger().info()`, `warn()`, `error()`
- **Timers**: `create_timer()` for periodic callbacks
- **Spinning**: `rclpy.spin()` to keep the node alive

## Files
- `my_first_node.py` - Minimal node example
- `node_with_class.py` - Class-based node
- `node_timer_without_class.py` - Timer with free functions
- `node_timer_with_class.py` - Timer with class methods

