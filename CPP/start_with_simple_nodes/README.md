# start_with_simple_nodes

## Overview
This package provides introductory examples for creating ROS2 nodes using `rclcpp`. It demonstrates the fundamental concepts of node creation, logging, and timers in both functional and class-based approaches.

## Purpose
Learn the basics of ROS2 node development in C++:
- Creating ROS2 nodes
- Using loggers for output
- Implementing timers
- Class-based vs functional node design

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- C++14 or higher compiler

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select start_with_simple_nodes
source install/setup.bash
```

## Usage

### my_first_node
A minimal ROS2 node that demonstrates basic node creation:
```bash
ros2 run start_with_simple_nodes my_first_node
```

### node_with_class
Demonstrates class-based node implementation:
```bash
ros2 run start_with_simple_nodes node_with_class
```

### node_timer_without_class
Shows timer implementation using free functions:
```bash
ros2 run start_with_simple_nodes node_timer_without_class
```

### node_timer_with_class
Demonstrates timer implementation in a class-based node:
```bash
ros2 run start_with_simple_nodes node_timer_with_class
```

## Key Concepts
- **Node Creation**: Using `rclcpp::Node` and `std::make_shared`
- **Logging**: `RCLCPP_INFO`, `RCLCPP_WARN`, `RCLCPP_ERROR`
- **Timers**: `create_wall_timer()` for periodic callbacks
- **Spinning**: `rclcpp::spin()` to keep the node alive

## Files
- `my_first_node.cpp` - Minimal node example
- `node_with_class.cpp` - Class-based node
- `node_timer_without_class.cpp` - Timer with free functions
- `node_timer_with_class.cpp` - Timer with class methods

