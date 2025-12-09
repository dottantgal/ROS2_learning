# publisher_and_subscriber

## Overview
This package demonstrates ROS2 publisher and subscriber patterns in C++ using `rclcpp`. It includes examples of both functional and class-based implementations, as well as custom message types.

## Purpose
Learn ROS2 communication patterns:
- Creating publishers and subscribers
- Publishing and subscribing to topics
- Working with standard and custom message types
- Class-based vs functional implementations

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- C++14 or higher compiler

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select publisher_and_subscriber
source install/setup.bash
```

## Usage

### Simple Publisher
```bash
ros2 run publisher_and_subscriber simple_publisher_node
```

### Simple Subscriber
```bash
ros2 run publisher_and_subscriber simple_subscriber_node
```

### Class-based Publisher
```bash
ros2 run publisher_and_subscriber simple_publisher_class_node
```

### Class-based Subscriber
```bash
ros2 run publisher_and_subscriber simple_subscriber_class_node
```

### Publisher-Subscriber Pipeline
A node that both publishes and subscribes:
```bash
ros2 run publisher_and_subscriber sub_pub_pipeline
```

### Custom Message Publisher
Publishes custom `EmployeeSalary` messages:
```bash
ros2 run publisher_and_subscriber publish_custom_message
```

## Custom Messages
This package defines a custom message type:
- `EmployeeSalary.msg` - Contains employee salary information

## Key Concepts
- **Publishers**: `create_publisher<MessageType>(topic_name, queue_size)`
- **Subscribers**: `create_subscription<MessageType>(topic_name, queue_size, callback)`
- **Custom Messages**: Using `rosidl_generate_interfaces()` in CMakeLists.txt
- **Message Publishing**: `publisher->publish(message)`
- **Callbacks**: Function pointers or lambdas for message handling

## Files
- `simple_publisher_node.cpp` - Basic publisher
- `simple_subscriber_node.cpp` - Basic subscriber
- `simple_publisher_class_node.cpp` - Class-based publisher
- `simple_subscriber_class_node.cpp` - Class-based subscriber
- `sub_pub_pipeline.cpp` - Combined publisher/subscriber
- `publish_custom_message.cpp` - Custom message publisher
- `msg/EmployeeSalary.msg` - Custom message definition

