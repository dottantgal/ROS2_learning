# publisher_and_subscriber_py

## Overview
This package demonstrates ROS2 publisher and subscriber patterns in Python using `rclpy`. It includes examples of both functional and class-based implementations.

## Purpose
Learn ROS2 communication patterns in Python:
- Creating publishers and subscribers
- Publishing and subscribing to topics
- Working with standard message types
- Class-based vs functional implementations

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- Python 3.8 or higher

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select publisher_and_subscriber_py --symlink-install
source install/setup.bash
```

## Usage

### Simple Publisher
```bash
ros2 run publisher_and_subscriber_py simple_publisher_node
```

### Simple Subscriber
```bash
ros2 run publisher_and_subscriber_py simple_subscriber_node
```

### Class-based Publisher
```bash
ros2 run publisher_and_subscriber_py simple_publisher_class_node
```

### Class-based Subscriber
```bash
ros2 run publisher_and_subscriber_py simple_subscriber_class_node
```

## Key Concepts
- **Publishers**: `self.create_publisher(MessageType, topic_name, queue_size)`
- **Subscribers**: `self.create_subscription(MessageType, topic_name, queue_size, callback)`
- **Message Publishing**: `publisher.publish(message)`
- **Callbacks**: Methods or functions for message handling
- **Exception Handling**: Using try/except for graceful shutdown

## Files
- `simple_publisher_node.py` - Basic publisher
- `simple_subscriber_node.py` - Basic subscriber
- `simple_publisher_class_node.py` - Class-based publisher
- `simple_subscriber_class_node.py` - Class-based subscriber

