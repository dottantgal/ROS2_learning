# message_sync

## Overview
This package demonstrates message synchronization in ROS2 C++ using `message_filters`. It shows how to synchronize messages from multiple topics using approximate time synchronization.

## Purpose
Learn message synchronization patterns:
- Synchronizing messages from multiple topics
- Using `message_filters` library
- Approximate time synchronization policy
- Handling timestamp alignment for bag files

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- C++14 or higher compiler
- `message_filters` package
- `sensor_msgs` package

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select message_sync
source install/setup.bash
```

## Usage

### Running the Synchronizer
```bash
ros2 run message_sync message_sync
```

The node subscribes to:
- `/laser/pcl` - Point cloud messages (input)
- `/laser/pcl_mod` - Modified point cloud with updated timestamps
- `/camera/camera_info` - Camera info messages

### Publishing Test Data
You'll need to publish messages to the topics for synchronization to occur. The node will:
1. Receive point cloud from `/laser/pcl`
2. Update its timestamp to current time
3. Publish to `/laser/pcl_mod`
4. Synchronize `/laser/pcl_mod` with `/camera/camera_info`
5. Call the sync callback when both messages are available

## Key Concepts
- **Message Filters**: `message_filters::Subscriber` for filtered subscriptions
- **Synchronizer**: `message_filters::Synchronizer` with `ApproximateTime` policy
- **Approximate Time Sync**: Matches messages with similar timestamps (within a tolerance)
- **Queue Size**: Controls how many messages to buffer for synchronization
- **Timestamp Alignment**: Important when working with bag files

## Synchronization Policy
Uses `ApproximateTime` policy which:
- Matches messages with timestamps within a tolerance window
- Useful when messages don't arrive at exactly the same time
- Queue size of 10 messages

## Files
- `message_sync.cpp` - Message synchronization node

