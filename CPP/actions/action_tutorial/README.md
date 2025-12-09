# action_tutorial

## Overview
This package demonstrates ROS2 actions in C++ using `rclcpp_action`. Actions provide a way to execute long-running tasks with feedback and cancellation support.

## Purpose
Learn ROS2 action patterns:
- Creating action servers
- Creating action clients
- Handling goal requests, cancellation, and execution
- Publishing feedback during execution
- Returning results upon completion

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- C++14 or higher compiler
- `custom_action` package (for action definition)

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select action_tutorial custom_action
source install/setup.bash
```

## Usage

### Simple Action Server
```bash
ros2 run action_tutorial simple_action_server
```

### Simple Action Client
```bash
ros2 run action_tutorial simple_action_client
```

### Class-based Action Server
```bash
ros2 run action_tutorial class_action_server
```

### Class-based Action Client
```bash
ros2 run action_tutorial class_action_client
```

### Sending Goals from Command Line
```bash
ros2 action send_goal /concatenation \
  custom_action/action/Concatenate "{num_concatenations: 5}"
```

## Key Concepts
- **Action Server**: `rclcpp_action::create_server<ActionType>()`
- **Action Client**: `rclcpp_action::create_client<ActionType>()`
- **Goal Handling**: `handle_goal()` callback to accept/reject goals
- **Cancel Handling**: `handle_cancel()` callback for cancellation requests
- **Execution**: `execute()` callback runs the action logic
- **Feedback**: `publish_feedback()` to send progress updates
- **Result**: `succeed()`, `canceled()`, or `abort()` to finish the action

## Action Definition
Uses `Concatenate` action from `custom_action`:
- **Goal**: `num_concatenations` (int16) - Number of concatenations to perform
- **Result**: `final_concatenation` (string) - Final concatenated string
- **Feedback**: `partial_concatenation` (string) - Progress update

## Files
- `simple_action_server.cpp` - Functional action server
- `simple_action_client.cpp` - Functional action client
- `class_action_server.cpp` - Class-based action server
- `class_action_client.cpp` - Class-based action client

