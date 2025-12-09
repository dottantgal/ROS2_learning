# action_tutorial_py

## Overview
This package demonstrates ROS2 actions in Python using `rclpy.action`. Actions provide a way to execute long-running tasks with feedback and cancellation support.

## Purpose
Learn ROS2 action patterns in Python:
- Creating action servers
- Creating action clients
- Handling goal requests, cancellation, and execution
- Publishing feedback during execution
- Returning results upon completion

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- Python 3.8 or higher
- `custom_action_py` package (for action definition)

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select action_tutorial_py custom_action_py
source install/setup.bash
```

## Usage

### Simple Action Server
```bash
ros2 run action_tutorial_py simple_action_server
```

### Simple Action Client
```bash
ros2 run action_tutorial_py simple_action_client
```

### Class-based Action Server
```bash
ros2 run action_tutorial_py class_action_server
```

### Class-based Action Client
```bash
ros2 run action_tutorial_py class_action_client
```

### Sending Goals from Command Line
```bash
ros2 action send_goal /concatenation \
  custom_action_py/action/Concatenate "{num_concatenations: 5}"
```

## Key Concepts
- **Action Server**: `ActionServer(node, ActionType, action_name, execute_callback, ...)`
- **Action Client**: `ActionClient(node, ActionType, action_name)`
- **Goal Handling**: `goal_callback` to accept/reject goals
- **Cancel Handling**: `cancel_callback` for cancellation requests
- **Execution**: `execute_callback` runs the action logic
- **Feedback**: `goal_handle.publish_feedback()` to send progress updates
- **Result**: `goal_handle.succeed()`, `canceled()`, or `abort()` to finish

## Action Definition
Uses `Concatenate` action from `custom_action_py`:
- **Goal**: `num_concatenations` (int16) - Number of concatenations to perform
- **Result**: `final_concatenation` (string) - Final concatenated string
- **Feedback**: `partial_concatenation` (string) - Progress update

## Files
- `simple_action_server.py` - Functional action server
- `simple_action_client.py` - Functional action client
- `class_action_server.py` - Class-based action server
- `class_action_client.py` - Class-based action client

