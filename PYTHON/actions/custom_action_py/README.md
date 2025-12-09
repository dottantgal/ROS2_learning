# custom_action_py

## Overview
This package defines a custom ROS2 action interface in Python. It provides the `Concatenate` action definition used by action tutorial examples.

## Purpose
Learn custom action definition in Python:
- Creating custom `.action` files
- Generating action interfaces using `rosidl`
- Using custom actions in Python nodes

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- Python 3.8 or higher

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select custom_action_py
source install/setup.bash
```

## Action Definition

### Concatenate.action
```action
int16 num_concatenations
---
string final_concatenation
---
string partial_concatenation
```

- **Goal**: `num_concatenations` (int16) - Number of string concatenations to perform
- **Result**: `final_concatenation` (string) - Final concatenated string result
- **Feedback**: `partial_concatenation` (string) - Progress update during execution

## Usage

This package provides the action definition. To use it, see:
- `action_tutorial` - C++ implementation examples
- `action_tutorial_py` - Python implementation examples

### Sending Goals
```bash
ros2 action send_goal /concatenation \
  custom_action_py/action/Concatenate "{num_concatenations: 5}"
```

## Key Concepts
- **Action Definition**: `.action` files define goal, result, and feedback types
- **Interface Generation**: `rosidl_generate_interfaces()` in CMakeLists.txt
- **Python Import**: Import generated actions: `from custom_action_py.action import Concatenate`
- **Usage**: Use in action servers and clients

## Files
- `action/Concatenate.action` - Action definition file

