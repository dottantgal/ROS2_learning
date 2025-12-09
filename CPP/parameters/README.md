# parameters

## Overview
This package demonstrates ROS2 parameter handling in C++ using `rclcpp`. It shows how to declare, retrieve, and dynamically update parameters at runtime.

## Purpose
Learn ROS2 parameter management:
- Declaring parameters with default values
- Retrieving parameter values
- Updating parameters at runtime via command line or launch files
- Using parameters to configure node behavior

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- C++14 or higher compiler

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select parameters
source install/setup.bash
```

## Usage

### Running the Node
```bash
ros2 run parameters parameters
```

### Setting Parameters at Runtime
You can change parameters while the node is running:

```bash
# Set vehicle type
ros2 param set /set_parameter_node vehicle_type car

# Set vehicle speed
ros2 param set /set_parameter_node vehicle_speed 100
```

### Viewing Parameters
```bash
# List all parameters
ros2 param list /set_parameter_node

# Get parameter value
ros2 param get /set_parameter_node vehicle_type
ros2 param get /set_parameter_node vehicle_speed
```

### Using Launch Files
Parameters can also be set via launch files:
```yaml
parameters:
  - vehicle_type: "truck"
  - vehicle_speed: 80
```

## Key Concepts
- **Parameter Declaration**: `declare_parameter<Type>(name, default_value)`
- **Parameter Retrieval**: `get_parameter(name, variable)`
- **Dynamic Updates**: Parameters can be changed at runtime without restarting
- **Parameter Types**: Supports int, float, string, bool, arrays, etc.

## Parameters
- `vehicle_type` (string, default: "bike") - Type of vehicle
- `vehicle_speed` (int, default: 10) - Speed of the vehicle

## Files
- `parameters.cpp` - Parameter handling node implementation

