# vehicle_base

## Overview
This package defines the base abstract class for a plugin system in ROS2 C++ using `pluginlib`. It provides the `RegularVehicle` interface that derived classes must implement.

## Purpose
Learn plugin system patterns:
- Creating abstract base classes for plugins
- Using `pluginlib` for dynamic loading
- Defining plugin interfaces
- Exporting base classes for plugin use

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- C++14 or higher compiler
- `pluginlib` package

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select vehicle_base
source install/setup.bash
```

## Base Class Interface

### RegularVehicle
Abstract base class with pure virtual methods:
- `initialize(vehicleType)` - Initialize the vehicle
- `setNumPassengers()` - Return number of passengers
- `getVehicleType()` - Return vehicle type string

## Usage
This package provides the base class. See `vehicle_plugins` for implementations and `create_vehicle.cpp` for usage examples.

## Key Concepts
- **Abstract Base Class**: Pure virtual functions define the interface
- **Pluginlib**: ROS2 plugin loading system
- **Header Installation**: Installing headers for plugin implementations
- **Interface Design**: Designing extensible plugin interfaces

## Files
- `include/vehicle_base/regular_vehicle.hpp` - Base class definition
- `src/create_vehicle.cpp` - Example usage of plugins

