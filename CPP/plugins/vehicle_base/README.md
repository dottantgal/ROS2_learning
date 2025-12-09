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

This package provides the base class and an example executable. To use it:

1. **Build both packages together**:
   ```bash
   colcon build --packages-select vehicle_base vehicle_plugins
   source install/setup.bash
   ```

2. **Run the example**:
   ```bash
   ros2 run vehicle_base create_vehicle
   ```

3. **Expected output**:
   ```
   First vehicle type is -> motorbike
   Second vehicle type is -> bicycle
   The plugin failed to load for some reason. Error: [error message for Rocket]
   ```

The `create_vehicle` executable demonstrates:
- Loading plugins dynamically using `pluginlib::ClassLoader`
- Creating instances of `Motorbike` and `Bicycle` plugins
- Handling plugin loading errors (trying to load non-existent `Rocket` plugin)

See `vehicle_plugins` package for the actual plugin implementations.

## Key Concepts
- **Abstract Base Class**: Pure virtual functions define the interface
- **Pluginlib**: ROS2 plugin loading system
- **Header Installation**: Installing headers for plugin implementations
- **Interface Design**: Designing extensible plugin interfaces

## Files
- `include/vehicle_base/regular_vehicle.hpp` - Base class definition
- `src/create_vehicle.cpp` - Example usage of plugins

