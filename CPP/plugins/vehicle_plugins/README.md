# vehicle_plugins

## Overview
This package implements concrete vehicle plugins derived from `RegularVehicle` base class using `pluginlib`. It provides `Motorbike`, `Bicycle`, and `Truck` implementations.

## Purpose
Learn plugin implementation patterns:
- Implementing plugin interfaces
- Registering plugins with `pluginlib`
- Using `PLUGINLIB_EXPORT_CLASS` macro
- Creating plugin XML files

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- C++14 or higher compiler
- `pluginlib` package
- `vehicle_base` package (must be built first)

## Building
```bash
cd ~/ros2_ws/src
# Build vehicle_base first
colcon build --packages-select vehicle_base vehicle_plugins
source install/setup.bash
```

## Available Plugins
- **Motorbike**: 2 passengers
- **Bicycle**: 1 passenger
- **Truck**: 5 passengers

## Usage

### Loading Plugins Programmatically
See `vehicle_base/src/create_vehicle.cpp` for examples of loading and using plugins.

### Plugin Registration
Plugins are registered in `plugins.xml`:
```xml
<class type="vehicle_plugins::Motorbike" base_class_type="vehicle_base::RegularVehicle">
  <description>MotorBike plugin</description>
</class>
```

## Key Concepts
- **Plugin Implementation**: Deriving from base class and implementing virtual methods
- **Plugin Registration**: Using `PLUGINLIB_EXPORT_CLASS` macro
- **Plugin XML**: Defining plugin metadata in `plugins.xml`
- **Dynamic Loading**: Using `pluginlib::ClassLoader` to load plugins at runtime

## Files
- `src/vehicle_plugins.cpp` - Plugin implementations
- `plugins.xml` - Plugin registration file

