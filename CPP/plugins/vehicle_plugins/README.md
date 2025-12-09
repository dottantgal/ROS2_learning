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

### Running the Plugin Example

1. **Build both packages** (vehicle_base must be built first):
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
   The plugin failed to load for some reason. Error: [error details for Rocket]
   ```

### Available Plugins

You can load any of these plugins:
- `vehicle_plugins::Motorbike` - 2 passengers
- `vehicle_plugins::Bicycle` - 1 passenger  
- `vehicle_plugins::Truck` - 5 passengers

### Loading Plugins in Your Code

Example of loading and using a plugin:
```cpp
#include <pluginlib/class_loader.hpp>
#include "vehicle_base/regular_vehicle.hpp"

pluginlib::ClassLoader<vehicle_base::RegularVehicle> loader(
    "vehicle_base", "vehicle_base::RegularVehicle");

// Load a plugin
auto vehicle = loader.createUniqueInstance("vehicle_plugins::Motorbike");
vehicle->initialize("motorbike");
std::cout << "Type: " << vehicle->getVehicleType() << std::endl;
std::cout << "Passengers: " << vehicle->setNumPassengers() << std::endl;
```

### Plugin Registration

Plugins are registered in `plugins.xml`:
```xml
<library path="vehicle_plugins">
  <class type="vehicle_plugins::Motorbike" base_class_type="vehicle_base::RegularVehicle">
    <description>MotorBike plugin</description>
  </class>
  <class type="vehicle_plugins::Bicycle" base_class_type="vehicle_base::RegularVehicle">
    <description>Bicycle plugin</description>
  </class>
  <class type="vehicle_plugins::Truck" base_class_type="vehicle_base::RegularVehicle">
    <description>Truck plugin</description>
  </class>
</library>
```

## Key Concepts
- **Plugin Implementation**: Deriving from base class and implementing virtual methods
- **Plugin Registration**: Using `PLUGINLIB_EXPORT_CLASS` macro
- **Plugin XML**: Defining plugin metadata in `plugins.xml`
- **Dynamic Loading**: Using `pluginlib::ClassLoader` to load plugins at runtime

## Files
- `src/vehicle_plugins.cpp` - Plugin implementations
- `plugins.xml` - Plugin registration file

