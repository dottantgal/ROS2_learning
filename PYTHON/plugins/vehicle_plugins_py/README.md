# vehicle_plugins_py

## Overview
This package implements concrete vehicle plugins derived from `RegularVehicle` base class in Python. It provides `Motorbike`, `Bicycle`, and `Truck` implementations using setuptools entry points.

## Purpose
Learn plugin implementation patterns in Python:
- Implementing plugin interfaces
- Registering plugins with setuptools entry points
- Using entry points for plugin discovery
- Creating factory functions for plugins

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- Python 3.8 or higher
- `vehicle_base_py` package (must be built first)

## Building
```bash
cd ~/ros2_ws/src
# Build vehicle_base_py first
colcon build --packages-select vehicle_base_py vehicle_plugins_py --symlink-install
source install/setup.bash
```

## Available Plugins
- **Motorbike**: 2 passengers
- **Bicycle**: 1 passenger
- **Truck**: 5 passengers

## Usage

### Running the Plugin Demo
```bash
ros2 run vehicle_plugins_py create_vehicle
```

### Loading Plugins Programmatically
```python
from pkg_resources import iter_entry_points
from vehicle_base_py import RegularVehicle

# Load a plugin
for entry_point in iter_entry_points('vehicle_base_py.plugins', 'motorbike'):
    factory = entry_point.load()
    vehicle = factory()
    vehicle.initialize("motorbike")
    print(vehicle.get_vehicle_type())
```

## Key Concepts
- **Plugin Implementation**: Deriving from base class and implementing abstract methods
- **Entry Points**: Registering plugins in `setup.py` under `entry_points`
- **Factory Functions**: Creating factory functions for plugin instantiation
- **Dynamic Loading**: Using `pkg_resources` or `importlib.metadata` to discover plugins

## Entry Points
Plugins are registered in `setup.py`:
```python
entry_points={
    'vehicle_base_py.plugins': [
        'motorbike = vehicle_plugins_py.vehicle_plugins:create_motorbike',
        'bicycle = vehicle_plugins_py.vehicle_plugins:create_bicycle',
        'truck = vehicle_plugins_py.vehicle_plugins:create_truck',
    ],
}
```

## Files
- `vehicle_plugins_py/vehicle_plugins.py` - Plugin implementations
- `vehicle_plugins_py/create_vehicle.py` - Example usage of plugins

