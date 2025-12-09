# vehicle_base_py

## Overview
This package defines the base abstract class for a plugin system in ROS2 Python. It provides the `RegularVehicle` interface that derived classes must implement using Python's `abc` module.

## Purpose
Learn plugin system patterns in Python:
- Creating abstract base classes for plugins
- Using Python's `abc` module
- Defining plugin interfaces
- Exporting base classes for plugin use

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- Python 3.8 or higher

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select vehicle_base_py --symlink-install
source install/setup.bash
```

## Base Class Interface

### RegularVehicle
Abstract base class with abstract methods:
- `initialize(vehicle_type: str)` - Initialize the vehicle
- `set_num_passengers()` - Return number of passengers
- `get_vehicle_type()` - Return vehicle type string

## Usage
This package provides the base class. See `vehicle_plugins_py` for implementations and `create_vehicle.py` for usage examples.

## Key Concepts
- **Abstract Base Class**: Using `ABC` and `@abstractmethod` decorator
- **Python Entry Points**: Using setuptools entry points for plugin discovery
- **Module Exports**: Exporting base class via `__init__.py`
- **Interface Design**: Designing extensible plugin interfaces

## Files
- `vehicle_base_py/regular_vehicle.py` - Base class definition

