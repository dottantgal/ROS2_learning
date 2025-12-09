# publisher_library_py

## Overview
This package demonstrates how to create a reusable ROS2 library in Python. It provides a `PublisherLibrary` class that can be imported and used by other packages.

## Purpose
Learn library creation patterns in Python:
- Creating reusable Python modules
- Using Python packages for library structure
- Building installable Python packages
- Exporting library interfaces

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- Python 3.8 or higher

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select publisher_library_py --symlink-install
source install/setup.bash
```

## Library Structure
- **Module**: `publisher_library_py/publisher_library.py` - Class implementation
- **Package Init**: `publisher_library_py/__init__.py` - Exports the class
- **setup.py**: Configures package installation

## Usage
This library is used by the `use_library_py` package. See that package for usage examples.

## Key Concepts
- **Python Modules**: Creating importable modules
- **Package Structure**: Using `__init__.py` for exports
- **Class Design**: Creating reusable Node-derived classes
- **Installation**: Making library available via `setup.py`

## Importing the Library
```python
from publisher_library_py import PublisherLibrary
```

## Files
- `publisher_library_py/publisher_library.py` - Library implementation
- `publisher_library_py/__init__.py` - Package initialization

