# use_library_py

## Overview
This package demonstrates how to use a ROS2 library created by another package in Python. It uses the `PublisherLibrary` from the `publisher_library_py` package.

## Purpose
Learn library usage patterns in Python:
- Importing external ROS2 Python libraries
- Using library classes in your nodes
- Dependency management between packages

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- Python 3.8 or higher
- `publisher_library_py` package (must be built first)

## Building
```bash
cd ~/ros2_ws/src
# Build publisher_library_py first
colcon build --packages-select publisher_library_py use_library_py --symlink-install
source install/setup.bash
```

## Usage

### Running the Node
```bash
ros2 run use_library_py use_library
```

The node will publish messages to `/my_published_msg` topic.

## Key Concepts
- **Python Imports**: `from publisher_library_py import PublisherLibrary`
- **Dependencies**: Declaring `publisher_library_py` as a dependency in package.xml
- **Module Usage**: Instantiating and using library classes
- **Package Installation**: Ensuring dependencies are installed

## Files
- `use_library_py/use_library.py` - Node that uses the library

