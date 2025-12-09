# use_library

## Overview
This package demonstrates how to use a ROS2 library created by another package. It uses the `PublisherLibrary` from the `publisher_library` package.

## Purpose
Learn library usage patterns:
- Linking against external ROS2 libraries
- Using library classes in your nodes
- Dependency management between packages

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- C++14 or higher compiler
- `publisher_library` package (must be built first)

## Building
```bash
cd ~/ros2_ws/src
# Build publisher_library first
colcon build --packages-select publisher_library use_library
source install/setup.bash
```

## Usage

### Running the Node
```bash
ros2 run use_library use_library
```

The node will publish messages to `/my_published_msg` topic.

## Key Concepts
- **Library Linking**: `target_link_libraries()` in CMakeLists.txt
- **Dependencies**: Declaring `publisher_library` as a dependency
- **Include Paths**: Including library headers
- **Shared Pointers**: Using `std::make_shared` for library classes

## Files
- `use_library.cpp` - Node that uses the library

