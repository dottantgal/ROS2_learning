# publisher_library

## Overview
This package demonstrates how to create a reusable ROS2 library in C++ with header files. It provides a `PublisherLibrary` class that can be used by other packages.

## Purpose
Learn library creation patterns:
- Creating reusable C++ libraries
- Using header files for class definitions
- Building shared libraries with CMake
- Exporting library interfaces

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- C++14 or higher compiler

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select publisher_library
source install/setup.bash
```

## Library Structure
- **Header**: `include/publisher_library/publisher_library.h` - Class declaration
- **Implementation**: `src/publisher_library.cpp` - Class implementation
- **CMakeLists.txt**: Configures library build and installation

## Usage
This library is used by the `use_library` package. See that package for usage examples.

## Key Concepts
- **Library Creation**: `add_library()` in CMakeLists.txt
- **Header Installation**: Installing headers to `include/` directory
- **Class Design**: Creating reusable Node-derived classes
- **Export**: Making library available to other packages

## Files
- `include/publisher_library/publisher_library.h` - Header file
- `src/publisher_library.cpp` - Implementation file

