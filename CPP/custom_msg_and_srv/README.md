# custom_msg_and_srv

## Overview
This package demonstrates how to create and use custom service definitions in ROS2 C++. It defines a `CapitalFullName` service that capitalizes a full name from first and last name inputs.

## Purpose
Learn custom service definition:
- Creating custom `.srv` files
- Generating service interfaces using `rosidl`
- Using custom services in C++ nodes

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- C++14 or higher compiler

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select custom_msg_and_srv
source install/setup.bash
```

## Service Definition

### CapitalFullName.srv
```srv
string name
string surname
---
string capitalfullname
```

- **Request**: `name` and `surname` (strings)
- **Response**: `capitalfullname` (string) - Capitalized full name

## Usage

This package provides the service definition. To use it, see:
- `service_server_and_client` - C++ implementation examples
- `service_server_and_client_py` - Python implementation examples

### Calling the Service
```bash
ros2 service call /create_cap_full_name \
  custom_msg_and_srv/srv/CapitalFullName "{name: 'John', surname: 'Doe'}"
```

## Key Concepts
- **Service Definition**: `.srv` files define request and response types
- **Interface Generation**: `rosidl_generate_interfaces()` in CMakeLists.txt
- **Header Files**: Generated headers in `custom_msg_and_srv/srv/`
- **Usage**: Include generated headers in your nodes

## Files
- `srv/CapitalFullName.srv` - Service definition file

