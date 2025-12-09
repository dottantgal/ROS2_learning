# custom_msg_and_srv_py

## Overview
This package demonstrates how to create and use custom service definitions in ROS2 Python. It defines a `CapitalFullName` service that capitalizes a full name from first and last name inputs, and includes example server and client nodes.

## Purpose
Learn custom service definition in Python:
- Creating custom `.srv` files
- Generating service interfaces using `rosidl`
- Using custom services in Python nodes
- Implementing service servers and clients

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- Python 3.8 or higher

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select custom_msg_and_srv_py --symlink-install
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

### Service Server
```bash
ros2 run custom_msg_and_srv_py capital_full_name_server
```

### Service Client
```bash
ros2 run custom_msg_and_srv_py capital_full_name_client
```

### Calling the Service from Command Line
```bash
ros2 service call /capitalize_full_name \
  custom_msg_and_srv_py/srv/CapitalFullName "{name: 'John', surname: 'Doe'}"
```

## Key Concepts
- **Service Definition**: `.srv` files define request and response types
- **Interface Generation**: `rosidl_generate_interfaces()` in CMakeLists.txt
- **Python Import**: Import generated services: `from custom_msg_and_srv_py.srv import CapitalFullName`
- **Service Implementation**: Handle requests and return responses

## Files
- `srv/CapitalFullName.srv` - Service definition file
- `capital_full_name_server.py` - Example service server
- `capital_full_name_client.py` - Example service client
- `formatting.py` - Helper functions for name formatting

