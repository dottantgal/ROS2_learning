# service_server_and_client

## Overview
This package demonstrates ROS2 service communication in C++ using `rclcpp`. It includes both functional and class-based implementations of service servers and clients.

## Purpose
Learn ROS2 service patterns:
- Creating service servers
- Creating service clients
- Handling service requests and responses
- Blocking and asynchronous service calls
- Using custom service definitions

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- C++14 or higher compiler
- `custom_msg_and_srv` package (for custom service definition)

## Building
```bash
cd ~/ros2_ws/src
# Clone or copy this package
colcon build --packages-select service_server_and_client
source install/setup.bash
```

## Usage

### Service Server (Functional)
```bash
ros2 run service_server_and_client service_node
```

### Service Client (Functional)
```bash
ros2 run service_server_and_client client_node
```

### Service Server (Class-based)
```bash
ros2 run service_server_and_client service_node_class
```

### Service Client (Class-based)
```bash
ros2 run service_server_and_client client_node_class
```

### Calling the Service from Command Line
```bash
ros2 service call /create_cap_full_name \
  custom_msg_and_srv/srv/CapitalFullName "{name: 'John', surname: 'Doe'}"
```

## Key Concepts
- **Service Server**: `create_service<ServiceType>(service_name, callback)`
- **Service Client**: `create_client<ServiceType>(service_name)`
- **Synchronous Calls**: `client->async_send_request()` with `spin_until_future_complete()`
- **Service Callbacks**: Functions that receive request and populate response
- **Waiting for Service**: `wait_for_service()` to ensure server is available

## Custom Service
This package uses the `CapitalFullName` service from `custom_msg_and_srv`:
- **Request**: `name` (string), `surname` (string)
- **Response**: `capitalfullname` (string) - Capitalized full name

## Files
- `service_node.cpp` - Functional service server
- `client_node.cpp` - Functional service client
- `service_node_class.cpp` - Class-based service server
- `client_node_class.cpp` - Class-based service client

