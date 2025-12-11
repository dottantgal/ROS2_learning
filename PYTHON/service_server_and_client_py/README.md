# service_server_and_client_py

## Overview
This package demonstrates ROS2 service communication in Python using `rclpy`. It includes both functional and class-based implementations of service servers and clients.

## Purpose
Learn ROS2 service patterns in Python:
- Creating service servers
- Creating service clients
- Handling service requests and responses
- Asynchronous service calls
- Using custom service definitions

## Prerequisites
- ROS2 Jazzy installed and sourced
- `colcon` build tools
- Python 3.8 or higher
- `custom_msg_and_srv` package (C++ package with custom service definition)

## Building

**Important:** This package depends on the C++ package `custom_msg_and_srv`. Build it first:

```bash
cd ~/ros2_ws
# Build the C++ package with custom service definition first
colcon build --packages-select custom_msg_and_srv --symlink-install
# Then build this Python package
colcon build --packages-select service_server_and_client_py --symlink-install
source install/setup.bash
```

Or build all packages together:
```bash
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Service Server (Functional)
```bash
ros2 run service_server_and_client_py service_node
```

### Service Client (Functional)
```bash
ros2 run service_server_and_client_py client_node
```

### Service Server (Class-based)
```bash
ros2 run service_server_and_client_py service_node_class
```

### Service Client (Class-based)
```bash
ros2 run service_server_and_client_py client_node_class
```

### Calling the Service from Command Line
```bash
ros2 service call /create_cap_full_name \
  custom_msg_and_srv/srv/CapitalFullName "{name: 'John', surname: 'Doe'}"
```

## Key Concepts
- **Service Server**: `self.create_service(ServiceType, service_name, callback)`
- **Service Client**: `self.create_client(ServiceType, service_name)`
- **Asynchronous Calls**: `client.call_async(request)` with `spin_until_future_complete()`
- **Service Callbacks**: Methods that receive request and return response
- **Waiting for Service**: `wait_for_service(timeout_sec)` to ensure server is available

## Custom Service
This package uses the `CapitalFullName` service from the C++ package `custom_msg_and_srv`:
- **Request**: `name` (string), `surname` (string)
- **Response**: `capitalfullname` (string) - Capitalized full name

**Note:** Following ROS 2 best practices, custom interfaces (messages, services, actions) are defined in C++ packages using `ament_cmake` and `rosidl_generate_interfaces`. Python packages then depend on these C++ packages to use the interfaces. This ensures consistency across languages and follows the recommended architecture.

## Files
- `service_node.py` - Functional service server
- `client_node.py` - Functional service client
- `service_node_class.py` - Class-based service server
- `client_node_class.py` - Class-based service client

