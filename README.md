# ROS 2 C++ & Python Learning
##### _Practical examples for exploring rclcpp and rclpy on ROS 2 Jazzy_

This repository collects C++ and Python packages designed to illustrate the core concepts of ROS 2: nodes, timers, topics, services, actions, parameters, plugins, TF2, and more. Every example runs on ROS 2 Jazzy; all packages are available in both C++ (`rclcpp`) and Python (`rclpy`) implementations.

---

## Prerequisites
- ROS 2 Jazzy installed and sourced (`source /opt/ros/jazzy/setup.bash`)
- `colcon` build tools (`python3-colcon-common-extensions`)
- `rosdep` to install system dependencies
- Recommended: Ubuntu 24.04 or another Jazzy-supported OS

---

## Getting Started

### Option 1: Using Docker (Recommended for Quick Setup)

The easiest way to get started is using the provided Docker environment:

```bash
# 1. Build the Docker image (one time)
cd docker
chmod +x build_image.sh run_container.sh attach_container.sh
./build_image.sh

# 2. Run the container (mounts repository automatically if ROS2_LEARNING_REPO is set)
export ROS2_LEARNING_REPO=$(pwd)/..  # Set to your repository path
./run_container.sh

# 3. Inside the container: copy packages and build
# If repository was mounted:
cp -r /mnt/ros2_learning/CPP/* ~/ros2_ws/src/
cp -r /mnt/ros2_learning/PYTHON/* ~/ros2_ws/src/

# Or use the helper script:
./helpers/copy_packages.sh /mnt/ros2_learning ~/ros2_ws all

# 4. Install dependencies and build
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# 5. Run examples
ros2 run start_with_simple_nodes my_first_node
```

**Note**: To open additional terminal sessions in the same container:
```bash
# From host (in docker directory)
./attach_container.sh
```

For detailed Docker instructions, see [docker/README.md](docker/README.md).

### Option 2: Native Installation

If you have ROS 2 Jazzy installed natively:

```bash
# create (or reuse) a colcon workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# clone this repository
git clone https://github.com/dottantgal/ROS2_learning.git
cd ROS2_learning
git checkout jazzy

# install dependencies declared in every package.xml
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

# build everything (C++ + Python packages)
colcon build --symlink-install

# remember to source the workspace before running examples
source install/setup.bash
```

---

## Repository Layout

### C++ Packages (`CPP/`)
Each subdirectory is a standalone ROS 2 package with its own `CMakeLists.txt` and `package.xml`, so you can copy individual folders into your workspace if preferred.

| Folder | Highlights | README |
| ------ | ---------- | ------ |
| `start_with_simple_nodes` | First `rclcpp::Node` examples (free functions vs. classes, timers). | [README](CPP/start_with_simple_nodes/README.md) |
| `publisher_and_subscriber` | Minimal publishers/subscribers, custom message type (`EmployeeSalary.msg`) and class-based nodes. | [README](CPP/publisher_and_subscriber/README.md) |
| `custom_msg_and_srv` | Shows how to create and use custom `.srv` definitions. | [README](CPP/custom_msg_and_srv/README.md) |
| `service_server_and_client` | Blocking and asynchronous service clients/servers. | [README](CPP/service_server_and_client/README.md) |
| `parameters` | Parameter declaration, retrieval, and updates at runtime. | [README](CPP/parameters/README.md) |
| `plugins/vehicle_base` | Base abstract class for plugin system using `pluginlib`. | [README](CPP/plugins/vehicle_base/README.md) |
| `plugins/vehicle_plugins` | Concrete plugin implementations (Motorbike, Bicycle, Truck). | [README](CPP/plugins/vehicle_plugins/README.md) |
| `actions/action_tutorial` | Action clients/servers, including Jazzy-compliant goal response callback signature. | [README](CPP/actions/action_tutorial/README.md) |
| `actions/custom_action` | Custom action definition (`Concatenate.action`). | [README](CPP/actions/custom_action/README.md) |
| `message_sync` | Synchronising multiple topics with message filters. | [README](CPP/message_sync/README.md) |
| `create_library_with_header/publisher_library` | Building a reusable library with header files. | [README](CPP/create_library_with_header/publisher_library/README.md) |
| `create_library_with_header/use_library` | Consuming a library from another package. | [README](CPP/create_library_with_header/use_library/README.md) |
| `dynamic_tf2_publisher` | TF2 broadcaster configurable via parameters, with launch file (`launch/tf2_pub.py`) and RViz config. | [README](CPP/dynamic_tf2_publisher/README.md) |

### Python Packages (`PYTHON/`)
Python equivalents live under their own packages and install via `setup.py`. All C++ packages have corresponding Python implementations.

| Folder | Highlights | README |
| ------ | ---------- | ------ |
| `start_with_simple_nodes_py` | Introductory `rclpy` nodes (timers, logging, class vs. functional style). | [README](PYTHON/start_with_simple_nodes_py/README.md) |
| `publisher_and_subscriber_py` | Publisher/subscriber examples, including class-based nodes and exception-aware loops. | [README](PYTHON/publisher_and_subscriber_py/README.md) |
| `custom_msg_and_srv_py` | Python example nodes using custom services from `custom_msg_and_srv`. | [README](PYTHON/custom_msg_and_srv_py/README.md) |
| `service_server_and_client_py` | Service server and client examples in Python. | [README](PYTHON/service_server_and_client_py/README.md) |
| `parameters_py` | Parameter handling examples in Python. | [README](PYTHON/parameters_py/README.md) |
| `plugins/vehicle_base_py` | Base abstract class for Python plugin system using `abc` module. | [README](PYTHON/plugins/vehicle_base_py/README.md) |
| `plugins/vehicle_plugins_py` | Concrete plugin implementations using setuptools entry points. | [README](PYTHON/plugins/vehicle_plugins_py/README.md) |
| `actions/action_tutorial_py` | Action clients/servers in Python using custom actions from `custom_action`. | [README](PYTHON/actions/action_tutorial_py/README.md) |
| `message_sync_py` | Message synchronization using `message_filters` in Python. | [README](PYTHON/message_sync_py/README.md) |
| `create_library_with_header_py/publisher_library_py` | Building a reusable Python library/module. | [README](PYTHON/create_library_with_header_py/publisher_library_py/README.md) |
| `create_library_with_header_py/use_library_py` | Consuming a Python library from another package. | [README](PYTHON/create_library_with_header_py/use_library_py/README.md) |
| `dynamic_tf2_publisher_py` | TF2 broadcaster in Python with launch file support. | [README](PYTHON/dynamic_tf2_publisher_py/README.md) |

---

## Running Selected Examples

### C++ Dynamic TF2 Publisher
```bash
source install/setup.bash
ros2 launch dynamic_tf2_publisher tf2_pub.py \
  parent_frame:=map child_frame:=base_link x:=1.0 y:=0.0 z:=0.5 yaw:=1.57
```
The launch file exposes every declared parameter so you can experiment without rebuilding. RViz configuration is available under `dynamic_tf2_publisher/dyn_tf2_pub.rviz`.

### Python Dynamic TF2 Publisher
```bash
source install/setup.bash
ros2 launch dynamic_tf2_publisher_py tf2_pub.py \
  parent_frame:=map child_frame:=base_link x:=1.0 y:=0.0 z:=0.5 yaw:=1.57
```

### C++ Publisher Library
The `publisher_library` package builds a shared library that creates a timed string publisher. The `use_library` executable links against it:
```bash
colcon build --packages-select publisher_library use_library
source install/setup.bash
ros2 run use_library use_library
```

### Python Publisher Library
The `publisher_library_py` package provides a Python module that creates a timed string publisher. The `use_library_py` executable uses it:
```bash
colcon build --packages-select publisher_library_py use_library_py --symlink-install
source install/setup.bash
ros2 run use_library_py use_library
```

### Python Publisher/Subscribers
```bash
colcon build --packages-select publisher_and_subscriber_py --symlink-install
source install/setup.bash
ros2 run publisher_and_subscriber_py simple_publisher_node
ros2 run publisher_and_subscriber_py simple_subscriber_node
```

### C++ Plugin System
```bash
colcon build --packages-select vehicle_base vehicle_plugins
source install/setup.bash
ros2 run vehicle_base create_vehicle
```

### Python Plugin System
```bash
colcon build --packages-select vehicle_base_py vehicle_plugins_py --symlink-install
source install/setup.bash
ros2 run vehicle_plugins_py create_vehicle
```

---

## Key Features

### Complete C++ and Python Coverage
Every C++ package has a corresponding Python implementation, allowing you to:
- Compare implementations side-by-side
- Learn both `rclcpp` and `rclpy` APIs
- Choose the language that fits your project

### Progressive Learning Path
Packages are organized from simple to complex:
1. **Basic Nodes** - Start here for fundamentals
2. **Communication** - Publishers, subscribers, services
3. **Advanced** - Actions, parameters, TF2
4. **Architecture** - Libraries, plugins, synchronization

### ROS 2 Jazzy Compatible
All packages are tested and verified for ROS 2 Jazzy:
- Updated APIs and patterns
- Modern C++17 standard (Jazzy recommendation) and Python 3.8+ support
- Jazzy-specific features and best practices
- Modern parameter API (`get_parameter().as_<type>()`)
- CMake 3.8+ requirement

---

## Contributing
Issues and pull requests are welcome. If you add a new example, please include:
- A short README snippet describing what learners will gain.
- Required dependencies in `package.xml` and `CMakeLists.txt` / `setup.py`.
- Launch files or test scripts when they add educational value.
- Both C++ and Python implementations when applicable.

---

## License
TODO: License declaration

---

## Maintainer
Antonio Mauro Galiano
