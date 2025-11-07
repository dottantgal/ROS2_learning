# ROS 2 C++ & Python Learning
##### _Practical examples for exploring rclcpp and rclpy on ROS 2 Humble_

This repository collects C++ and Python packages designed to illustrate the core concepts of ROS 2: nodes, timers, topics, services, actions, parameters, plugins, TF2, and more. Every example runs on ROS 2 Humble; most also work on other rolling releases, but Humble is the tested baseline.

---

## Prerequisites
- ROS 2 Humble installed and sourced (`source /opt/ros/humble/setup.bash`)
- `colcon` build tools (`python3-colcon-common-extensions`)
- `rosdep` to install system dependencies
- Recommended: Ubuntu 22.04 or another Humble-supported OS

---

## Getting Started
```bash
# create (or reuse) a colcon workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# clone this repository
git clone https://github.com/<your-org>/ros2-learning-examples.git

# install dependencies declared in every package.xml
cd ..
rosdep install --from-paths src --ignore-src --rosdistro humble -y

# build everything (C++ + Python packages)
colcon build --symlink-install

# remember to source the workspace before running examples
source install/setup.bash
```

---

## Repository Layout

### C++ Packages (`CPP/`)
Each subdirectory is a standalone ROS 2 package with its own `CMakeLists.txt` and `package.xml`, so you can copy individual folders into your workspace if preferred.

| Folder | Highlights |
| ------ | ---------- |
| `start_with_simple_nodes` | First `rclcpp::Node` examples (free functions vs. classes, timers). |
| `publisher_and_subscriber` | Minimal publishers/subscribers, custom message type (`EmployeeSalary.msg`) and class-based nodes. |
| `custom_msg_and_srv` | Shows how to create and use custom `.srv` definitions. |
| `service_server_and_client` | Blocking and asynchronous service clients/servers. |
| `parameters` | Parameter declaration, retrieval, and updates at runtime. |
| `plugins` | Demonstrates a pluginlib-based architecture (`vehicle_base`, `vehicle_plugins`). |
| `actions/action_tutorial` | Action clients/servers, including the Humble-compliant goal response callback signature. |
| `message_sync` | Synchronising multiple topics with message filters. |
| `create_library_with_header` | Building a reusable library (`publisher_library`) and consuming it from `use_library`. |
| `dynamic_tf2_publisher` | TF2 broadcaster configurable via parameters, with launch file (`launch/tf2_pub.py`) and RViz config. |

### Python Packages (`PYTHON/`)
Python equivalents live under their own packages and install via `setup.py`:

| Folder | Highlights |
| ------ | ---------- |
| `start_with_simple_nodes_py` | Introductory `rclpy` nodes (timers, logging, class vs. functional style). |
| `publisher_and_subscriber_py` | Publisher/subscriber examples, including class-based nodes and exception-aware loops. |

Both Python packages include lint tests (`ament_flake8`, `ament_pep257`) to showcase standard ROS 2 Python tooling.

---

## Running Selected Examples

### C++ Dynamic TF2 Publisher
```bash
source install/setup.bash
ros2 launch dynamic_tf2_publisher tf2_pub.py \
  parent_frame:=map child_frame:=base_link x:=1.0 y:=0.0 z:=0.5 yaw:=1.57
```
The launch file exposes every declared parameter so you can experiment without rebuilding. RViz configuration is available under `dynamic_tf2_publisher/dyn_tf2_pub.rviz`.

### C++ Publisher Library
The `publisher_library` package builds a shared library that creates a timed string publisher. The `use_library` executable links against it:
```bash
colcon build --packages-select publisher_library use_library
source install/setup.bash
ros2 run use_library use_library
```

### Python Publisher/Subscribers
```bash
colcon build --packages-select publisher_and_subscriber_py --symlink-install
source install/setup.bash
ros2 run publisher_and_subscriber_py simple_publisher_node
ros2 run publisher_and_subscriber_py simple_subscriber_node
```

---

## Testing & Linting
Several packages enable `ament_lint_auto`. You can run the full suite with:
```bash
colcon test
colcon test-result --all
```
For faster iteration, target specific packages, e.g. `colcon test --packages-select publisher_and_subscriber_py`.

---

## Contributing
Issues and pull requests are welcome. If you add a new example, please include:
- A short README snippet describing what learners will gain.
- Required dependencies in `package.xml` and `CMakeLists.txt` / `setup.py`.
- Launch files or test scripts when they add educational value.
