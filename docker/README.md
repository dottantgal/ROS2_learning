# ROS 2 Jazzy Docker Environment

This directory contains scripts and configuration to create a minimal ROS 2 Jazzy Docker environment for building and testing the ROS 2 learning packages.

## Overview

The Docker setup provides:
- **Minimal ROS 2 Jazzy base image** (`ros:jazzy-ros-base`) - lightweight, no GUI
- **Pre-installed dependencies** - All common ROS 2 packages needed for the learning examples
- **Build tools** - `colcon`, `rosdep`, C++17 compiler, Python 3.8+
- **Helper scripts** - Optional utilities for workspace management

## Quick Start

### 1. Build the Docker Image

```bash
cd docker
chmod +x build_image.sh
./build_image.sh
```

This creates a Docker image named `ros2-jazzy-learning:latest` with all necessary dependencies.

### 2. Run the Container

```bash
chmod +x run_container.sh
./run_container.sh
```

Or with a custom container name:
```bash
./run_container.sh my-ros2-container
```

**Optional**: Mount the repository for easy package copying:
```bash
export ROS2_LEARNING_REPO=/path/to/ROS2_learning
./run_container.sh
```

This starts an interactive container with:
- ROS 2 Jazzy environment pre-sourced
- Workspace directory at `~/ros2_ws`
- Persistent volume for build artifacts
- Repository mounted at `/mnt/ros2_learning` (if ROS2_LEARNING_REPO is set)

### 3. Open Additional Shell Sessions

To open a new shell in an already running container:
```bash
chmod +x attach_container.sh
./attach_container.sh
```

Or with a custom container name:
```bash
./attach_container.sh my-ros2-container
```

This is useful for:
- Running multiple terminals simultaneously
- Monitoring logs while building
- Running nodes while editing code

## Usage Workflow

### Inside the Container

1. **Copy packages into workspace** (Option B - isolated builds):
   ```bash
   # If you mounted the repository (via ROS2_LEARNING_REPO), copy packages:
   cp -r /mnt/ros2_learning/CPP/* ~/ros2_ws/src/
   cp -r /mnt/ros2_learning/PYTHON/* ~/ros2_ws/src/
   
   # Or use the helper script:
   ./helpers/copy_packages.sh /mnt/ros2_learning ~/ros2_ws all
   ```

2. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build packages**:
   ```bash
   colcon build --symlink-install
   ```

4. **Source and use**:
   ```bash
   source install/setup.bash
   ros2 run start_with_simple_nodes my_first_node
   ```

### Helper Scripts (Optional)

Helper scripts are available in the container at `/root/ros2_ws/helpers/`:

- **`setup_workspace.sh`** - Create a new workspace structure
  ```bash
  ./helpers/setup_workspace.sh ~/my_custom_ws
  ```

- **`copy_packages.sh`** - Copy packages from source to workspace
  ```bash
  ./helpers/copy_packages.sh /tmp/ros2_learning ~/ros2_ws all
  # Options: 'cpp', 'python', or 'all'
  ```

- **`build_packages.sh`** - Build packages in workspace
  ```bash
  ./helpers/build_packages.sh ~/ros2_ws
  # Or build specific packages:
  ./helpers/build_packages.sh ~/ros2_ws parameters parameters_py
  ```

## Container Management

### Start/Stop Container

```bash
# Start existing container
docker start ros2-jazzy-ws

# Attach to running container (use the script)
./attach_container.sh

# Or manually:
docker exec -it ros2-jazzy-ws /bin/bash

# Stop container
docker stop ros2-jazzy-ws

# Remove container (keeps volume)
docker rm ros2-jazzy-ws
```

### Persistent Storage

The container uses a named Docker volume (`ros2-jazzy-ws-workspace`) to persist:
- Build artifacts (`install/`, `build/`, `log/`)
- Workspace structure
- User files

To remove the volume (clean slate):
```bash
docker volume rm ros2-jazzy-ws-workspace
```

## Image Details

### Base Image
- `ros:jazzy-ros-base` - Minimal ROS 2 Jazzy installation (no GUI)

### Pre-installed Packages
- **Core ROS 2**: `rclcpp`, `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`
- **TF2**: `tf2-ros`, `tf2-geometry-msgs`
- **Advanced**: `message-filters`, `pluginlib`, `rclcpp-action`, `rclcpp-components`
- **Build Tools**: `rosidl-default-generators`, `ament-cmake` (ament_python included in base image)
- **Development**: `colcon`, `rosdep`, `git`, `vim`, `nano`

### System Requirements
- Docker installed
- ~2GB disk space for image
- Additional space for volumes

## Troubleshooting

### Image not found
```bash
# Rebuild the image
./build_image.sh
```

### Container already exists
```bash
# Remove old container
docker rm ros2-jazzy-ws
# Or use a different name
./run_container.sh my-new-container
```

### Permission issues
```bash
# Ensure scripts are executable
chmod +x build_image.sh run_container.sh
```

### Build errors
```bash
# Clean build
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
```

## Notes

- The image is **minimal** - no GUI tools (rviz, rqt) to keep it lightweight
- Packages are **copied** into the container (Option B) for isolated builds
- Workspace is **persistent** via Docker volume (Option B - named volume)
- Helper scripts are **optional** - users can manage workspaces manually if preferred

## Example: Complete Workflow

```bash
# 1. Build image (one time)
cd docker
./build_image.sh

# 2. Run container
./run_container.sh

# 3. Inside container:
# Copy packages (assuming repo is mounted at /mnt/ros2_learning)
cp -r /mnt/ros2_learning/CPP/* ~/ros2_ws/src/
cp -r /mnt/ros2_learning/PYTHON/* ~/ros2_ws/src/

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source and run
source install/setup.bash
ros2 run parameters set_parameter_node
```

