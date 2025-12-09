#!/bin/bash

# Helper script to set up a ROS 2 workspace in the container
# Usage: ./setup_workspace.sh [workspace_path]
# Default: ~/ros2_ws

set -e

WORKSPACE_PATH="${1:-$HOME/ros2_ws}"

echo "Setting up ROS 2 workspace at: ${WORKSPACE_PATH}"

# Create workspace structure
mkdir -p "${WORKSPACE_PATH}/src"

# Source ROS 2 (if not already sourced)
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash
fi

echo "Workspace created at: ${WORKSPACE_PATH}"
echo ""
echo "To use this workspace:"
echo "  1. Copy your packages to: ${WORKSPACE_PATH}/src"
echo "  2. Install dependencies: cd ${WORKSPACE_PATH} && rosdep install --from-paths src --ignore-src -r -y"
echo "  3. Build: cd ${WORKSPACE_PATH} && colcon build"
echo "  4. Source: source ${WORKSPACE_PATH}/install/setup.bash"

