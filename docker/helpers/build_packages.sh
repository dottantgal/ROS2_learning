#!/bin/bash

# Helper script to build ROS 2 packages in workspace
# Usage: ./build_packages.sh [workspace_path] [packages]
#   workspace_path: Path to workspace (default: ~/ros2_ws)
#   packages: Space-separated list of packages to build (default: all)

set -e

WORKSPACE_PATH="${1:-$HOME/ros2_ws}"
PACKAGES="${@:2}"

if [ ! -d "${WORKSPACE_PATH}/src" ]; then
    echo "Error: Workspace '${WORKSPACE_PATH}' does not exist or has no src directory."
    exit 1
fi

# Source ROS 2
source /opt/ros/jazzy/setup.bash

cd "${WORKSPACE_PATH}"

echo "Building packages in: ${WORKSPACE_PATH}"
echo ""

if [ -z "${PACKAGES}" ]; then
    echo "Building all packages..."
    colcon build --symlink-install
else
    echo "Building packages: ${PACKAGES}"
    colcon build --packages-select ${PACKAGES} --symlink-install
fi

echo ""
echo "Build completed!"
echo ""
echo "To use the built packages, source:"
echo "  source ${WORKSPACE_PATH}/install/setup.bash"

