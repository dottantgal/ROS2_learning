#!/bin/bash

# Helper script to copy ROS 2 learning packages into workspace
# Usage: ./copy_packages.sh [source_path] [workspace_path] [package_type]
#   source_path: Path to ROS2_learning repository (default: /tmp/ros2_learning)
#   workspace_path: Target workspace (default: ~/ros2_ws)
#   package_type: 'cpp', 'python', or 'all' (default: 'all')

set -e

SOURCE_PATH="${1:-/tmp/ros2_learning}"
WORKSPACE_PATH="${2:-$HOME/ros2_ws}"
PACKAGE_TYPE="${3:-all}"

if [ ! -d "${SOURCE_PATH}" ]; then
    echo "Error: Source directory '${SOURCE_PATH}' not found."
    echo "Please mount or copy the ROS2_learning repository first."
    exit 1
fi

# Create workspace if it doesn't exist
mkdir -p "${WORKSPACE_PATH}/src"

echo "Copying packages from: ${SOURCE_PATH}"
echo "To workspace: ${WORKSPACE_PATH}/src"
echo "Package type: ${PACKAGE_TYPE}"
echo ""

case "${PACKAGE_TYPE}" in
    cpp)
        if [ -d "${SOURCE_PATH}/CPP" ]; then
            cp -r "${SOURCE_PATH}/CPP"/* "${WORKSPACE_PATH}/src/"
            echo "✓ C++ packages copied"
        else
            echo "Warning: CPP directory not found in source"
        fi
        ;;
    python)
        if [ -d "${SOURCE_PATH}/PYTHON" ]; then
            cp -r "${SOURCE_PATH}/PYTHON"/* "${WORKSPACE_PATH}/src/"
            echo "✓ Python packages copied"
        else
            echo "Warning: PYTHON directory not found in source"
        fi
        ;;
    all)
        if [ -d "${SOURCE_PATH}/CPP" ]; then
            cp -r "${SOURCE_PATH}/CPP"/* "${WORKSPACE_PATH}/src/"
            echo "✓ C++ packages copied"
        fi
        if [ -d "${SOURCE_PATH}/PYTHON" ]; then
            cp -r "${SOURCE_PATH}/PYTHON"/* "${WORKSPACE_PATH}/src/"
            echo "✓ Python packages copied"
        fi
        ;;
    *)
        echo "Error: Invalid package type '${PACKAGE_TYPE}'. Use 'cpp', 'python', or 'all'"
        exit 1
        ;;
esac

echo ""
echo "Packages copied successfully!"
echo ""
echo "Next steps:"
echo "  1. Install dependencies: cd ${WORKSPACE_PATH} && rosdep install --from-paths src --ignore-src -r -y"
echo "  2. Build: cd ${WORKSPACE_PATH} && colcon build"
echo "  3. Source: source ${WORKSPACE_PATH}/install/setup.bash"

