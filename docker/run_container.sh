#!/bin/bash

# Script to run ROS 2 Jazzy Docker container
# Usage: ./run_container.sh [container_name]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="ros2-jazzy-learning"
IMAGE_TAG="latest"
CONTAINER_NAME="${1:-ros2-jazzy-ws}"

# Check if image exists
if ! docker image inspect "${IMAGE_NAME}:${IMAGE_TAG}" > /dev/null 2>&1; then
    echo "Error: Docker image '${IMAGE_NAME}:${IMAGE_TAG}' not found."
    echo "Please run './build_image.sh' first to build the image."
    exit 1
fi

# Check if container already exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container '${CONTAINER_NAME}' already exists."
    read -p "Do you want to start the existing container? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "Container is already running. Attaching to it..."
            docker exec -it "${CONTAINER_NAME}" /bin/bash
        else
            echo "Starting existing container..."
            docker start "${CONTAINER_NAME}"
            docker exec -it "${CONTAINER_NAME}" /bin/bash
        fi
        exit 0
    else
        echo "Aborted."
        exit 1
    fi
fi

echo "=========================================="
echo "Starting ROS 2 Jazzy Learning Container"
echo "=========================================="
echo "Container name: ${CONTAINER_NAME}"
echo "Image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo ""

# Check if ROS2_learning repository path is provided as environment variable
REPO_PATH="${ROS2_LEARNING_REPO:-}"
MOUNT_OPTION=""

if [ -n "${REPO_PATH}" ] && [ -d "${REPO_PATH}" ]; then
    MOUNT_OPTION="-v ${REPO_PATH}:/mnt/ros2_learning:ro"
    echo "Repository will be mounted at: /mnt/ros2_learning (read-only)"
    echo ""
fi

# Create a named volume for workspace persistence
VOLUME_NAME="${CONTAINER_NAME}-workspace"

# Run the container
docker run -it \
    --name "${CONTAINER_NAME}" \
    --hostname ros2-jazzy-dev \
    -v "${VOLUME_NAME}:/root/ros2_ws" \
    ${MOUNT_OPTION} \
    "${IMAGE_NAME}:${IMAGE_TAG}"

echo ""
echo "Container stopped. To restart it, run:"
echo "  docker start ${CONTAINER_NAME}"
echo "  docker exec -it ${CONTAINER_NAME} /bin/bash"
echo ""
echo "Tip: To mount the repository, set ROS2_LEARNING_REPO environment variable:"
echo "  export ROS2_LEARNING_REPO=/path/to/ROS2_learning"
echo "  ./run_container.sh"

