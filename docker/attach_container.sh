#!/bin/bash

# Script to attach/open a new shell in an existing ROS 2 Jazzy container
# Usage: ./attach_container.sh [container_name]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_NAME="${1:-ros2-jazzy-ws}"

# Check if container exists
if ! docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' does not exist."
    echo "Please run './run_container.sh' first to create the container."
    exit 1
fi

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container '${CONTAINER_NAME}' exists but is not running."
    read -p "Do you want to start it? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Starting container..."
        docker start "${CONTAINER_NAME}"
    else
        echo "Aborted."
        exit 1
    fi
fi

echo "Attaching to container '${CONTAINER_NAME}'..."
echo ""

# Attach a new shell to the running container
docker exec -it "${CONTAINER_NAME}" /bin/bash

