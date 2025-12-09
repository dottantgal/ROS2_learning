#!/bin/bash

# Script to build ROS 2 Jazzy Docker image for learning packages
# Usage: ./build_image.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="ros2-jazzy-learning"
IMAGE_TAG="latest"

echo "=========================================="
echo "Building ROS 2 Jazzy Learning Docker Image"
echo "=========================================="
echo "Image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo ""

cd "${SCRIPT_DIR}"

# Build the Docker image
docker build -t "${IMAGE_NAME}:${IMAGE_TAG}" .

echo ""
echo "=========================================="
echo "Build completed successfully!"
echo "=========================================="
echo "Image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo ""
echo "To run the container, use:"
echo "  ./run_container.sh"
echo ""

