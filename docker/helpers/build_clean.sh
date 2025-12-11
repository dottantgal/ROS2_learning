#!/bin/bash

# Helper script to build workspace with clean environment (suppresses AMENT_PREFIX_PATH warnings)
# Usage: ./build_clean.sh [colcon_args...]

set -e

# Unset prefix paths to avoid warnings about non-existent install directories
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH

# Run colcon build with provided arguments
colcon build "$@"

