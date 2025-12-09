# ROS 2 Jazzy Minimal Development Image
# Base: ros:jazzy-ros-base (minimal, no GUI)
FROM ros:jazzy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install build tools and essential dependencies
RUN apt-get update && apt-get install -y \
    # Build tools
    python3-colcon-common-extensions \
    python3-vcstool \
    # C++ compiler and tools (C++17 support)
    build-essential \
    cmake \
    # Boost libraries (required by some packages)
    libboost-dev \
    # Python development
    python3-pip \
    python3-dev \
    # ROS 2 core dependencies (pre-install common ones)
    ros-jazzy-rclcpp \
    ros-jazzy-rclpy \
    ros-jazzy-std-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-message-filters \
    ros-jazzy-pluginlib \
    ros-jazzy-rclcpp-action \
    ros-jazzy-rclcpp-components \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-ament-cmake \
    # Development utilities
    git \
    vim \
    nano \
    bash-completion \
    # Cleanup
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2 setup in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Create workspace directory structure
RUN mkdir -p /root/ros2_ws/src /root/ros2_ws/helpers

# Copy helper scripts into container
COPY helpers/ /root/ros2_ws/helpers/
RUN chmod +x /root/ros2_ws/helpers/*.sh

# Set working directory
WORKDIR /root/ros2_ws

# Default command
CMD ["/bin/bash"]

