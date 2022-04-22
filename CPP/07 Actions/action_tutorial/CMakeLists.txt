cmake_minimum_required(VERSION 3.5)
project(action_tutorial)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(custom_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(simple_action_server src/simple_action_server.cpp)
ament_target_dependencies(simple_action_server
  "rclcpp"
  "rclcpp_action"
  "custom_action")

add_executable(class_action_server src/class_action_server.cpp)
ament_target_dependencies(class_action_server
  "rclcpp"
  "rclcpp_action"
  "custom_action")

add_executable(simple_action_client src/simple_action_client.cpp)
ament_target_dependencies(simple_action_client
  "rclcpp"
  "rclcpp_action"
  "custom_action")

add_executable(class_action_client src/class_action_client.cpp)
ament_target_dependencies(class_action_client
  "rclcpp"
  "rclcpp_action"
  "custom_action")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  simple_action_server
	class_action_server
	simple_action_client
  class_action_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()