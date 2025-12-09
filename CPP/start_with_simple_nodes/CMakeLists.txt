cmake_minimum_required(VERSION 3.8)
project(start_with_simple_nodes)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 (Jazzy recommendation)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(my_first_node src/my_first_node.cpp)
ament_target_dependencies(my_first_node rclcpp)

add_executable(node_with_class src/node_with_class.cpp)
ament_target_dependencies(node_with_class rclcpp)

add_executable(node_timer_with_class src/node_timer_with_class.cpp)
ament_target_dependencies(node_timer_with_class rclcpp)

add_executable(node_timer_without_class src/node_timer_without_class.cpp)
ament_target_dependencies(node_timer_without_class rclcpp)

install(TARGETS
   my_first_node
   node_with_class
   node_timer_with_class
   node_timer_without_class
   DESTINATION lib/${PROJECT_NAME}
 )

ament_export_dependencies(rosidl_default_runtime)
ament_package()
