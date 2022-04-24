cmake_minimum_required(VERSION 3.5)
project(service_server_and_client)

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
find_package(rclcpp REQUIRED)
find_package(custom_msg_and_srv REQUIRED)

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

add_executable(service_node src/service_node.cpp)
ament_target_dependencies(service_node rclcpp custom_msg_and_srv)

add_executable(client_node src/client_node.cpp)
ament_target_dependencies(client_node rclcpp custom_msg_and_srv)

add_executable(client_node_class src/client_node_class.cpp)
ament_target_dependencies(client_node_class rclcpp custom_msg_and_srv)

add_executable(service_node_class src/service_node_class.cpp)
ament_target_dependencies(service_node_class rclcpp custom_msg_and_srv)

install(TARGETS
	 service_node
	 client_node
	 client_node_class
	 service_node_class
   DESTINATION lib/${PROJECT_NAME}
 )

ament_export_dependencies(rosidl_default_runtime)
ament_package()
