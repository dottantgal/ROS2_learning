cmake_minimum_required(VERSION 3.5)
project(publisher_and_subscriber)

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
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/EmployeeSalary.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

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

add_executable(simple_subscriber_node src/simple_subscriber_node.cpp)
ament_target_dependencies(simple_subscriber_node rclcpp)

add_executable(simple_publisher_node src/simple_publisher_node.cpp)
ament_target_dependencies(simple_publisher_node rclcpp)

add_executable(simple_publisher_class_node src/simple_publisher_class_node.cpp)
ament_target_dependencies(simple_publisher_class_node rclcpp)

add_executable(simple_subscriber_class_node src/simple_subscriber_class_node.cpp)
ament_target_dependencies(simple_subscriber_class_node rclcpp)

add_executable(sub_pub_pipeline src/sub_pub_pipeline.cpp)
ament_target_dependencies(sub_pub_pipeline rclcpp)

add_executable(publish_custom_message src/publish_custom_message.cpp)
ament_target_dependencies(publish_custom_message rclcpp)

install(TARGETS
	 simple_subscriber_node
	 simple_publisher_node
	 simple_publisher_class_node
	 simple_subscriber_class_node
	 sub_pub_pipeline
	 publish_custom_message
   DESTINATION lib/${PROJECT_NAME}
 )

rosidl_target_interfaces(
	publish_custom_message
  ${PROJECT_NAME} rosidl_typesupport_cpp)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
