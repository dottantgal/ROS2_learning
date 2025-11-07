cmake_minimum_required(VERSION 3.8)
project(custom_msg_and_srv_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_python_install_package(${PROJECT_NAME})

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CapitalFullName.srv"
)

install(PROGRAMS
  ${PROJECT_NAME}/capital_full_name_server.py
  ${PROJECT_NAME}/capital_full_name_client.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  set(TEST_SCRIPTS
    test/test_capital_full_name.py
  )
  foreach(test_script ${TEST_SCRIPTS})
    ament_add_pytest_test(${test_script}_pytest ${test_script})
  endforeach()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
