cmake_minimum_required(VERSION 3.8)
project(state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yasmin_viewer REQUIRED)
find_package(yasmin_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(PYTHON_SCRIPTS
  yasmin_viewer_node
)

set(CPP_PROGRAMS
  publisher_demo
  yasmin_dji_tello
  concurrence_demo
  simple_state
)

set(DEPENDENCIES
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  example_interfaces
  yasmin
  yasmin_ros
  yasmin_viewer
  yasmin_msgs
)

foreach(PYTHON_SCRIPT ${PYTHON_SCRIPTS})
  install(PROGRAMS ${PROJECT_NAME}/${PYTHON_SCRIPT}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${PYTHON_SCRIPT})
endforeach()

foreach(PROGRAM_NAME ${CPP_PROGRAMS})
  add_executable(${PROGRAM_NAME} src/${PROGRAM_NAME}.cpp)
  ament_target_dependencies(${PROGRAM_NAME} ${DEPENDENCIES})
  install(TARGETS ${PROGRAM_NAME} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
