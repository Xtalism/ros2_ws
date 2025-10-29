cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(PROGRAM_NAMES
  pub_template
  sub_template
  sub_percentage
)

set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
)

foreach(PROGRAM_NAME ${PROGRAM_NAMES})
  add_executable(${PROGRAM_NAME} src/${PROGRAM_NAME}.cpp)
  ament_target_dependencies(${PROGRAM_NAME} ${DEPENDENCIES})
  install(TARGETS ${PROGRAM_NAME} DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
