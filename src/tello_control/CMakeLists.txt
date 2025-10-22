cmake_minimum_required(VERSION 3.5)
project(tello_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Try for OpenCV 4.X, but settle for whatever is installed
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif ()
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(CPP_PROGRAMS
  tello_control
  auto_twist
  circular_motion
)

set(DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  OpenCV
)

foreach(PROGRAM_NAME ${CPP_PROGRAMS})
  add_executable(${PROGRAM_NAME} src/${PROGRAM_NAME}.cpp)
  ament_target_dependencies(${PROGRAM_NAME} ${DEPENDENCIES})
  target_include_directories(${PROGRAM_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
  install(TARGETS ${PROGRAM_NAME} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
