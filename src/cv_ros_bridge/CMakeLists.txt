cmake_minimum_required(VERSION 3.8)
project(cv_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)

add_executable(cv_ros_bridge_talker_cpp src/cv_ros_bridge_talker.cpp)
ament_target_dependencies(cv_ros_bridge_talker_cpp rclcpp sensor_msgs cv_bridge std_msgs image_transport)

install(TARGETS
  cv_ros_bridge_talker_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

set(PYTHON_SCRIPTS
  cv_ros_bridge
  cv_ros_bridge_talker
  cv_ros_bridge_listener
  imagePublisher
  imageSplitter
  imageThreshold
  imageErotion
  imageDilation
  edgeDetection
  lineDetection
  sobelDerivative
  arucoDetector
  arucoPub
)

foreach(script ${PYTHON_SCRIPTS})
  install(PROGRAMS ${PROJECT_NAME}/${script}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
