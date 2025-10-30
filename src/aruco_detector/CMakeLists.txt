cmake_minimum_required(VERSION 3.8)
project(aruco_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED aruco)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoInfo.msg"
  "msg/KfState.msg"
  "msg/RobotCommand.msg"
  "msg/RobotState.msg"
  "msg/SensorState.msg"
  "msg/Sound.msg"
  "msg/TransformStampedCertainty.msg"
  "msg/VersionInfo.msg"
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(${PROJECT_NAME}_node src/aruco_detector_ocv.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS} "${cpp_typesupport_target}")

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  cv_bridge
  image_geometry
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  image_transport
  OpenCV
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python scripts
set(PYTHON_SCRIPTS
  marker_filter.py
)

foreach(script ${PYTHON_SCRIPTS})
  install(PROGRAMS ${PROJECT_NAME}/${script}
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

# Install launch directory if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ament lint auto
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
