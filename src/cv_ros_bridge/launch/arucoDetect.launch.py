#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    package_name: str = "cv_ros_bridge"

    name_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="aruco_detect",
        default_value="aruco_detect",
        description="OpenCV and ROS aruco detection",
    )
    ld.add_action(name_arg)

    aruco_node: Node = Node(
        package=package_name,
        executable="arucoDetector",
        name=LaunchConfiguration("aruco_detect"),
        output="screen",
    )
    ld.add_action(aruco_node)

    rviz_node: Node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        namespace="/",
        name="rviz",
        arguments=["-d", "/home/xtal/ros2_ws/src/cv_ros_bridge/config/aruco_detect.rviz"],
        respawn=True,
    )
    ld.add_action(rviz_node)

    return ld
