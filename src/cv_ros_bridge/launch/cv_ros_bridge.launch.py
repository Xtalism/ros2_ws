#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    package_name: str = "cv_ros_bridge"

    name_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="cv_ros",
        default_value="cv_ros_bridge_node_publisher",
        description="OpenCV and ROS bridge publisher nodes",
    )
    ld.add_action(name_arg)

    image_publisher_node: Node = Node(
        package=package_name,
        executable="image_publisher",
        name=LaunchConfiguration("cv_ros"),
        output="screen",
    )
    ld.add_action(image_publisher_node)

    image_splitter: Node = Node(
        package=package_name,
        executable="image_splitter",
        name="image_splitter",
        output="screen",
    )
    ld.add_action(image_splitter)

    image_threshold: Node = Node(
        package=package_name,
        executable="image_threshold",
        name="image_threshold",
        output="screen",
    )
    ld.add_action(image_threshold)

    image_erotion: Node = Node(
        package=package_name,
        executable="image_erotion",
        name="image_erotion",
        output="screen",
    )
    ld.add_action(image_erotion)

    image_dilation: Node = Node(
        package=package_name,
        executable="image_dilation",
        name="image_dilation",
        output="screen",
    )
    ld.add_action(image_dilation)

    edge_detection: Node = Node(
        package=package_name,
        executable="edge_detection",
        name="edge_detection",
        output="screen",
    )
    ld.add_action(edge_detection)

    line_detection: Node = Node(
        package=package_name,
        executable="line_detection",
        name="line_detection",
        output="screen",
    )
    ld.add_action(line_detection)

    sobel_derivative: Node = Node(
        package=package_name,
        executable="sobel_derivative",
        name="sobel_derivative",
        output="screen",
    )
    ld.add_action(sobel_derivative)

    return ld
