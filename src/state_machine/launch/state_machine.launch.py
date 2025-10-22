#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    package_name: str = "state_machine"

    name_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="state_machine",
        default_value="state_machine",
        description="YASMIN",
    )
    ld.add_action(name_arg)

    yasmin_viewer_node: Node = Node(
        package=package_name,
        executable="yasmin_viewer_node",
        name=LaunchConfiguration("state_machine"),
        output="screen",
    )
    ld.add_action(yasmin_viewer_node)

    yasmin_dji_tello: Node = Node(
        package=package_name,
        executable="yasmin_dji_tello",
        name=LaunchConfiguration("state_machine"),
        output="screen",
    )
    ld.add_action(yasmin_dji_tello)

    # simple_state: Node = Node(
    #     package=package_name,
    #     executable="simple_state",
    #     name=LaunchConfiguration("state_machine"),
    #     output="screen",
    # )
    # ld.add_action(simple_state)

    return ld
