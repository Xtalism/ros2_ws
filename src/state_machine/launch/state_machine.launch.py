#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    viewer_detached = ExecuteProcess(
        cmd=['bash', '-lc', 'nohup setsid ros2 run state_machine yasmin_viewer_node.py > /tmp/yasmin_viewer.log 2>&1 &'],
        shell=False,
    )
    ld.add_action(viewer_detached)

    package_name: str = "state_machine"

    name_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="state_machine",
        default_value="state_machine",
        description="YASMIN",
    )
    ld.add_action(name_arg)

    yasmin_dji_tello: Node = Node(
        package=package_name,
        executable="yasmin_dji_tello",
        name=LaunchConfiguration("state_machine"),
        output="screen",
        emulate_tty=True,
    )
    ld.add_action(yasmin_dji_tello)

    # simple_state: Node = Node(
    #     package=package_name,
    #     executable="simple_state",
    #     name=LaunchConfiguration("state_machine"),
    #     output="screen",
    #     emulate_tty=True,
    # )
    # ld.add_action(simple_state)

    return ld