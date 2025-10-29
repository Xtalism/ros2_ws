#!/home/xtal/ros2_venv/bin python

import subprocess

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, Shutdown
from launch_ros.actions import Node

SSID_OBJETIVO = "TELLO-9A0D42"
# SSID_OBJETIVO = "TELLO-A04B3A"
# SSID_OBJETIVO = "ConectaUACJ"
# SSID_OBJETIVO = "MotherBase"


def verificar_ssid(context, *args, **kwargs):
    try:
        resultado = subprocess.run(
            ["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        ssids_activos = [
            linea.split(":")[1]
            for linea in resultado.stdout.splitlines()
            if linea.startswith("yes:")
        ]

        if SSID_OBJETIVO in ssids_activos:
            return []
        else:
            return [
                LogInfo(
                    msg=f"SSID incorrecto. Se esperaba '{SSID_OBJETIVO}', pero se encontró: {ssids_activos}"
                ),
                Shutdown(reason="SSID no válido"),
            ]
    except Exception as e:
        return [
            LogInfo(msg=f"Error verificando SSID: {e}"),
            Shutdown(reason="Error en verificación"),
        ]


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    # Check if connected to Tello WiFi network
    ld.add_action(OpaqueFunction(function=verificar_ssid))

    tello_driver_node: Node = Node(
        package="tello",
        executable="tello",
        output="screen",
        namespace="/",
        name="tello",
        parameters=[
            {"connect_timeout": 10.0},
            {"tello_ip": "192.168.10.1"},
            {"tf_base": "map"},
            {"tf_drone": "drone"},
        ],
        remappings=[("/image_raw", "/camera"), ("/imu", "/imu/data_raw")],
        respawn=True,
    )
    ld.add_action(tello_driver_node)

    tello_control_node: Node = Node(
        package="tello_control",
        executable="tello_control",
        output="screen",
        namespace="/",
        name="control",
        respawn=False,
    )
    ld.add_action(tello_control_node)

    # auto_twist: Node = Node(
    #     package="tello_control",
    #     executable="auto_twist",
    #     output="screen",
    #     namespace="/",
    #     name="control",
    #     respawn=False,
    # )
    # ld.add_action(auto_twist)

    # circular_motion: Node = Node(
    #     package="tello_control",
    #     executable="circular_motion",
    #     output="screen",
    #     namespace="/",
    #     name="control",
    #     respawn=False,
    # )
    # ld.add_action(circular_motion)

    # simple_state: Node = Node(
    #     package="state_machine",
    #     executable="simple_state",
    #     output="screen",
    #     namespace="/",
    #     name="control",
    #     respawn=False,
    # )
    # ld.add_action(simple_state)

    # yasmin_dji_tello: Node = Node(
    #     package="state_machine",
    #     executable="yasmin_dji_tello",
    #     output="screen",
    #     namespace="/",
    #     name="control",
    #     respawn=False,
    # )
    # ld.add_action(yasmin_dji_tello)

    tello_tf_node: Node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace="/",
        name="tf",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            "map",
            "--child-frame-id",
            "drone",
        ],
        respawn=True,
    )
    ld.add_action(tello_tf_node)

    rviz_node: Node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        namespace="/",
        name="rviz",
        arguments=["-d", "/home/xtal/ros2_ws/src/tello_competition/rviz/tello.rviz"],
        respawn=True,
    )
    ld.add_action(rviz_node)

    imu_filter_node: Node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        output="screen",
        namespace="/",
        name="imu_filter",
        parameters=[{"use_mag": False}, {"use_gyro": True}, {"use_accel": True}],
        respawn=True,
    )
    ld.add_action(imu_filter_node)

    # ld.add_action(ExecuteProcess(cmd=["ros2", "bag", "record", "-a"], output="screen"))

    return ld
