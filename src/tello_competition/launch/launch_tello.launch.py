#!/home/xtal/ros2_venv/bin python

import subprocess

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, Shutdown
from launch_ros.actions import Node

# TARGET_SSID = "TELLO-9A0D42"
# TARGET_SSID = "TELLO-A04B3A"
# TARGET_SSID = "Totalplay-5G-5970"
TARGET_SSID = "TELLO-5C8A2D"
# TARGET_SSID = "ConectaUACJ"
# TARGET_SSID = "MotherBase"


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

        if TARGET_SSID in ssids_activos:
            return []
        else:
            return [
                LogInfo(
                    msg=f"SSID incorrecto. Se esperaba '{TARGET_SSID}', pero se encontró: {ssids_activos}"
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
        arguments=["-d", "/home/xtal/ros2_ws/src/tello_competition/config/tello.rviz"],
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

    orb_slam3 = Node(
        package="ros2_orb_slam3",
        executable="tello_slam_cpp",
        output="screen",
        name="orb_slam3",
        parameters=[
            {"node_name_arg": "orb_slam3"},
            {"config_name": "dji_tello_slam"},  # Name of your YAML config
        ],
    )
    ld.add_action(orb_slam3)
    
    # map_saver_node = Node(
    #     package="tello_competition",
    #     executable="save_slam_map",
    #     output="screen",
    #     name="map_server",
    #     parameters=[
    #         {"save_directory": "/home/xtal/ros2_ws/slam_maps/tello_maps"},
    #         {"auto_save_interval": 30.0},
    #         {"min_points": 100}
    #     ],
    # )
    # ld.add_action(map_saver_node)
    

    # ld.add_action(ExecuteProcess(cmd=["ros2", "bag", "record", "-a"], output="screen"))

    return ld