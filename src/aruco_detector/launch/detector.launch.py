from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os

def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument('camera', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('tf_prefix', default_value='marker_id'),
        DeclareLaunchArgument('show_detections', default_value='true'),
        DeclareLaunchArgument('marker_size', default_value='0.05'),
        DeclareLaunchArgument('dictionary_name', default_value='DICT_4X4_1000'),
        DeclareLaunchArgument('blur_window_size', default_value='7'),
        DeclareLaunchArgument('num_detected', default_value='25'),
        DeclareLaunchArgument('min_prec_value', default_value='50'),
    ]

    # Path to your RViz config file
    rviz_config_path = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'aruco_opencv.rviz'
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_d435',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': False,
            'enable_infra1': False,
            'enable_infra2': False,
            'color_width': 640,
            'color_height': 480,
            'color_fps': 30,
        }]
    )

    aruco_node = Node(
        package='aruco_detector',
        executable='marker_filter.py',
        name='marker_filter',
        output='screen',
        parameters=[{
            'parent_frame_id': 'camera_link',
            'marker_id': 'marker0',
            'marker_filtered_id': 'marker0_filtered',
            'alpha': 0.9,
            'rate_value': 125,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='markers_extrinsic',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription(args + [
        realsense_node,
        aruco_node,
        rviz_node,
    ])