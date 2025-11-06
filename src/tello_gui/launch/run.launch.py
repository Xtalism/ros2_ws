import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch = LaunchDescription()
    urdf_file = os.path.join(
        get_package_share_directory('tello_gui'),
        'models', 'tello', 'model.urdf'
    )

    sdf_path = PathJoinSubstitution([
        FindPackageShare('tello_gui'),
        'models',
        'tello',
        'model.sdf'
    ])
    urdf_path = PathJoinSubstitution([
        FindPackageShare('tello_gui'),
        'models',
        'tello',
        'model.urdf'
    ])

    launch.add_action(DeclareLaunchArgument(
        name='jsp_gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui'
    ))
    rviz_path = PathJoinSubstitution([
        FindPackageShare('tello_gui'),
        'rviz',
        'view.rviz'
    ])
    launch.add_action(DeclareLaunchArgument(
        name='rviz_config',
        default_value=rviz_path,
        description='Full path to the RVIZ config file to use'
     ))

    launch.add_action(DeclareLaunchArgument(
        name='model',
        default_value=urdf_path,
        description='Absolute path to robot urdf file'
    ))

    launch.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('urdf_launch'),
            'launch',
            'description.launch.py'
        ]),
        launch_arguments={
            'urdf_package': 'tello_gui',
            'urdf_package_path': LaunchConfiguration('model'),
            }.items()
        ))


    launch.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('jsp_gui'))
    ))

    launch.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    ))

    launch.add_action(ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', 'empty.sdf'],
        output='screen'
    ))

    launch.add_action(Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tello',
            '-x', '0', '-y', '0', '-z', '0',
            '-Y', '0', '-R', '0', '-P', '0',
            '-file', sdf_path
        ],
        output='screen'
    ))
    
    launch.add_action(Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/takeoff@std_msgs/msg/Empty@gz.msgs.Empty",
            "/land@std_msgs/msg/Empty@gz.msgs.Empty",
            "/emergency@std_msgs/msg/Empty@gz.msgs.Empty",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            # "/robot_description@std_msgs/msgs/String@gz.msgs.StringMsg"
            "/flip@std_msgs/msg/String@gz.msgs.StringMsg",
            "/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/control@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/initialpose@geometry_msgs/msg/PoseWithCovarianceStamped@gz.msgs.PoseWithCovariance",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    ))

    return launch
