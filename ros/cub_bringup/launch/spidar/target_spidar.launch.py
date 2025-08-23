from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import PushRosNamespace
import os

def generate_launch_description():
    
    urdf_file_name = 'spidar.urdf'
    urdf = os.path.join(
        get_package_share_directory('cub_description'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Launchファイルの返り値
    return LaunchDescription([
        # wheel_odometryノード
        # Node(
        #     package='wheel_odometry',
        #     executable='wheel_odometry_node',
        #     name='wheel_odometry_node',
        #     output='screen'
        # ),

        # pointcloud to laser scan
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/points_raw'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': '',
                'transform_tolerance': 0.01,
                'min_height': -0.35,
                'max_height': 1.5,
                'angle_min': -3.14,  # -M_PI
                'angle_max': 3.14,  # M_PI
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.1,
                'range_max': 100.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        ),

        # 2Dのマッピングやナビゲーションを実行する際はこれを有効化、3Dでは無効化
        Node(
            package='cub_bringup',
            executable='odom_to_tf',
        ),

        # RVizの起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("cub_visualization"), "launch", "rviz.launch.py"]
                )
            )
        ),

        # state_publisherの起動
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),
    ])
