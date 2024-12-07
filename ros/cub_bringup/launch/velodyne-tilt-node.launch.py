from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import PushRosNamespace
import ament_index_python.packages
import launch_ros.actions
import yaml
import os

def generate_launch_description():
    # Launchファイルの返り値
    return LaunchDescription([
        # correction point cloud tilt
        Node(
            package='cub_bringup',
            executable='tilt_correction_node',
            name='tilt_correction_node',
            parameters=[{
                'tilt_angle_degrees': 2.07
            }],
        ),

        # pointcloud to laser scan
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/velodyne_points_corrected'),
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
    ])