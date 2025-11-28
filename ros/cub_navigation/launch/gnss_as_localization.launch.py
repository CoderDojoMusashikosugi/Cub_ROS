# GNSS-based Localization Launch File
# This launch file provides map->base_link TF using GNSS and odometry fusion
#
# Requirements:
# - GNSS receiver publishing to /fix (sensor_msgs/NavSatFix)
# - Odometry published to /odom (nav_msgs/Odometry)
# - Map origin coordinates must be set in gnss_fusion.yaml
#
# Usage:
#   ros2 launch cub_navigation gnss_as_localization.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get parameter file path
    gnss_fusion_params = os.path.join(
        get_package_share_directory('cub_bringup'),
        'params',
        'gnss_fusion.yaml'
    )

    return LaunchDescription([
        # GNSS-Odom Fusion Node:
        # - Converts GNSS (lat/lon) to local map coordinates (x/y)
        # - Fuses GNSS position with odometry orientation
        # - Publishes map->base_link TF
        Node(
            package='cub_bringup',
            executable='gnss_odom_fusion',
            name='gnss_odom_fusion',
            parameters=[gnss_fusion_params],
            output='screen',
            remappings=[
                # Input: /fix (sensor_msgs/NavSatFix)
                # Input: /odom (nav_msgs/Odometry)
                # Output: /gps_pose (geometry_msgs/PoseWithCovarianceStamped) - GPS position before fusion
                # Output: /gnss_pose (geometry_msgs/PoseStamped) - Final fused pose for debugging
                # Output: map->base_link TF
            ]
        ),

        # Static TF: map->odom (identity transform)
        # Since we directly publish map->base_link, map and odom frames are identical
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
            output='screen'
        ),
    ])
