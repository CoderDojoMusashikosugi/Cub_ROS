# GNSS-based Localization Launch File
# This launch file provides map->odom->base_link TF chain using GNSS and odometry fusion
#
# Requirements:
# - GNSS receiver publishing to /fix (sensor_msgs/NavSatFix)
# - Odometry published to /odom (nav_msgs/Odometry)
# - Map origin coordinates must be set in gnss_fusion.yaml
#
# TF Structure:
# - map->odom: Published by gnss_odom_fusion (GNSS position correction)
# - odom->base_link: Published by odom_to_tf (odometry data)
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
        # - Publishes map->odom TF
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
                # Output: map->odom TF
            ]
        ),

        # Odom to TF Node:
        # - Publishes odom->base_link TF from odometry data
        Node(
            package='cub_bringup',
            executable='odom_to_tf',
            name='odom_to_tf',
            output='screen'
        ),
    ])
