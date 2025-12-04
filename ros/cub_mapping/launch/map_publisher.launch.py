#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('cub_mapping')

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/home/cub/maps/furo_kakunin/map.yaml',
        description='Path to 2D map YAML file for nav2 map_server (empty = no map)'
    )

    pcd_file_arg = DeclareLaunchArgument(
        'pcd_file',
        default_value='/home/cub/maps/map_tc25_gnss_0_3.pcd',
        description='Optional: Path to PCD file for 3D visualization'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': LaunchConfiguration('map_file')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Lifecycle manager for map_server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['map_server']},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # PCD publisher (for 3D point cloud)
    pcd_publisher_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='pcd_publisher_waypoint',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'file_name': LaunchConfiguration('pcd_file')},
            {'tf_frame': 'map'}
        ],
    )

    return LaunchDescription([
        map_file_arg,
        pcd_file_arg,
        use_sim_time_arg,
        map_server_node,
        lifecycle_manager_node,
        pcd_publisher_node,
    ])
