#!/usr/bin/env python3
# ros2 service call /save_map std_srvs/srv/Trigger
# で地図保存

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('cub_mapping')

    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'params', 'map_converter.yaml'),
        description='Path to the parameters file for map converter node'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    pcd_file_mapping_arg = DeclareLaunchArgument(
        'pcd_file_mapping',
        default_value='/home/cub/maps/map_tc25_gnss_0_2.pcd',
        description='Path to high-density PCD file for mapping'
    )

    pcd_file_display_arg = DeclareLaunchArgument(
        'pcd_file_display',
        default_value='/home/cub/maps/map_tc25_gnss_0_3.pcd',
        description='Path to lightweight PCD file for RViz display'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'rviz', 'map_converter.rviz'),
        description='Path to RViz config file'
    )

    # Launch map converter node
    map_converter_node = Node(
        package='cub_mapping',
        executable='map_converter_node',
        name='map_converter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True
    )

    # Launch PCD publisher for mapping (high-density)
    pcd_publisher_mapping_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='pcd_publisher_mapping',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'file_name': LaunchConfiguration('pcd_file_mapping')},
            {'tf_frame': 'map'}
        ],
        remappings=[
            ('cloud_pcd', 'cloud_pcd_mapping')
        ]
    )

    # Launch PCD publisher for display (lightweight)
    pcd_publisher_display_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='pcd_publisher_display',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'file_name': LaunchConfiguration('pcd_file_display')},
            {'tf_frame': 'map'}
        ],
        remappings=[
            ('cloud_pcd', 'cloud_pcd_display')
        ]
    )

    # Launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        pcd_file_mapping_arg,
        pcd_file_display_arg,
        rviz_config_arg,
        map_converter_node,
        pcd_publisher_mapping_node,
        pcd_publisher_display_node,
        rviz_node,
    ])
