#!/usr/bin/env python3

# 地図の保存には ros2 service call /save_waypoints std_srvs/srv/Trigger
# グループ作成には ros2 service call /create_group cub_mapping/srv/CreateGroup '{group_name: \"Test Path\", require_input: false}'

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('cub_mapping')

    # Launch arguments
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value='/home/cub/colcon_ws/src/cub/cub_behavior_tree/routes/furo_waypoints.yaml',
        description='Path to waypoint YAML file to load (empty = no auto-load)'
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/home/cub/maps/20251126_123706/map.yaml',
        description='Path to 2D map YAML file for nav2 map_server (empty = no map)'
    )

    pcd_file_arg = DeclareLaunchArgument(
        'pcd_file',
        default_value='/home/cub/maps/map_tc25_gnss_0_3.pcd',
        description='Optional: Path to PCD file for 3D visualization'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'params', 'waypoint_editor.yaml'),
        description='Path to parameters file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'rviz', 'waypoint_editor.rviz'),
        description='Path to RViz config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Waypoint Manager Node (backend)
    waypoint_manager_node = Node(
        package='cub_mapping',
        executable='waypoint_manager_node',
        name='waypoint_manager_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'waypoint_file': LaunchConfiguration('waypoint_file'),
            }
        ],
        emulate_tty=True
    )

    # Map Server (for 2D map)
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
        remappings=[
            ('cloud_pcd', 'cloud_pcd_waypoint')
        ]
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        waypoint_file_arg,
        map_file_arg,
        pcd_file_arg,
        params_file_arg,
        rviz_config_arg,
        use_sim_time_arg,
        waypoint_manager_node,
        map_server_node,
        lifecycle_manager_node,
        pcd_publisher_node,
        rviz_node,
    ])
