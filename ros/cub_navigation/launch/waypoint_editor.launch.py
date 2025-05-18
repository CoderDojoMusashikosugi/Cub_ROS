# 使い方
#  ros2 launch cub_navigation waypoint_editor.launch.py  map:=/home/cub/maps/map1/map.yaml
#  のようにして起動する。少し待つと地図が読み込まれる。
# ウェイポイントを追加するには
#  RVizでGキーを押して(RVizの2D Pose Estimateを押して)地図上に矢印を設置すると、~/wp.yamlの後ろに追記される。
#  あとはそれを実際のWPにコピペして整形する
# ウェイポイントを削除するには
#  wp.yamlから消したいWPを削除して、RVizのPublish Pointを押したら画面上に反映される。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    map_yaml_file = LaunchConfiguration('map')
    
    rviz_config_dir = os.path.join(
        get_package_share_directory('cub_navigation'),
        'param',
        'create_waypoints.rviz')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn_delay=2.0,
            parameters=[{'yaml_filename': map_yaml_file}],
        ),#ros2 lifecycle set map_server configure && ros2 lifecycle set map_server activate
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),

        TimerAction(period=1.0, actions=[Node(
            package='cub_navigation',
            executable='waypoint_manager',
            name='waypoint_manager',
            output='screen',
        )]),


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0','1','map','base_link']
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'), #press g key to publish goal pose
    ])