# navigation.launch.py
# これをlaunch_at_boot.launch.pyと一緒に起動すると自律走行をする。
# 自己位置推定もする。
# デフォルト起動順序: RViz (0s) -> EKF (3s) -> Nav2 (8s)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    cub_target = os.getenv('CUB_TARGET', 'cub4')
    print("launch target:", cub_target)
    map_dir = LaunchConfiguration(
        'map',
        default = "/home/cub/colcon_ws/src/cub/cub_navigation/maps/tc26_1017/map.yaml")
        # default = "/home/cub/maps/map_shiminkan_20261114_115056/map.yaml")
        # default = "/home/cub/colcon_ws/src/cub/cub_navigation/maps/empty/map.yaml")

    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='none', choices=['none', 'godot']),
        DeclareLaunchArgument('rviz_delay', default_value='0.0'),
        DeclareLaunchArgument('ekf_delay', default_value='3.0'),
        DeclareLaunchArgument('nav2_delay', default_value='8.0'),
        
        # ekf_delay 秒後に localization (EKF) を起動
        TimerAction(
            period=LaunchConfiguration('ekf_delay'),
            actions=[
                IncludeLaunchDescription(
                    PathJoinSubstitution(
                        [FindPackageShare("cub_bringup"), "launch", "localization.launch.py"] # ここでlocalizationを起動
                    ),
                    launch_arguments=[
                        ('map_localization', map_dir),
                        ('simulation', LaunchConfiguration('simulation'))
                    ],
                ),
            ],
        ),

        # RViz (rviz_delay 秒後) と Nav2 (nav2_delay 秒後) を起動
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_navigation"), "launch", "cub_navigation.launch.py"]
            ),
            launch_arguments=[
                ('map', map_dir),
                ('simulation', LaunchConfiguration('simulation')),
                ('rviz_delay', LaunchConfiguration('rviz_delay')),
                ('nav2_delay', LaunchConfiguration('nav2_delay')),
            ],
        ),

    ])