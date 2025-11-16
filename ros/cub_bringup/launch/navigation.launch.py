# navigation.launch.py
# これをlaunch_at_boot.launch.pyと一緒に起動すると自律走行をする。
# 自己位置推定もする。
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    cub_target = os.getenv('CUB_TARGET', 'cub3')
    print("launch target:", cub_target)
    map_dir = LaunchConfiguration(
        'map',
        default = "/home/cub/maps/oudanhodoumade/mapoudanhodoumade_manual_crean.yaml")

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_bringup"), "launch", "localization.launch.py"] # ここでlocalizationを起動
            )
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_navigation"), "launch", "cub_navigation.launch.py"]
            ),
            launch_arguments=[
                ('map', map_dir)
            ],
        ),
    ])