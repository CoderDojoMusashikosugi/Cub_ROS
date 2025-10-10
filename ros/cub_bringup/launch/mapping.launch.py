# mapping.launch.py
# これをlaunch_at_boot.launch.pyと一緒に起動すると地図作成をする
# これmcub専用か？
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os


def generate_launch_description():
    cub_target = os.getenv('CUB_TARGET', 'cub3')
    print("launch target:", cub_target)

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_bringup"), "launch", "common.launch.py"]
            ),
            condition=IfCondition("true" if cub_target == 'cub3' else "false")
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_bringup"), "launch", "3d_mapping.launch.py"]
            ),
            condition=IfCondition("true" if cub_target == 'cub3' else "false")
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_bringup"), "launch", "2d_mapping.launch.py"]
            ),
            condition=IfCondition("true" if (cub_target == 'mcub' or cub_target == 'mcub_direct') else "false")
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_bringup"), "launch", "3d_mapping.launch.py"]
            ),
            condition=IfCondition("true" if cub_target == 'spidar' else "false")
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("handy1_bringup"), "launch", "mapping_mid360.launch.py"]
            ),
            condition=IfCondition("true" if cub_target == 'handy1' else "false")
        ),
    ])