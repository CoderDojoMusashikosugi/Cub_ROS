# localization.launch.py
# これをlaunch_at_boot.launch.pyと一緒に起動すると自己位置推定が出来る
# 自律走行はしない。
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
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("ekf_localizer"), "launch", "ekf_locali.launch.py"]
            ),
            condition=IfCondition("true" if cub_target == 'cub3' else "false")
        ),
        # IncludeLaunchDescription(
        #     PathJoinSubstitution(
        #         [FindPackageShare("mcub_bringup"), "launch", "localization.launch.py"]
        #     ),
        #     condition=IfCondition("true" if (cub_target == 'mcub' or cub_target == 'mcub_direct') else "false")
        # ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("ekf_localizer"), "launch", "ekf_locali.launch.py"]
            ),
            condition=IfCondition("true" if cub_target == 'spidar' else "false")
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("handy1_bringup"), "launch", "localization.launch.py"]
            ),
            condition=IfCondition("true" if cub_target == 'handy1' else "false")
        ),
    ])