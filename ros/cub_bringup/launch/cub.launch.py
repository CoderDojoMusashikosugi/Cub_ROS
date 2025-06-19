from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os

def generate_launch_description():
    cub_target = os.getenv('CUB_TARGET', 'cub3')
    print("launch target:", cub_target)

    return LaunchDescription([
        Node(# オドメトリをクリアしておく
            package='cub_bringup',
            executable='clear_odom',
            name='clear_odom',
            output='screen'
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_bringup"), "launch", "target_cub3.launch.py"]
            ),
            condition=IfCondition("true" if cub_target == 'cub3' else "false")
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_bringup"), "launch", "target_mcub.launch.py"]
            ),
            condition=IfCondition("true" if (cub_target == 'mcub' or cub_target == 'mcub_direct') else "false")
        ),
    ])