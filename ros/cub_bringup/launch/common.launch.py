from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
import os


def launch_setup(context):
    simulation = LaunchConfiguration('simulation').perform(context)
    cub_target = 'cub4' if simulation == 'godot' else os.getenv('CUB_TARGET', 'cub4')
    return [
        Node(# オドメトリをクリアしておく
            package='cub_bringup',
            executable='clear_odom',
            name='clear_odom',
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulation'), 'none')),
            output='screen'
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub4_bringup"), "launch", "common.launch.py"]
            ),
            launch_arguments={'simulation': LaunchConfiguration('simulation')}.items(),
            condition=IfCondition("true" if cub_target == "cub4" else "false")
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("mcub_bringup"), "launch", "common.launch.py"]
            ),
            condition=IfCondition("true" if cub_target in ("mcub", "mcub_direct") else "false")
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("spidar_bringup"), "launch", "common.launch.py"]
            ),
            condition=IfCondition("true" if cub_target == "spidar" else "false")
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("handy1_bringup"), "launch", "common.launch.py"]
            ),
            condition=IfCondition("true" if cub_target == "handy1" else "false")
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='none', choices=['none', 'godot']),
        OpaqueFunction(function=launch_setup),
    ])
