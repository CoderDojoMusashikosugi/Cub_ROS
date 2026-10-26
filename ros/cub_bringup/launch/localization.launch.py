# localization.launch.py
# これをlaunch_at_boot.launch.pyと一緒に起動すると自己位置推定が出来る
# 自律走行はしない。
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os


def require_navigation(context):
    # Godot currently provides Joy/Twist only; do not start sensor-dependent nodes.
    simulation = LaunchConfiguration('simulation').perform(context)
    if simulation != 'none':
        raise ValueError(f'simulation:={simulation} currently supports velocity control only; '
                         'odom, localization TF and scan are required before navigation/localization')
    return []


def generate_launch_description():
    cub_target = os.getenv('CUB_TARGET', 'cub4')
    print("launch target:", cub_target)

    map_dir = LaunchConfiguration(
        'map_localization',
        default = "/home/cub/maps/oudanhodoumade/mapoudanhodoumade_manual_crean.yaml")


    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='none', choices=['none', 'godot']),
        OpaqueFunction(function=require_navigation),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_bringup"), "launch", "common.launch.py"]
            ),
            launch_arguments={'simulation': LaunchConfiguration('simulation')}.items(),
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("ekf_localizer"), "launch", "ekf_locali.launch.py"]
            ),
            launch_arguments=[
                ('map_localization', map_dir)
            ],
            condition=IfCondition("true" if cub_target == 'cub4' else "false")
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0','1','map','odom'],
            condition=IfCondition("true" if cub_target == 'cub4' else "false")
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_navigation"), "launch", "2d_localization.launch.py"]
            ),
            launch_arguments=[
                ('map_localization', map_dir)
            ],
            condition=IfCondition("true" if (cub_target == 'mcub' or cub_target == 'mcub_direct') else "false")
        ),
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

        # IncludeLaunchDescription(
        #     PathJoinSubstitution(
        #         [FindPackageShare("cub_navigation"), "launch", "2d_localization.launch.py"]
        #     ),
        #     launch_arguments=[
        #         ('map_localization', map_dir)
        #     ],
        # ),
        # IncludeLaunchDescription(
        #     PathJoinSubstitution(
        #         [FindPackageShare("cub_navigation"), "launch", "odom_as_localization.launch.py"]
        #     ),
        # ),
        # IncludeLaunchDescription(
        #     PathJoinSubstitution(
        #         [FindPackageShare("cub_navigation"), "launch", "gnss_as_localization.launch.py"]
        #     ),
        # ),

    ])