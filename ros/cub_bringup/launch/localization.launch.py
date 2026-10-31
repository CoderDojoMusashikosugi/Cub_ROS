# localization.launch.py
# これをlaunch_at_boot.launch.pyと一緒に起動すると自己位置推定が出来る
# 自律走行はしない。
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    cub_target = os.getenv('CUB_TARGET', 'cub4')
    print("launch target:", cub_target)

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo / rosbag) clock if true'
    )

    map_dir = LaunchConfiguration(
        'map_localization',
        default = "/home/cub/maps/oudanhodoumade/mapoudanhodoumade_manual_crean.yaml")


    return LaunchDescription([
        declare_use_sim_time,
        SetParameter(name='use_sim_time', value=use_sim_time),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_bringup"), "launch", "common.launch.py"]
            ),
            launch_arguments=[
                ('use_sim_time', use_sim_time)
            ]
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("ekf_localizer"), "launch", "ekf_locali.launch.py"]
            ),
            launch_arguments=[
                ('map_localization', map_dir),
                ('use_sim_time', use_sim_time)
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
                ('map_localization', map_dir),
                ('use_sim_time', use_sim_time)
            ],
            condition=IfCondition("true" if (cub_target == 'mcub' or cub_target == 'mcub_direct') else "false")
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("ekf_localizer"), "launch", "ekf_locali.launch.py"]
            ),
            launch_arguments=[
                ('use_sim_time', use_sim_time)
            ],
            condition=IfCondition("true" if cub_target == 'spidar' else "false")
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("handy1_bringup"), "launch", "localization.launch.py"]
            ),
            launch_arguments=[
                ('use_sim_time', use_sim_time)
            ],
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