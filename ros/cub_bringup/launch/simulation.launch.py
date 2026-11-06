"""Simulation replacement for launch_at_boot; common is started separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='none', choices=['none', 'godot']),
        DeclareLaunchArgument('address', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='9090'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('cub_simulation'), 'launch', 'bringup.launch.py'])),
            launch_arguments={key: LaunchConfiguration(key)
                              for key in ('simulation', 'address', 'port')}.items()),
    ])
