"""Replace hardware boot bringup with a simulator transport and commander."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def start(context):
    name = LaunchConfiguration('simulation').perform(context)
    if name == 'none':
        return [LogInfo(msg='simulation:=none: use the existing hardware launch_at_boot.launch.py')]
    return [
        Node(package='cub_commander', executable='cub_commander_node', output='screen',
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulation'), 'godot')),
             parameters=[{'use_sim_time': False}]),
        Node(package='rosbridge_server', executable='rosbridge_websocket', output='screen',
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulation'), 'godot')),
             parameters=[{'address': LaunchConfiguration('address'),
                          'port': int(LaunchConfiguration('port').perform(context)),
                          'use_sim_time': False}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='none', choices=['none', 'godot']),
        DeclareLaunchArgument('address', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='9090'),
        OpaqueFunction(function=start),
    ])
