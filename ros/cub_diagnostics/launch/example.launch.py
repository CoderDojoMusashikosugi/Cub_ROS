from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    diag_config = PathJoinSubstitution([
        FindPackageShare('cub_diagnostics'),
        'config',
        'example.yaml'
    ])

    return LaunchDescription([
        Node(
            package='cub_diagnostics',
            executable='topic_hz_node',
            name='topic_hz_checker',
            output='both',
            parameters=[{'use_sim_time': use_sim_time}, diag_config],
        ),
        Node(
            package='cub_diagnostics',
            executable='linux_state_node',
            name='linux_state',
            output='both',
            parameters=[{'use_sim_time': use_sim_time}, diag_config],
        ),
        # Node(
        #     package='cub_diagnostics',
        #     executable='jetson_state_node',
        #     name='jetson_state',
        #     output='both',
        #     parameters=[{'use_sim_time': use_sim_time}, diag_config],
        # ),
        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='diagnostic_aggregator',
            output='both',
            parameters=[{'use_sim_time': use_sim_time}, diag_config],
        ),
    ])
