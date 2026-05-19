from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        Node(
            package='handy2_bringup',
            executable='gnss_time_node',
            name='gnss_time_node',
            output='screen',
            parameters=[{
                'port': '/dev/serial0',
                'baudrate': 9600,
                'shm_unit': 0,
            }],
        ),
    ])