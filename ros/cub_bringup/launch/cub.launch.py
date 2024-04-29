from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_linux',
            executable='joy_linux_node',
        ),
        Node(
            package='cub_bringup',
            executable='teleop_joy',
        )
    ])