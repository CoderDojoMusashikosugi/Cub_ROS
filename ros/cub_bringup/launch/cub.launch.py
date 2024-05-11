from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_linux',
            executable='joy_linux_node',
        ),
        Node(
            package='cub_bringup',
            executable='teleop_joy',
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_visualization"), "launch", "rviz.launch.py"]
            ),
        ),
    ])