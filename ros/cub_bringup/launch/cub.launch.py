from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os

def generate_launch_description():
    cub_target = os.getenv('CUB_TARGET', 'cub2')
    print("launch target:", cub_target)

    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "--dev", "/dev/ttyATOM", "-b", "115200", "-v6"]
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_bringup"), "launch", "target_cub2.launch.py"]
            ),
            condition=IfCondition("true" if cub_target == 'cub2' else "false")
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("cub_bringup"), "launch", "target_mcub.launch.py"]
            ),
            condition=IfCondition("true" if cub_target == 'mcub' else "false")
        ),

        Node(
            package='joy_linux',
            executable='joy_linux_node',
        ),
        Node(
            package='cub_bringup',
            executable='teleop_joy',
        ),    
    ])