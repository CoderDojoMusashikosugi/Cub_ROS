from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os

def generate_launch_description():
    joy_dev = "/dev/input/js0"
    cub_target = os.getenv('CUB_TARGET', 'mcub')
    print("launch target:", cub_target)

    return LaunchDescription([
        Node(
            package='cub_commander',
            executable='cub_commander_node',
            output='screen',
            parameters=[{'dev': joy_dev}],
        ),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            parameters=[{'dev': joy_dev}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(FindPackageShare('sllidar_ros2').find('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py')
            ]),
            launch_arguments={
                'serial_port': "/dev/ttySLC1",
                'frame_id': "SLC1_link",
            }.items()
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "--dev", "/dev/ttyATOM", "-b", "115200", "-v6"],
            condition=IfCondition("true" if cub_target == 'mcub' else "false")
        ),
        Node(
            package='control_mcub',
            executable='control_mcub_moter_node',
            condition=IfCondition("true" if cub_target == 'mcub_direct' else "false")
        ),
    ])