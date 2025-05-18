from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    joy_dev = "/dev/input/js0"

    return LaunchDescription([
        Node(
            package='cub_commander',
            executable='cub_commander_node',
            output='screen',
            parameters=[{'dev': joy_dev}],
            # remappings=[('/cmd_vel_atom', '/cmd_vel'),
            #             ('/cmd_vel', '/cmd_vel_input')],
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
            arguments=["serial", "--dev", "/dev/ttyATOM", "-b", "115200", "-v6"]
        ),
        # Node(
        #     package='control_mcub',
        #     executable='control_mcub_moter_node',
        # ),
    ])