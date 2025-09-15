from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
import os

def generate_launch_description():
    cub_target = os.getenv('CUB_TARGET', 'cub3')
    print("launch target:", cub_target)

    home_directory="/home/cub/"
    topic_list=[
        "/camera/camera_info",
        # "/camera/image_raw",
        "/camera/image_raw/compressed",
        "/livox/lidar",
        "/livox/imu",
        "/livox/time_type",
    ]
    return LaunchDescription([
        TimerAction(period=1.0, actions=[ExecuteProcess(
            cwd=home_directory+"rosbag",
            cmd=['ros2', 'bag', 'record', '-s', 'mcap'] + topic_list,
            output='screen'
        )])
    ])
