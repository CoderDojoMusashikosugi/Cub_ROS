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
    cub_target = os.getenv('CUB_TARGET', 'cub2')
    print("launch target:", cub_target)

    home_directory="/home/cub/"
    topic_list=[
        "/bno055/calib_status",
        "/bno055/grav",
        "/bno055/imu",
        "/bno055/imu_raw",
        "/bno055/mag",
        "/bno055/temp",
        "/clicked_point",
        "/cmd_vel",
        "/diagnostics",
        "/fix",
        "/goal_pose",
        "/heading",
        "/initialpose",
        "/joy",
        "/joy/set_feedback",
        "/mros_debug_topic",
        "/odom",
        "/parameter_events",
        "/rosout",
        "/scan",
        "/tf",
        "/tf_static",
        "/time_reference",
        "/vel",
        "/velodyne_packets",
        "/velodyne_points",
        "/wheel_positions",
    ]

    return LaunchDescription([
        TimerAction(period=1.0, actions=[ExecuteProcess(
            cwd=home_directory+"rosbag",
            cmd=['ros2', 'bag', 'record', '-s', 'mcap'] + topic_list,
            output='screen'
        )])
    ])