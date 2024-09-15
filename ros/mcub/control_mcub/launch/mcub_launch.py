from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. micro_ros_agent の実行
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyATOM', '-b', '1500000'],
            output='screen'
        ),

        # 2. mcub_wheel_odometry_node の実行
        ExecuteProcess(
            cmd=['ros2', 'run', 'mcub_wheel_odometry', 'mcub_wheel_odometry_node'],
            output='screen'
        ),

        # 3. sllidar_c1_launch.py の実行
        ExecuteProcess(
            cmd=['ros2', 'launch', 'sllidar_ros2', 'sllidar_c1_launch.py'],
            output='screen'
        ),
    ])
