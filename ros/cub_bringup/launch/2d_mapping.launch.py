from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import PushRosNamespace
import os

def generate_launch_description():
    # Launchファイルの返り値
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(FindPackageShare('slam_toolbox').find('slam_toolbox'), 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'slam_params_file': os.path.join(get_package_share_directory("cub_bringup"), 'config', 'mapper_params_online_async.yaml'),
                'use_sim_time': "False",
            }.items()
        ),
    ])
