# copy from https://github.com/ROBOTIS-GIT/turtlebot3/blob/humble-devel/turtlebot3_navigation2/launch/navigation2.launch.py
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.conditions import IfCondition

# TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def select_default_params(simulation: str) -> str:
    cub_target = 'cub4' if simulation == 'godot' else os.getenv('CUB_TARGET', 'cub4')
    if cub_target == 'cub4':
        param_file_name = 'cub4_nav2.yaml'
    elif cub_target == 'mcub' or cub_target == 'mcub_direct':
        param_file_name = 'mcub_nav2.yaml'
    elif cub_target == 'spidar':
        param_file_name = 'spidar.yaml'
    else:
        param_file_name = 'cub4_nav2.yaml'
    return os.path.join(
        get_package_share_directory('cub_navigation'),
        'param',
        param_file_name)


def launch_setup(context):
    simulation = LaunchConfiguration('simulation').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_dir = LaunchConfiguration('map')
    waypoints_file = LaunchConfiguration('waypoints')
    params_arg = LaunchConfiguration('params_file').perform(context)
    param_dir = params_arg if params_arg else select_default_params(simulation)
    cub_navigation_launch_dir = os.path.join(get_package_share_directory('cub_navigation'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('cub_navigation'),
        'param',
        'nav2_rviz_view.rviz')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([cub_navigation_launch_dir, '/nav2_bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'use_localization': "False"}.items(), # localizationはnav2_bringup.launch.pyで起動してるためFalseにする。
        ),

        Node(
            package='cub_behavior_tree',
            executable='waypoint_navigator',
            arguments=[waypoints_file]
            # arguments=["/home/cub/colcon_ws/src/cub/cub_behavior_tree/routes/2d_waypoints.yaml"]
        ),
        Node(
            package='cub_navigation',
            executable='waypoint_visualizer'
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ここの地図は経路計画専用
    # 白紙地図なら /home/cub/colcon_ws/src/cub/cub_navigation/maps/empty/map.yaml を使うと良い
    map_dir = LaunchConfiguration(
        'map',
        default="/home/cub/maps/kakunin-1004/kakunin-1004-modified.yaml") # cub_bringup navigation.launch.pyで呼び出し時に上書きされる。デフォルト値設定はそちらに。

    waypoints_dir = LaunchConfiguration(
        'waypoints',
        default="/home/cub/colcon_ws/src/cub/cub_behavior_tree/routes/furo_waypoints.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation',
            default_value='none',
            choices=['none', 'godot']),

        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'waypoints',
            default_value=waypoints_dir,
            description='Full path to waypoints yaml file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        OpaqueFunction(function=launch_setup),
    ])