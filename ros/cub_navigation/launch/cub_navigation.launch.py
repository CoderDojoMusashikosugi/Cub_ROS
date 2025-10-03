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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

# TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default = "/home/cub/support_tools/3dmap_converter/kakunin-0920.yaml")
    cub_target = os.getenv('CUB_TARGET', 'cub3')
    if cub_target == 'cub3':
        param_file_name = 'cub3_nav2.yaml'
    elif cub_target == 'mcub' or cub_target == 'mcub_direct':
        param_file_name = 'mcub_tb3base.yaml'
    elif cub_target == 'spidar':
        param_file_name = 'spidar.yaml'
    else:
        param_file_name = 'cub3_nav2.yaml'

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('cub_navigation'),
            'param',
            param_file_name))
    
    cub_bringup_launch_dir = os.path.join(get_package_share_directory('cub_bringup'), "launch")

    cub_navigation_launch_dir = os.path.join(get_package_share_directory('cub_navigation'), "launch")

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    collison_monitor_launch_file_dir = os.path.join(get_package_share_directory('nav2_collision_monitor'), 'launch')

    ekf_licalizer_launch_dir = os.path.join(get_package_share_directory('ekf_localizer'), "launch")

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        # Node(
        #     package='cub_bringup',
        #     executable='odom_to_tf',
        # ),
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([cub_bringup_launch_dir, '/cub.launch.py']),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ekf_licalizer_launch_dir, '/ekf_locali.launch.py']),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([cub_navigation_launch_dir, '/bringup_launch.py']),
            launch_arguments=[
                ('map', map_dir),
                ('use_sim_time', use_sim_time),
                ('params_file', param_dir)
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([collison_monitor_launch_file_dir, '/collision_monitor_node.launch.py']),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
            ],
        ),

        
        TimerAction(period=5.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([cub_navigation_launch_dir, '/cub_costmapfilter.launch.py']),)]),
    ])