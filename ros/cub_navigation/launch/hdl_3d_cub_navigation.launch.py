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

# TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default="/home/cub/support_tools/pcd_slicer/map_output/musakosumap.yaml")

    cub_target = os.getenv('CUB_TARGET', 'cub3')
    if cub_target == 'cub3':
        param_file_name = 'cub3_nav2_3d.yaml'
    elif cub_target == 'mcub':
        param_file_name = 'mcub_nav2.yaml'
    else:
        param_file_name = 'cub3_nav2_3d.yaml'

    cub_nav_params_default_path = os.path.join(
        get_package_share_directory('cub_navigation'),
        'param',
        param_file_name)

    nav2_launch_file_dir = os.path.join(get_package_share_directory('cub_navigation'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('hdl_localization'), 'launch'), '/hdl_localization.launch.py']),
            launch_arguments={
                'globalmap_pcd': "/home/cub/rosbag/musashikosugi_indoor.pcd",
                'params_file': os.path.join(get_package_share_directory('hdl_localization'), 'param', 'hdl_localization.yaml')
            }.items()
        ),

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
            default_value=cub_nav_params_default_path,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/nav2_bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': cub_nav_params_default_path,
                'use_localization': 'False'}.items(),
        ),

        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','map','odom']
        ),

        Node(
            package='cub_behavior_tree',
            executable='waypoint_navigator',
            arguments=["/home/cub/colcon_ws/src/cub/cub_behavior_tree/routes/3d_waypoints.yaml"]
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
    ])