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
from launch.conditions import IfCondition

# TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map_localization',
        default="/home/cub/maps/kakunin-1004/kakunin-1004-modified.yaml") # cub_bringup navigation.launch.pyで呼び出し時に上書きされる。デフォルト値設定はそちらに。

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    cub_target = os.getenv('CUB_TARGET', 'cub3')
    if cub_target == 'cub3':
        param_file_name = 'cub3_nav2.yaml'
    elif cub_target == 'mcub' or cub_target == 'mcub_direct':
        param_file_name = 'mcub_nav2.yaml'
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
    cub_navigation_launch_dir = os.path.join(get_package_share_directory('cub_navigation'), 'launch')

    # collison_monitor_launch_file_dir = os.path.join(get_package_share_directory('nav2_collision_monitor'), 'launch')


    # rviz_config_dir = os.path.join(
    #     get_package_share_directory('nav2_bringup'),
    #     'rviz',
    #     'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_localization',
            default_value=map_dir,
            description='Full path to localization map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([cub_navigation_launch_dir, '/nav2_localization_launch.py']),
        #     launch_arguments={
        #         'map': map_dir,
        #         'use_sim_time': use_sim_time,
        #         'params_file': param_dir,
        #     }.items(),
        # ),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'use_composition', default_value='True',
            description='Whether to use composed bringup'),

        DeclareLaunchArgument(
            'use_respawn', default_value='False',
            description='Whether to respawn if a node crashes. Applied when composition is disabled.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(cub_navigation_launch_dir,
                                                    'nav2_localization_launch.py')),
            launch_arguments={'namespace': namespace,
                            'map': map_dir,
                            'use_sim_time': use_sim_time,
                            'autostart': autostart,
                            'params_file': params_file,
                            'use_composition': use_composition,
                            'use_respawn': use_respawn,
                            'container_name': 'nav2_container'}.items()
        ),
    
        # オドメトリオンリー走行時にグローバル経路計画に使う地図読み込み用
        # 白紙地図なら /home/cub/colcon_ws/src/cub/cub_navigation/maps/empty/map.yaml を使うと良い
        # Node(
        #     package='nav2_map_server',
        #     executable='map_server',
        #     name='map_server',
        #     output='screen',
        #     respawn_delay=2.0,
        #     parameters=[{'yaml_filename': map_dir}],
        # ),
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_cub_navigation',
        #     output='screen',
        #     parameters=[{'autostart': True}, {'node_names': ['map_server']}],
        # ),

        # 通常のmcub(map->odom->base_link環境)では不要、オドメトリオンリーなmcubでは必要
        # Cub3(ekf_localiがmap->base_linkを出す)では必要
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0','0','0','0','0','0','1','map','odom'],
        #     condition=IfCondition("false" if (cub_target == 'mcub' or cub_target == 'mcub_direct') else "true"),
        # ),

        # Node(
        #     package='cub_behavior_tree',
        #     executable='waypoint_navigator',
        #     arguments=["/home/cub/colcon_ws/src/cub/cub_behavior_tree/routes/3d_waypoints.yaml"]
        #     # arguments=["/home/cub/colcon_ws/src/cub/cub_behavior_tree/routes/2d_waypoints.yaml"]
        # ),
        # Node(
        #     package='cub_navigation',
        #     executable='waypoint_visualizer'
        # ),


        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([collison_monitor_launch_file_dir, '/collision_monitor_node.launch.py']),
        #     launch_arguments=[
        #         ('use_sim_time', use_sim_time),
        #     ],
        # ),
        
        # TimerAction(period=5.0, actions=[IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([cub_navigation_launch_dir, '/cub_costmapfilter.launch.py']),)]),
    ])