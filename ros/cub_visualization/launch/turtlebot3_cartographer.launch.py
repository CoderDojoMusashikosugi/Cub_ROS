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
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown

from os import system, getenv

def kill_remote(event, context):
    # docker execをkillしても、その先で実行してるプロセスをkill出来ないらしいので、仕方なくここで実行
    # dockerのこの挙動、10年前から困ってる人が居るみたい https://github.com/moby/moby/issues/9098
    system('sudo docker exec -u ubuntu cub_ros_rviz /bin/bash -c "pkill -f -2 turtlebot3_cartographer.launch.py"')


def generate_launch_description():
    remote_arg = DeclareLaunchArgument("remote", default_value= ("true" if (getenv('ENABLE_REMOTE_RVIZ') == '1') else "false") )
    remote = LaunchConfiguration('remote')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  turtlebot3_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='turtlebot3_lds_2d.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(get_package_share_directory('turtlebot3_cartographer'),
                                   'rviz', 'tb3_cartographer.rviz')

    return LaunchDescription([
        remote_arg,
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('odom', '/diff_drive_base_controller/odom')],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
            condition=UnlessCondition(remote) # ENABLE_REMOTE_RVIZ=0の時に実行
            ),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid',
            ),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare("turtlebot3_cartographer"), '/launch/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
            condition=UnlessCondition(remote) # ENABLE_REMOTE_RVIZ=0の時に実行
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=UnlessCondition(remote) # ENABLE_REMOTE_RVIZ=0の時に実行
            ),
        ExecuteProcess( # rviz2 リモート起動用
            cmd=['sudo', 'docker', 'exec', '-u', 'ubuntu', 'cub_ros_rviz',
                 '/bin/bash', '-c',
                 'source /home/cub/colcon_ws/install/setup.bash && DISPLAY=:1 ros2 launch cub_visualization turtlebot3_cartographer.launch.py'],
            output='both',
            condition=IfCondition(remote) # ENABLE_REMOTE_RVIZ=1の時に実行

        ),
        RegisterEventHandler( # rviz2 リモート終了用
            event_handler=OnShutdown(
                on_shutdown=kill_remote # localのlaunchファイル終了タイミングでリモート実行のlaunchをkill
            ),
            condition=IfCondition(remote) # ENABLE_REMOTE_RVIZ=1の時に実行

        ),
    ])