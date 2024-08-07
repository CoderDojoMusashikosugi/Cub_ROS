from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from os import system, getenv

import sys

def kill_remote(event, context):
    # docker execをkillしても、その先で実行してるプロセスをkill出来ないらしいので、仕方なくここで実行
    # dockerのこの挙動、10年前から困ってる人が居るみたい https://github.com/moby/moby/issues/9098
    system('sudo docker exec -u ubuntu cub_ros_rviz /bin/bash -c "pkill -f -2 navigation2.launch.py"')


def generate_launch_description():
    remote_arg = DeclareLaunchArgument("remote", default_value= ("true" if (getenv('ENABLE_REMOTE_RVIZ') == '1') else "false") )
    remote = LaunchConfiguration('remote')

    args = sys.argv

    return LaunchDescription([
        remote_arg,
        ExecuteProcess( # ローカル起動用
            cmd=['/opt/ros/humble/bin/ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py']+args[4:],
            # cmd=['ls'],
            output='both',
            condition=UnlessCondition(remote) # ENABLE_REMOTE_RVIZ=0の時に実行
        ),

        ExecuteProcess( # リモート起動用
            # cmd=['sudo', 'docker', 'exec', '-u', 'ubuntu', 'cub_ros_rviz', '/bin/bash', '-c', ]+
            #     ['"source /home/cub/colcon_ws/install/setup.bash && DISPLAY=:1 '+" ".join(args)+'"'],
            cmd=['sudo', 'docker', 'exec', '-u', 'ubuntu', 'cub_ros_rviz',
                 '/bin/bash', '-c',
                 "source /home/cub/colcon_ws/install/setup.bash && DISPLAY=:1 TURTLEBOT3_MODEL='burger' ros2 launch turtlebot3_navigation2 navigation2.launch.py "+" ".join(args[4:])],
            output='both',
            condition=IfCondition(remote) # ENABLE_REMOTE_RVIZ=1の時に実行
        ),
        RegisterEventHandler( # リモート終了用
            event_handler=OnShutdown(
                on_shutdown=kill_remote # localのlaunchファイル終了タイミングでリモート実行のlaunchをkill
            ),
            condition=IfCondition(remote) # ENABLE_REMOTE_RVIZ=1の時に実行
        ),
    ])