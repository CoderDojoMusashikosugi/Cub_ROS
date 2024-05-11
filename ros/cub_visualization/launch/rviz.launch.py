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

def kill_rviz2(event, context):
    # docker execをkillしても、その先で実行してるプロセスをkill出来ないらしいので、仕方なくここで実行
    # dockerのこの挙動、10年前から困ってる人が居るみたい https://github.com/moby/moby/issues/9098
    system('docker exec -u ubuntu cub_ros_rviz /bin/bash -c "pkill -f rviz2/rviz2"')

def generate_launch_description():
    remote_arg = DeclareLaunchArgument("remote", default_value= ("true" if (getenv('ENABLE_REMOTE_RVIZ') == '1') else "false") )
    remote = LaunchConfiguration('remote')
    rviz_config_file = PathJoinSubstitution([FindPackageShare('cub_visualization'), 'config', 'cub.rviz'])

    return LaunchDescription([
        remote_arg,
        Node( # rviz2 ローカル起動用
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=UnlessCondition(remote) # ENABLE_REMOTE_RVIZ=0の時に実行
        ),

        ExecuteProcess( # rviz2 リモート起動用
            cmd=['docker', 'exec', '-u', 'ubuntu', 'cub_ros_rviz', 
                 '/bin/bash', '-c', 
                 'source /home/cub/colcon_ws/install/setup.bash && DISPLAY=:1 ros2 launch cub_visualization rviz.launch.py'],
            output='both',
            condition=IfCondition(remote) # ENABLE_REMOTE_RVIZ=1の時に実行
        ),
        RegisterEventHandler( # rviz2 リモート終了用
            event_handler=OnShutdown(
                on_shutdown=kill_rviz2 # launchファイル終了タイミングでrviz2をkill
            ),
            condition=IfCondition(remote) # ENABLE_REMOTE_RVIZ=1の時に実行
        ),
    ])
