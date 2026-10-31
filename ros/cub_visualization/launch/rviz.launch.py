from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
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
    system('sudo docker exec -u ubuntu cub_ros_rviz /bin/bash -c "pkill -f rviz2/rviz2"')

def generate_launch_description():
    rviz_config_file = PathJoinSubstitution([FindPackageShare('cub_visualization'), 'config', 'cub.rviz'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo / rosbag) clock if true'
    )

    return LaunchDescription([
        declare_use_sim_time,
        SetParameter(name='use_sim_time', value=use_sim_time),
        Node( # rviz2 ローカル起動用
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
