"""cub_console(診断を見る Web コンソール)を起動する。

ロボット本体(common.launch.py)とは別に起動する想定:
    ros2 launch cub_bringup common.launch.py
    ros2 launch cub_bringup cub_console.launch.py

アプリ本体は ~/cub_console(colcon ビルドの対象外)にある。初回は ~/cub_console/setup.sh が必要。
既定では LAN の同じサブネットからのみ接続を許可する(認証は無い)。
このマシンだけで見る場合: ros2 launch cub_bringup cub_console.launch.py host:=127.0.0.1
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    console_dir = LaunchConfiguration('console_dir')
    return LaunchDescription([
        DeclareLaunchArgument(
            'console_dir', default_value=os.path.expanduser('~/cub_console'),
            description='cub_console の配置ディレクトリ'),
        DeclareLaunchArgument(
            'host', default_value='0.0.0.0',
            description='待受アドレス。127.0.0.1 にするとこのマシンからのみ'),
        DeclareLaunchArgument(
            'port', default_value='8080', description='待受ポート'),
        DeclareLaunchArgument(
            'allow', default_value='auto',
            description='接続を許可する CIDR(空白/カンマ区切り)。auto は LAN インタフェースのサブネット'),
        ExecuteProcess(
            name='cub_console',
            # 別セッションで起動: 端末の Ctrl+C がアプリに直接届くと、launch からの SIGINT と合わせて
            # 2 回になり uvicorn が強制終了(後始末を飛ばす)するため。停止は launch からの SIGINT だけで行う
            cmd=['setsid', PathJoinSubstitution([console_dir, 'run.sh'])],
            cwd=console_dir,
            additional_env={
                'CUB_CONSOLE_HOST': LaunchConfiguration('host'),
                'CUB_CONSOLE_PORT': LaunchConfiguration('port'),
                'CUB_CONSOLE_ALLOW': LaunchConfiguration('allow'),
            },
            output='screen',
        ),
    ])
