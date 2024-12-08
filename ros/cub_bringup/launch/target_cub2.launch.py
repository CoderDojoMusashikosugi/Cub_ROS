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
    # sllidar_ros2パッケージの共有ディレクトリを取得
    state_launch_file_dir = os.path.join(
        get_package_share_directory('cub_description'),
        'launch',
        'display.launch.py'
    )

    # state_publisherの起動
    state_publisher_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(state_launch_file_dir))
    
    # Launchファイルの返り値
    return LaunchDescription([
        # wheel_odometryノード
        Node(
            package='wheel_odometry',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            output='screen'
        ),

        # # RVizの起動
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution(
        #             [FindPackageShare("cub_visualization"), "launch", "rviz.launch.py"]
        #         )
        #     )
        # ),

        state_publisher_launch,
    ])
