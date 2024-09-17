from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')  # パッケージ名を確認
    robot_description_dir = get_package_share_directory('mcub_assy_description')
    
    # 各Launchファイルのパスを指定
    navigation_launch = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    slam_toolbox_launch = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
    sllidar_launch = os.path.join(sllidar_ros2_dir, 'launch', 'sllidar_c1_launch.py')
    robot_description = os.path.join(robot_description_dir, 'launch', 'robot_description.launch.py')
    # 各Launchファイルをインクルード
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch)
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch)
    )

    sllidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch)
    )
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description)
    )
    return LaunchDescription([
        Node(
            package='wheel_odometry',
            executable='wheel_odometry_node',
        ),
        nav2,
        slam_toolbox,
        sllidar,
        description
    ])