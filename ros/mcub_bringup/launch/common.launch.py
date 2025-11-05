from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'mcub.urdf'
    urdf = os.path.join(
        get_package_share_directory('cub_description'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    diag_config = PathJoinSubstitution([
        FindPackageShare('mcub_bringup'),
        'config',
        'diagnostics.yaml'
    ])

    return LaunchDescription([
        Node(
            package='wheel_odometry',
            executable='wheel_odometry_node',
        ),

        # 2Dのマッピングやナビゲーションを実行する際はこれを有効化、3Dでは無効化
        Node(
            package='cub_bringup',
            executable='odom_to_tf',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),

        Node(
            package='cub_diagnostics',
            executable='topic_hz_node',
            name='topic_hz_checker',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, diag_config],
        ),
        Node(
            package='cub_diagnostics',
            executable='linux_state_node',
            name='linux_state',
            output='both',
            parameters=[{'use_sim_time': use_sim_time}, diag_config],
        ),
        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='diagnostic_aggregator',
            output='both',
            parameters=[{'use_sim_time': use_sim_time}, diag_config],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('cub_visualization'), 'config', 'mcub.rviz'])],
            output='screen',
        ),
    ])