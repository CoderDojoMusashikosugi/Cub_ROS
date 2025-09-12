from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import PushRosNamespace
import ament_index_python.packages
import launch_ros.actions
import yaml
import os

def generate_launch_description():
    ublox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('ublox_gps').find('ublox_gps'),
                "launch",
                "ublox_gps_node_zedf9p-launch.py"
            )

        )
    )
    ublox_launch_delayed = TimerAction(period=3.0, actions=[ublox_launch])

    # Launchファイルの返り値
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "--dev", "/dev/ttyATOM", "-b", "115200", "-v6"]
        ),

        # bno055ノード
        # serialerror以外は通常通り動きます
        Node(
            package='bno055',
            executable='bno055',
            name='bno055',
            parameters=[{
                'uart_port': "/dev/ttyBNO055",
                'data_query_frequency': 100,
                'frame_id':"imu_link"
            }],
            output='screen'
        ),


        # GPS launch
        #gps_launch,
        # ublox_launch_delayed,
    ])
