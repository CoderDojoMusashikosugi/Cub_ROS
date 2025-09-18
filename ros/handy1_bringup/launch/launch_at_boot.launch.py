from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import TimerAction
import os

def generate_launch_description():
    joy_dev = "/dev/input/js0"
    cub_target = os.getenv('CUB_TARGET', 'mcub')
    print("launch target:", cub_target)

    print("file://" + os.path.join(get_package_share_directory('handy1_bringup'), 'config', 'rpgsc.yaml'))

    return LaunchDescription([
        Node(
            package='handy1_bringup',
            executable='web_control_node.py',
            output='screen',
        ),
        Node(
            package='handy1_bringup',
            executable='pwm_timesync',
            prefix='sudo -E',
            output='both',
            respawn=True,
        ),
        Node(
            package='camera_ros',
            executable='camera_node',
            parameters=[{'format': "BGR888", 
                         'width' : 1456,
                         'height': 1088,
                        #  'width' : 1024,
                        #  'height': 768,
                         'camera_info_url': "file://" + os.path.join(get_package_share_directory('handy1_bringup'), 'config', 'rpgsc.yaml'),
                         'role': 'video',
                         'frame_id': 'gs_camera',
                        }],
            output='both',
        ),
        TimerAction( 
            period=5.0,
            actions=[
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare("handy1_bringup"), "launch", "msg_MID360.launch.py"]
                ),
            ),
            ]
        ),
        TimerAction( 
            period=10.0,
            actions=[
                Node(
                    package='rqt_image_view',
                    executable='rqt_image_view',
                    name='rqt_image_view_camera',
                    output='screen',
                    arguments=['/camera/image_raw/compressed'],
                    remappings=[('image', '/camera/image_raw/compressed')],
                ),
            ]
        ),
    ])