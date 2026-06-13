from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os
import shlex


def generate_launch_description():
    joy_dev = "/dev/input/js0"
    cub_target = os.getenv('CUB_TARGET', 'handy2_bringup')
    print("launch target:", cub_target)

    print("file://" + os.path.join(get_package_share_directory('handy2_bringup'), 'config', 'rpgsc.yaml'))


    lidar_config=get_package_share_directory('handy2_bringup')+'/config/jt128-config.yaml'
    print("lidar config:", lidar_config);

    return LaunchDescription([
        Node(
            package='handy2_bringup',
            executable='web_control_node.py',
            output='screen',
        ),
        Node(
            package='handy2_bringup',
            executable='gnss_time_node',
            name='gnss_time_node',
            output='screen',
            parameters=[{
                'port': '/dev/serial0',
                'baudrate': 9600,
                'shm_unit': 0,
            }],
        ),
        Node(
            package='handy2_bringup',
            executable='pico_node',
            name='pico_node',
            output='screen',
            parameters=[{
                'shutter_on_us': 2000,
                'shutter_offset_us': 0,
            }],
        ),
        Node(
            namespace='hesai_ros_driver', 
            package='hesai_ros_driver', 
            executable='hesai_ros_driver_node', 
            output='screen',
            parameters=[{'config_path': lidar_config}]
        ),
        TimerAction(
            period=2.0,
            actions=[
                ComposableNodeContainer(
                    name='handy2_driver_container',
                    namespace='',
                    package='rclcpp_components',
                    executable='component_container_mt',
                    composable_node_descriptions=[
                        ComposableNode(
                            package='camera_ros',
                            plugin='camera::CameraNode',
                            name='camera_node',
                            parameters=[{
                                'format': "BGR888", 
                                'width' : 1456,
                                'height': 1088,
                                'camera_info_url': "file://" + os.path.join(get_package_share_directory('handy2_bringup'), 'config', 'rpgsc.yaml'),
                                'role': 'video',
                                'frame_id': 'gs_camera',
                                'ExposureTimeMode': 1, # 1: manual 0: auto
                                'AwbEnable': False,
                                'ColourGains': [2.56, 2.08],
                                'AeEnable': False,
                                'AnalogueGainMode': 1, # 1: manual 0: auto
                                'AnalogueGain': 4.6,
                            }],
                            remappings=[('image_raw', '/camera_node/image_raw')],
                            extra_arguments=[{'use_intra_process_comms': True}],
                        ),
                        # ComposableNode(
                        #     package='image_view',
                        #     plugin='image_view::ImageViewNode',
                        #     name='image_view_node',
                        #     remappings=[('image', '/camera_node/image_raw')],
                        #     parameters=[{'autosize': True}],
                        #     extra_arguments=[{'use_intra_process_comms': True}],
                        # ),
                    ],
                    output='both',
                ),
            ]
        ),
    ])
