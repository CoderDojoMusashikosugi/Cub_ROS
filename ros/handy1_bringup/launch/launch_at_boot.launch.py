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
    cub_target = os.getenv('CUB_TARGET', 'mcub')
    print("launch target:", cub_target)

    print("file://" + os.path.join(get_package_share_directory('handy1_bringup'), 'config', 'rpgsc.yaml'))

    ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    ld_library_path_escaped = shlex.quote(ld_library_path)



    # Livox MID360 parameters
    livox_params = [
        {"xfer_format": 0},
        {"multi_topic": 0},
        {"data_src": 0},
        {"publish_freq": 10.0},
        {"output_data_type": 0},
        {"frame_id": "livox_frame"},
        {"lvx_file_path": "/home/livox/livox_test.lvx"},
        {"user_config_path": os.path.join(get_package_share_directory('handy1_bringup'), 'config', 'MID360_config.json')},
        {"cmdline_input_bd_code": "livox0000000001"}
    ]

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
        # TimerAction(  # TimerActionを無効化しておく
        #     period=5.0,
        #     actions=[
            ComposableNodeContainer(
                name='handy1_driver_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                    ComposableNode(
                        package='livox_ros_driver2',
                        plugin='livox_ros::DriverNode',
                        name='livox_lidar_publisher',
                        parameters=livox_params,
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                    ComposableNode(
                        package='camera_ros',
                        plugin='camera::CameraNode',
                        name='camera_node',
                        parameters=[{
                            'format': "BGR888", 
                            'width' : 1456,
                            'height': 1088,
                            'camera_info_url': "file://" + os.path.join(get_package_share_directory('handy1_bringup'), 'config', 'rpgsc.yaml'),
                            'role': 'video',
                            'frame_id': 'gs_camera',
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
                    # ComposableNode(
                    #     package='handy1_bringup',
                    #     plugin='handy1_bringup::FrequencyChecker',
                    #     name='frequency_checker',
                    #     remappings=[('input_image', '/camera_node/image_raw')],
                    #     extra_arguments=[{'use_intra_process_comms': True}],
                    # ),
                    ComposableNode(
                        package='handy1_bringup',
                        plugin='handy1_bringup::LivoxFrequencyChecker',
                        name='livox_frequency_checker',
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                    ComposableNode(
                        package='handy1_bringup',
                        plugin='handy1_bringup::ImuFrequencyChecker',
                        name='imu_frequency_checker',
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),

                ],
                # prefix=f"sudo -E env LD_LIBRARY_PATH={ld_library_path_escaped} nice -n -10", # sudo利用を無効化しておく
                output='both',
            ),
        #     ]
        # ),
    ])
