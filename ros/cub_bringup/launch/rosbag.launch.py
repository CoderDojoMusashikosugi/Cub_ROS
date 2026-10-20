from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.actions import LogInfo
import os

def generate_launch_description():
    cub_target = os.getenv('CUB_TARGET', 'cub4')
    print("launch target:", cub_target)

    home_directory="/home/cub/"
    ext_storage_dir = os.path.join(home_directory, "ext_storage")
    rosbag_dir = os.path.join(home_directory, "rosbag")

    # リンク切れではなく、実体が存在する有効なディレクトリかを確認する
    if os.path.exists(ext_storage_dir) and os.path.isdir(ext_storage_dir):
        save_dir = ext_storage_dir
    else:
        save_dir = rosbag_dir
    topic_list=[
        # 今はlaunch at boot系のtopicしか書いてないことに注意
        "/cmd_vel",
        "/cmd_vel_atom",
        "/diagnostics",
        "/imu/data_raw",
        "/imu/temperature",
        "/joy",
        "/joy/set_feedback",
        "/livox/livox/imu",
        "/livox/livox/lidar",
        "/livox/livox/time_type",
        "/mros_debug_topic",
        "/parameter_events",
        "/proceed_to_next_group",
        "/rosout",
        "/um982_node/fix",
        "/um982_node/fix_secondary",
        "/um982_node/pose",
        # "/velodyne_packets",
        "/velodyne_points",
        "/wheel_positions",
        "/oak/imu/data",                          
        "/oak/rgb/camera_info",
        "/oak/rgb/image_raw",
        # "/oak/rgb/image_raw/compressed",
        # "/oak/rgb/image_raw/compressedDepth",
        # "/oak/rgb/image_raw/theora",
        # "/oak/rgb/image_raw/zstd",
        "/oak/stereo/camera_info",
        "/oak/stereo/image_raw",
        # "/oak/stereo/image_raw/compressed",
        # "/oak/stereo/image_raw/compressedDepth",
        # "/oak/stereo/image_raw/theora",
        # "/oak/stereo/image_raw/zstd",
    ]
    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PathJoinSubstitution(
        #         [FindPackageShare("cub_bringup"), "launch", "common.launch.py"]
        #     ),
        # ),

        LogInfo(msg=f"rosbag is saving to: {save_dir}"),

        TimerAction(period=3.0, actions=[ExecuteProcess(
            cwd=save_dir,
            cmd=['ros2', 'bag', 'record', '-s', 'mcap'] + topic_list,
            output='screen'
        )])
    ])
