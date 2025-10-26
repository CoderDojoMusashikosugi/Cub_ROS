from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
import os

def generate_launch_description():
    cub_target = os.getenv('CUB_TARGET', 'cub3')
    print("launch target:", cub_target)

    home_directory="/home/cub/"
    topic_list=[
        "/SLC1L_scan",
        "/SLC1R_scan",
        "/aidalm",
        "/aideph",
        # "/camera/camera/aligned_depth_to_color/camera_info",
        # "/camera/camera/aligned_depth_to_color/image_raw",
        # "/camera/camera/color/camera_info",
        # "/camera/camera/color/image_raw",
        # "/camera/camera/color/metadata",
        # "/camera/camera/depth/camera_info",
        #"/camera/camera/depth/color/points",
        # "/camera/camera/depth/image_rect_raw",
        # "/camera/camera/depth/metadata",
        # "/camera/camera/extrinsics/depth_to_color",
        # "/camera/camera/extrinsics/depth_to_depth",
        # "/camera/camera/rgbd",
        "/cmd_vel_atom",
        "/diagnostics",
        "/diagnostics_agg",
        "/diagnostics_toplevel_state",
        "/fix",
        "/fix_velocity",
        "/imu/data",
        "/imu/data_raw",
        "/monhw",
        "/mros_debug_topic",
        "/navclock",
        "/navheading",
        "/navpvt",
        "/navrelposned",
        "/navstate",
        "/navstatus",
        "/navsvin",
        "/parameter_events",
        "/pose",
        "/proceed_to_next_group",
        "/rosout",
        "/rxmrtcm",
        "/tf",
        "/tf_static",
        #"/velodyne_packets",
        "/velodyne_points",
        "/wheel_positions",
        "/test/ekf_pose",
        "/test/ndt_pose",
        "/gps_pose",
        "/odom"
    ]
    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PathJoinSubstitution(
        #         [FindPackageShare("cub_bringup"), "launch", "common.launch.py"]
        #     ),
        # ),

        TimerAction(period=3.0, actions=[ExecuteProcess(
            cwd=home_directory+"rosbag",
            cmd=['ros2', 'bag', 'record', '-s', 'mcap'] + topic_list,
            output='screen'
        )])
    ])
