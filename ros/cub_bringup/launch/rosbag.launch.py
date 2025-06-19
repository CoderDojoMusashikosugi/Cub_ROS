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
       "/bno055/calib_status",
       "/bno055/grav",
        "/bno055/imu",
        "/bno055/imu_raw",
       "/bno055/mag",
       "/bno055/temp",
#        "/clicked_point",
#        "/cmd_vel",
        "/cmd_vel_atom",
#        "/diagnostics",
        "/fix",
#        "/goal_pose",
        "/heading",
#        "/initialpose",
#        "/joy",
#        "/joy/set_feedback",
#        "/mros_debug_topic",
        "/odom",
#        "/parameter_events",
#        "/rosout",
        "/scan",
        "/tf",
        "/tf_static",
#        "/time_reference",
#        "/vel",
        "/velodyne_packets",
        "/velodyne_points",
        "/wheel_positions",
        "/navclock",
        "/navheading",
        "/navposecef",
        "/navpvt",
        "/navrelposned",
        "/navstate",
        "/navstatus",
        "/navsvin",
        "/fix/velocity",
        # "/camera/camera/aligned_depth_to_color/camera_info",
        # "/camera/camera/aligned_depth_to_color/image_raw",
        # "/camera/camera/color/camera_info",
        # "/camera/camera/color/image_raw",
        # "/camera/camera/color/metadata",
        # "/camera/camera/depth/camera_info",
        # "/camera/camera/depth/color/points",
        # "/camera/camera/depth/image_rect_raw",
        # "/camera/camera/depth/metadata",
        # "/camera/camera/extrinsics/depth_to_color",
        # "/camera/camera/extrinsics/depth_to_depth",
        # "/camera/camera/rgbd",
    #    "/SLC1L_scan",
    #    "/SLC1R_scan",
    ]

    return LaunchDescription([
        TimerAction(period=1.0, actions=[ExecuteProcess(
            cwd=home_directory+"rosbag",
            cmd=['ros2', 'bag', 'record', '-s', 'mcap'] + topic_list,
            output='screen'
        )])
    ])
