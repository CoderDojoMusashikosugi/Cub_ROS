"""OAK-D bringup equivalent to cub_bringup/src/oak-d_s2_record.py.

Usage:
  ros2 launch depthai_oak_d oak_d.launch.py                       # RGB H264 only
  ros2 launch depthai_oak_d oak_d.launch.py record:=true output:=test_video
  ros2 launch depthai_oak_d oak_d.launch.py mode:=rgbd record:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("depthai_oak_d")
    driver_dir = get_package_share_directory("depthai_ros_driver_v3")

    name = LaunchConfiguration("name").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    if not params_file:
        params_file = os.path.join(pkg_dir, "config", f"{mode}_h264.yaml")

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(driver_dir, "launch", "driver.launch.py")),
        launch_arguments={
            "name": name,
            "camera_model": LaunchConfiguration("camera_model"),
            "params_file": params_file,
            "parent_frame": LaunchConfiguration("parent_frame"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    # Recording (replacement for dai.node.RecordVideo)
    topics_regex = f"/{name}/rgb/.*"
    if mode == "rgbd":
        topics_regex = f"/{name}/(rgb|stereo)/.*"
    record = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "-o", LaunchConfiguration("output"),
            "-s", "mcap",
            "--regex", topics_regex,
            "/tf", "/tf_static",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("record")),
    )
    return [driver, record]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D-S2"),
        DeclareLaunchArgument("parent_frame", default_value="oak_parent_frame"),
        DeclareLaunchArgument("mode", default_value="rgb", description="rgb | rgbd"),
        DeclareLaunchArgument("params_file", default_value="",
                              description="Override config (default: config/<mode>_h264.yaml)"),
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("output", default_value="test_video",
                              description="rosbag output directory"),
        OpaqueFunction(function=launch_setup),
    ])
