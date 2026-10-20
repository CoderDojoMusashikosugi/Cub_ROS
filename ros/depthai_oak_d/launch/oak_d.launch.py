"""OAK-D bringup: publishes CAM_A as sensor_msgs/Image (1280x720).

Usage:
  ros2 launch depthai_oak_d oak_d.launch.py              # RGB
  ros2 launch depthai_oak_d oak_d.launch.py mode:=rgbd   # RGB + Depth
Record separately, e.g.:
  ros2 bag record /oak/rgb/image_raw /oak/rgb/camera_info /tf_static
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("depthai_oak_d")
    driver_dir = get_package_share_directory("depthai_ros_driver_v3")

    mode = LaunchConfiguration("mode").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    if not params_file:
        params_file = os.path.join(pkg_dir, "config", f"{mode}.yaml")

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(driver_dir, "launch", "driver.launch.py")),
        launch_arguments={
            "name": LaunchConfiguration("name"),
            "camera_model": LaunchConfiguration("camera_model"),
            "params_file": params_file,
            "parent_frame": LaunchConfiguration("parent_frame"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )
    return [driver]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D-S2"),
        DeclareLaunchArgument("parent_frame", default_value="oak_parent_frame"),
        DeclareLaunchArgument("mode", default_value="rgb", description="rgb | rgbd"),
        DeclareLaunchArgument("params_file", default_value="",
                              description="Override config (default: config/<mode>.yaml)"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
