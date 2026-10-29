"""Build a GLIM map directly from a rosbag containing Velodyne and Spresense IMU."""

from datetime import datetime
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_mapping(context):
    bag = Path(LaunchConfiguration('rosbag').perform(context)).expanduser().resolve()
    if not bag.exists():
        raise RuntimeError(f'Rosbag does not exist: {bag}')

    config = Path(LaunchConfiguration('config_path').perform(context)).expanduser().resolve()
    if not (config / 'config.json').is_file():
        raise RuntimeError(f'GLIM config.json does not exist in: {config}')

    start = float(LaunchConfiguration('start_offset').perform(context))
    duration = float(LaunchConfiguration('playback_duration').perform(context))
    if not all(math.isfinite(value) and value >= 0.0 for value in (start, duration)):
        raise RuntimeError('start_offset and playback_duration must be finite and nonnegative')

    output = LaunchConfiguration('dump_path').perform(context)
    if output:
        dump = Path(output).expanduser().resolve()
    else:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        dump = Path.home() / 'maps' / f'{bag.stem}_glim_velodyne_{stamp}'
    if dump.exists() and (not dump.is_dir() or any(dump.iterdir())):
        raise RuntimeError(f'Choose an empty output directory: {dump}')
    dump.mkdir(parents=True, exist_ok=True)

    # glim_rosbag globs positional argv entries as bag paths. Pass parameters
    # inline: a Node(parameters=[...]) YAML file can be mistaken for another bag.
    mapping = Node(
        package='glim_ros',
        executable='glim_rosbag',
        output='both',
        arguments=[
            str(bag), '--ros-args',
            '-p', f'config_path:={config}',
            '-p', f'dump_path:={dump}',
            '-p', 'auto_quit:=true',
            '-p', f'start_offset:={start}',
            '-p', f'playback_duration:={duration}',
        ],
        # Allow GLIM to finish optimization and saving after an interruption.
        sigterm_timeout='300',
        sigkill_timeout='30',
    )
    return [
        LogInfo(msg=f'GLIM Velodyne input: {bag}'),
        LogInfo(msg=f'GLIM map output: {dump}'),
        mapping,
    ]


def generate_launch_description():
    config = Path(get_package_share_directory('cub4_bringup')) / 'config' / 'glim_params_velodyne'
    return LaunchDescription([
        DeclareLaunchArgument('rosbag', description='Container path to a rosbag2 directory or MCAP file'),
        DeclareLaunchArgument(
            'dump_path', default_value='',
            description='Empty output directory; defaults to ~/maps/<bag>_glim_velodyne_<timestamp>'),
        DeclareLaunchArgument('config_path', default_value=str(config), description='GLIM JSON config directory'),
        DeclareLaunchArgument('start_offset', default_value='0.0', description='Skip this many bag seconds'),
        DeclareLaunchArgument(
            'playback_duration', default_value='0.0',
            description='Process this many bag seconds after start_offset (0: entire bag)'),
        OpaqueFunction(function=_launch_mapping),
    ])
