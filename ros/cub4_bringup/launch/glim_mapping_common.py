"""Shared rosbag mapping launch for Cub4 LiDAR profiles."""

from datetime import datetime
import math
import json
import shutil
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_mapping(context, sensor):
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
        dump = Path.home() / 'maps' / f'{bag.stem}_glim_{sensor}_{stamp}'
    if dump.exists() and (not dump.is_dir() or any(dump.iterdir())):
        raise RuntimeError(f'Choose an empty output directory: {dump}')
    dump.mkdir(parents=True, exist_ok=True)

    def mapping_node(config_dir, output_dir, seconds):
        # Inline parameters: glim_rosbag can mistake a parameters YAML for a bag.
        return Node(
            package='glim_ros', executable='glim_rosbag', output='both',
            arguments=[str(bag), '--ros-args',
                       '-p', f'config_path:={config_dir}', '-p', f'dump_path:={output_dir}',
                       '-p', 'auto_quit:=true', '-p', f'start_offset:={start}',
                       '-p', f'playback_duration:={seconds}'],
            sigterm_timeout='300', sigkill_timeout='30')

    enabled = LaunchConfiguration('enable_gnss').perform(context).lower()
    if enabled not in ('true', 'false'):
        raise RuntimeError('enable_gnss must be true or false')
    info = [LogInfo(msg=f'GLIM {sensor} input: {bag}'), LogInfo(msg=f'GLIM map output: {dump}')]
    if enabled == 'false':
        return info + [mapping_node(config, dump, duration)]

    seconds = float(LaunchConfiguration('calibration_duration').perform(context))
    if not math.isfinite(seconds) or seconds <= 0:
        raise RuntimeError('calibration_duration must be finite and positive')
    if duration > 0:
        seconds = min(seconds, duration)
    library = Path(get_package_prefix('cub_mapping')) / 'lib' / 'libcub_mapping_gnss.so'
    if not library.is_file():
        raise RuntimeError(f'Build cub_mapping with CUB_MAPPING_BUILD_GNSS=ON: {library}')
    gnss_path = LaunchConfiguration('gnss_config').perform(context)
    gnss = json.loads(Path(gnss_path).expanduser().read_text()) if gnss_path else json.loads(
        (config / 'config_cub_gnss.json').read_text())

    def prepare_config(name, mode):
        dest = dump / name
        shutil.copytree(config, dest)
        root = json.loads((dest / 'config.json').read_text())
        root['global']['config_cub_gnss'] = 'config_cub_gnss.json'
        (dest / 'config.json').write_text(json.dumps(root, indent=2) + '\n')
        ros_path = dest / root['global']['config_ros']
        ros = json.loads(ros_path.read_text())
        modules = ros['glim_ros'].setdefault('extension_modules', [])
        modules[:] = [m for m in modules if Path(m).name != library.name]
        modules.append(str(library))
        ros_path.write_text(json.dumps(ros, indent=2) + '\n')
        settings = dict(gnss, mode=mode, alignment_file=str(dump / 'calibration' / 'gnss_report.json'))
        (dest / 'config_cub_gnss.json').write_text(json.dumps(settings, indent=2) + '\n')
        if mode == 'mapping':
            global_path = dest / root['global']['config_global_mapping']
            global_config = json.loads(global_path.read_text())
            # Global IMU velocities are in the original LIO world frame. Keep
            # IMU in local LIO, use VGICP + GNSS for the ENU submap graph.
            global_config['global_mapping']['enable_imu'] = False
            global_config['global_mapping']['create_between_factors'] = True
            global_config['global_mapping']['between_registration_type'] = 'NONE'
            global_config['global_mapping']['init_pose_damping_scale'] = 0.01
            global_config['global_mapping']['isam2_relinearize_thresh'] = 0.01
            global_path.write_text(json.dumps(global_config, indent=2) + '\n')
        return dest

    calibration = mapping_node(prepare_config('calibration_config', 'calibrate'), dump / 'calibration', seconds)
    mapping_config = prepare_config('mapping_config', 'mapping')

    def check_result(event, output_dir):
        if event.returncode != 0:
            raise RuntimeError(f'GLIM process failed with code {event.returncode}: {output_dir}')
        report = json.loads((output_dir / 'gnss_report.json').read_text())
        if not report.get('success'):
            raise RuntimeError(f'GNSS processing failed: {report.get("error", "no GNSS factors")}; see {output_dir}')
        return report

    def start_mapping(event, _context):
        report = check_result(event, dump / 'calibration')
        mapping = mapping_node(mapping_config, dump, duration)

        def finish(event, _context):
            final = check_result(event, dump)
            return [LogInfo(msg=f'GNSS map complete: {final["counts"].get("factors", 0)} factors; {dump}')]

        return [LogInfo(msg=f'ENU alignment RMSE: {report["alignment_rmse_m"]:.3f} m; starting GNSS mapping'),
                RegisterEventHandler(OnProcessExit(target_action=mapping, on_exit=finish)), mapping]

    return info + [RegisterEventHandler(OnProcessExit(target_action=calibration, on_exit=start_mapping)), calibration]


def generate_mapping_launch_description(sensor):
    config = Path(get_package_share_directory('cub4_bringup')) / 'config' / f'glim_params_{sensor}'
    return LaunchDescription([
        DeclareLaunchArgument('rosbag', description='Container path to a rosbag2 directory or MCAP file'),
        DeclareLaunchArgument(
            'dump_path', default_value='',
            description=f'Empty output directory; defaults to ~/maps/<bag>_glim_{sensor}_<timestamp>'),
        DeclareLaunchArgument('config_path', default_value=str(config), description='GLIM JSON config directory'),
        DeclareLaunchArgument('enable_gnss', default_value='true', description='Use secondary RTK FIX in an ENU map'),
        DeclareLaunchArgument('gnss_config', default_value='', description='Optional GNSS JSON override'),
        DeclareLaunchArgument('calibration_duration', default_value='60.0',
                              description='Initial bag interval for GNSS/LIO yaw and translation alignment'),
        DeclareLaunchArgument('start_offset', default_value='0.0', description='Skip this many bag seconds'),
        DeclareLaunchArgument(
            'playback_duration', default_value='0.0',
            description='Process this many bag seconds after start_offset (0: entire bag)'),
        OpaqueFunction(function=lambda context: _launch_mapping(context, sensor)),
    ])
