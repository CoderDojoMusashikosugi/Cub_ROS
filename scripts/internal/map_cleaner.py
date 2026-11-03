"""Host CLI for reproducible, isolated MapCleaner batch jobs."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
REVISION = '34d1052c35de11bef6bfe5dbef8bde92b5dea2a8'
IMAGE = 'cub-map-cleaner:' + REVISION[:7]
CACHE = ROOT / 'docker/home/.cache/map_cleaner'
PRODUCTS = ('ground', 'static', 'dynamic', 'ground_below', 'other', 'terrain')


def command(args, **kwargs):
    print('+ ' + shlex.join(map(str, args)), flush=True)
    return subprocess.run(list(map(str, args)), check=True, **kwargs)


def cub(args):
    # run_in_container.sh joins argv into one shell command. Quote each argument.
    return command([ROOT / 'docker/run_in_container.sh', shlex.join(map(str, args))], cwd=ROOT)


def host_path(value):
    return Path(value).expanduser().resolve()


def container_path(value):
    p = host_path(value)
    try:
        return '/home/cub/' + str(p.relative_to(ROOT / 'docker/home'))
    except ValueError:
        try:
            return '/home/cub/colcon_ws/src/cub/' + str(p.relative_to(ROOT / 'ros'))
        except ValueError as e:
            raise ValueError(f'Path is not mounted into Cub container: {p}') from e


def setup(args):
    context = CACHE / 'context'
    context.mkdir(parents=True, exist_ok=True)
    source = context / 'source'
    if not source.exists():
        command(['git', 'clone', 'https://github.com/kamibukuro5656/MapCleaner_Unofficial.git', source])
        command(['git', '-C', source, 'checkout', '--detach', REVISION])
    actual = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    dirty = subprocess.check_output(['git', '-C', str(source), 'status', '--porcelain'], text=True).strip()
    if actual != REVISION or dirty:
        raise ValueError(f'Expected clean upstream checkout at {REVISION}: {source}')
    for path in (ROOT / 'docker/map_cleaner').iterdir():
        if path.is_file():
            shutil.copy2(path, context / path.name)
    command(['docker', 'build', '--tag', IMAGE, context])
    if not args.skip_ros_build:
        cub(['bash', '-c', 'cd /home/cub/colcon_ws && colcon build --symlink-install --packages-select cub_mapping cub4_bringup --cmake-args -DCUB_MAPPING_BUILD_GNSS=ON -DCMAKE_BUILD_TYPE=Release'])
    print('Setup complete. Image: ' + IMAGE)


def validate_frames(source, start, end):
    trajectory = source / 'traj_lidar.txt'
    frames = source / 'frames'
    report = json.loads((frames / 'report.json').read_text())
    if not report.get('success'):
        raise ValueError('Frame-saving report is not successful')
    poses = np.loadtxt(trajectory, ndmin=2)
    if poses.shape[1] != 8 or not np.isfinite(poses).all() or np.any(np.diff(poses[:, 0]) <= 0):
        raise ValueError('Invalid or non-monotonic TUM trajectory')
    if np.any(np.abs(np.linalg.norm(poses[:, 4:8], axis=1)-1) > 1e-3):
        raise ValueError('Trajectory contains an invalid rotation quaternion')
    count = len(poses)
    if report['frames'] != count:
        raise ValueError('Saved scan count and trajectory count differ')
    stop = count if end == -1 else end
    if not (0 <= start < stop <= count):
        raise ValueError(f'Frame interval must satisfy 0 <= start < end <= {count}; end=-1 means all')
    with (frames / 'manifest.csv').open() as f:
        rows = list(csv.DictReader(f))
    if len(rows) != count:
        raise ValueError('Frame manifest length differs from trajectory')
    points = 0
    for i in range(start, stop):
        row = rows[i]
        if int(row['row']) != i or abs(float(row['timestamp']) - poses[i, 0]) > 1e-7:
            raise ValueError(f'Frame/pose timestamp mismatch at row {i}')
        n = int(row['points'])
        directory = frames / f'{i:08d}'
        if n <= 0 or (directory / 'points_compact.bin').stat().st_size != 12*n or (directory / 'intensities_compact.bin').stat().st_size != 4*n:
            raise ValueError(f'Missing or truncated scan at row {i}')
        points += n
    return {'frames_total': count, 'start': start, 'end': stop, 'frames_selected': stop-start,
            'selected_scan_points': points,
            'trajectory_sha256': hashlib.sha256(trajectory.read_bytes()).hexdigest()}


def pcd_info(path):
    header = {}
    with path.open('rb') as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError('Missing PCD DATA header: ' + str(path))
            words = line.decode('ascii').split()
            if words and not words[0].startswith('#'):
                header[words[0]] = words[1:]
            if words and words[0] == 'DATA':
                offset = f.tell()
                break
    if header['FIELDS'] != ['x', 'y', 'z', 'intensity'] or header['SIZE'] != ['4']*4 or header['TYPE'] != ['F']*4 or header['DATA'] != ['binary']:
        raise ValueError('Unexpected MapCleaner PCD schema: ' + str(path))
    n = int(header['POINTS'][0])
    if path.stat().st_size != offset+16*n:
        raise ValueError('Truncated PCD payload: ' + str(path))
    info = {'points': n, 'bytes': path.stat().st_size}
    if n:
        values = np.memmap(path, mode='r', dtype='<f4', offset=offset, shape=(n, 4))
        if not np.isfinite(values).all():
            raise ValueError('Non-finite output coordinates: ' + str(path))
        info.update(minimum=values[:, :3].min(0).tolist(), maximum=values[:, :3].max(0).tolist())
    return info


def run(args):
    source, output = host_path(args.input), host_path(args.output)
    selected = validate_frames(source, args.start, args.end)
    frame_metadata = json.loads((source / 'frames/report.json').read_text())
    if frame_metadata.get('sensor') != args.sensor:
        raise ValueError('The saved sensor profile does not match --sensor')
    if ',' in str(source) or ',' in str(output):
        raise ValueError('Docker mount paths must not contain commas')
    if source == output or source in output.parents or output in source.parents:
        raise ValueError('Input and output must be separate, non-nested directories')
    if output.exists() and any(output.iterdir()):
        raise ValueError('Choose an empty output directory: ' + str(output))
    config_path = host_path(args.config) if args.config else ROOT / f'ros/cub_mapping/params/map_cleaner_{args.sensor}.yaml'
    config = yaml.safe_load(config_path.read_text())
    if not isinstance(config, dict):
        raise ValueError('Invalid MapCleaner YAML')
    if not isinstance(config.get('map_cleaner'), dict):
        raise ValueError('MapCleaner YAML requires a map_cleaner section')
    config['loader'] = dict(pcds_dir='/input/frames/', pose_file='/input/traj_lidar.txt',
                            kitti_calibration_file='', start=args.start, end=selected['end'], format='glim')
    config['map_cleaner'].update(save_dir='/output', frame_id='map', in_process_visualization=False, keep_alive=False)
    inspected = command(['docker', 'image', 'inspect', '--format', '{{.Id}}', IMAGE],
                        stdout=subprocess.PIPE, text=True)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'config.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    provenance = dict(success=False, source=str(source), sensor=args.sensor, upstream_revision=REVISION,
                      image=IMAGE, image_id=inspected.stdout.strip(), frame_saving=frame_metadata,
                      config_source=str(config_path), **selected)
    gnss_path = source / 'gnss_report.json'
    if gnss_path.exists():
        gnss = json.loads(gnss_path.read_text())
        provenance.update(origin_lla=gnss.get('origin_lla'), geoid_height_m=gnss.get('geoid_height_m'),
                          map_frame=gnss.get('map_frame'))
    (output / 'run_report.json').write_text(json.dumps(provenance, indent=2)+'\n')
    # readonly input, isolated ROS master, caller-owned output; no privileged mode or X server.
    argv = ['docker', 'run', '--rm', '--init', '--network', 'none',
            '--user', f'{os.getuid()}:{os.getgid()}', '--env', 'ROS_HOME=/tmp/ros',
            '--env', 'ROS_LOG_DIR=/output/ros_logs', '--env', 'ROS_HOSTNAME=localhost',
            '--env', 'ROS_MASTER_URI=http://localhost:11311', '--env', f'OMP_NUM_THREADS={args.threads}',
            '--mount', f'type=bind,src={source},dst=/input,readonly',
            '--mount', f'type=bind,src={output},dst=/output', IMAGE]
    print('+ ' + shlex.join(argv), flush=True)
    started = time.monotonic()
    try:
        with (output / 'processing.log').open('w') as log:
            with subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True) as process:
                for line in process.stdout:
                    print(line, end='', flush=True); log.write(line); log.flush()
                if process.wait():
                    raise RuntimeError(f'MapCleaner container exited with status {process.returncode}')
        provenance['products'] = {name: pcd_info(output / (name+'.pcd')) for name in PRODUCTS}
        if sum(provenance['products'][name]['points'] for name in ('ground', 'static')) == 0:
            raise ValueError('No usable ground/static output')
        provenance.update(success=True, elapsed_seconds=time.monotonic()-started)
        print('MapCleaner completed: ' + str(output))
    except BaseException as e:
        provenance['error'] = str(e)
        raise
    finally:
        (output / 'run_report.json').write_text(json.dumps(provenance, indent=2)+'\n')


def export(args):
    # Resolve/check all arguments on the host; the C++ exporter rejects existing outputs.
    ground, stationary, prefix = map(host_path, (args.ground, args.static, args.output))
    pcd_info(ground); pcd_info(stationary)
    cub(['ros2', 'run', 'cub_mapping', 'export_clean_map', '--ground', container_path(ground),
         '--static', container_path(stationary), '--output', container_path(prefix)])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    setup_parser = sub.add_parser('setup', help='Build the pinned Noetic image and Cub ROS2 tools')
    setup_parser.add_argument('--skip-ros-build', action='store_true')
    setup_parser.set_defaults(function=setup)
    runner = sub.add_parser('run', help='Run MapCleaner on saved scans and their final poses')
    runner.add_argument('--input', required=True)
    runner.add_argument('--output', required=True)
    runner.add_argument('--sensor', choices=['mid360', 'velodyne'], required=True)
    runner.add_argument('--config', help='Override the sensor YAML; input/output fields are generated')
    runner.add_argument('--start', type=int, default=0)
    runner.add_argument('--end', type=int, default=-1, help='Exclusive frame index; -1 means all')
    runner.add_argument('--threads', type=int, choices=range(1, 65), default=4)
    runner.set_defaults(function=run)
    exporter = sub.add_parser('export', help='Merge ground/static PCD without coordinate changes')
    exporter.add_argument('--ground', required=True)
    exporter.add_argument('--static', required=True)
    exporter.add_argument('--output', required=True, help='Output stem, without extension')
    exporter.set_defaults(function=export)
    args = parser.parse_args()
    try:
        args.function(args)
    except (ValueError, RuntimeError, OSError, KeyError, csv.Error, yaml.YAMLError, subprocess.CalledProcessError) as e:
        parser.exit(1, 'Error: ' + str(e) + '\n')


if __name__ == '__main__':
    main()
