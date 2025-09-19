#!/usr/bin/env python3
"""Distance logging node for Cub robots."""

import csv
import math
import os
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


@dataclass
class Position:
    x: float
    y: float
    z: float


class DistanceLogger(Node):
    def __init__(self) -> None:
        super().__init__('cub_distance_logger')

        self._declare_parameters()

        self.odom_topic: str = self.get_parameter('odom_topic').value
        self.log_directory = Path(self.get_parameter('log_directory').value).expanduser()
        self.min_motion_threshold = float(self.get_parameter('min_motion_threshold_m').value)
        self.max_jump_distance = float(self.get_parameter('max_jump_distance_m').value)
        self.log_interval = float(self.get_parameter('log_interval_s').value)
        self.distance_resolution = float(self.get_parameter('distance_resolution_m').value)
        self.valid_year_threshold = int(self.get_parameter('valid_year_threshold').value)

        self.log_directory.mkdir(parents=True, exist_ok=True)
        self.sessions_dir = self.log_directory / 'sessions'
        self.sessions_dir.mkdir(parents=True, exist_ok=True)
        self.by_day_dir = self.log_directory / 'by_day'
        self.by_day_dir.mkdir(parents=True, exist_ok=True)
        self.pending_dir = self.log_directory / 'pending'
        self.pending_dir.mkdir(parents=True, exist_ok=True)

        self.start_monotonic = time.monotonic()
        self.session_identifier = f"session_{int(self.start_monotonic)}_{uuid.uuid4().hex[:6]}"
        self.session_path = self.sessions_dir / self.session_identifier
        self.session_path.mkdir(parents=True, exist_ok=True)
        self.session_file = self.session_path / 'distance_log.csv'
        self.pending_file = self.pending_dir / f'{self.session_identifier}.csv'

        self.fieldnames = [
            'record_index',
            'event',
            'session_id',
            'ros_stamp',
            'wall_time_iso',
            'system_time_valid',
            'monotonic_time_s',
            'session_distance_m',
            'distance_increment_m',
            'distance_increment_xy_m',
            'position_x',
            'position_y',
            'position_z',
            'frame_id',
            'notes',
        ]

        self.record_index = 0
        self.session_distance = 0.0
        self.pending_distance = 0.0
        self.pending_distance_xy = 0.0
        self.last_log_time = self.start_monotonic
        self.last_position: Optional[Position] = None
        self.last_msg: Optional[Odometry] = None
        self._shutdown_complete = False

        self._log_session_marker('start')

        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odometry,
            10,
        )

        self.get_logger().info(
            f"Distance logger started. Logging to {self.log_directory}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('log_directory', str(Path.home() / 'distance_logs'))
        self.declare_parameter('min_motion_threshold_m', 0.005)
        self.declare_parameter('max_jump_distance_m', 5.0)
        self.declare_parameter('log_interval_s', 1.0)
        self.declare_parameter('distance_resolution_m', 0.05)
        self.declare_parameter('valid_year_threshold', 2023)

    def _on_odometry(self, msg: Odometry) -> None:
        position = Position(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )

        now_monotonic = time.monotonic()
        if self.last_position is None:
            self.last_position = position
            self.last_msg = msg
            return

        distance_xy = math.hypot(
            position.x - self.last_position.x,
            position.y - self.last_position.y,
        )
        distance_3d = math.sqrt(
            (position.x - self.last_position.x) ** 2
            + (position.y - self.last_position.y) ** 2
            + (position.z - self.last_position.z) ** 2
        )

        if distance_3d < self.min_motion_threshold:
            self.last_position = position
            self.last_msg = msg
            return

        if distance_3d > self.max_jump_distance:
            note = f'ignored_jump:{distance_3d:.3f}m'
            self._emit_record(msg, 0.0, 0.0, event='reset', note=note)
            self.last_position = position
            self.last_msg = msg
            self.pending_distance = 0.0
            self.pending_distance_xy = 0.0
            self.last_log_time = now_monotonic
            return

        self.session_distance += distance_3d
        self.pending_distance += distance_3d
        self.pending_distance_xy += distance_xy
        self.last_position = position
        self.last_msg = msg

        should_log = (
            self.pending_distance >= self.distance_resolution
            or (now_monotonic - self.last_log_time) >= self.log_interval
        )

        if should_log:
            self._emit_record(msg, self.pending_distance, self.pending_distance_xy, event='data')
            self.pending_distance = 0.0
            self.pending_distance_xy = 0.0
            self.last_log_time = now_monotonic

    def _emit_record(
        self,
        msg: Optional[Odometry],
        distance_increment: float,
        distance_increment_xy: float,
        event: str,
        note: str = '',
    ) -> None:
        wall_dt = datetime.now(timezone.utc).astimezone()
        system_time_valid = wall_dt.year >= self.valid_year_threshold
        monotonic_since_start = time.monotonic() - self.start_monotonic

        ros_stamp = ''
        frame_id = ''
        position_x: Optional[float] = None
        position_y: Optional[float] = None
        position_z: Optional[float] = None

        if msg is not None:
            ros_stamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
            frame_id = msg.header.frame_id
            position_x = msg.pose.pose.position.x
            position_y = msg.pose.pose.position.y
            position_z = msg.pose.pose.position.z

        row = {
            'record_index': self.record_index,
            'event': event,
            'session_id': self.session_identifier,
            'ros_stamp': ros_stamp,
            'wall_time_iso': wall_dt.isoformat(),
            'system_time_valid': int(system_time_valid),
            'monotonic_time_s': f"{monotonic_since_start:.3f}",
            'session_distance_m': f"{self.session_distance:.4f}",
            'distance_increment_m': f"{distance_increment:.4f}",
            'distance_increment_xy_m': f"{distance_increment_xy:.4f}",
            'position_x': '' if position_x is None else f"{position_x:.4f}",
            'position_y': '' if position_y is None else f"{position_y:.4f}",
            'position_z': '' if position_z is None else f"{position_z:.4f}",
            'frame_id': frame_id,
            'notes': note,
        }

        self._append_to_csv(self.session_file, row)
        if event == 'data':
            if system_time_valid:
                bucket_file = self._daily_bucket_file(wall_dt)
            else:
                bucket_file = self.pending_file
            self._append_to_csv(bucket_file, row)

        self.record_index += 1

    def _daily_bucket_file(self, wall_dt: datetime) -> Path:
        local_dt = wall_dt.astimezone()
        day_dir = self.by_day_dir / local_dt.strftime('%Y-%m-%d')
        day_dir.mkdir(parents=True, exist_ok=True)
        filename = f"{local_dt.strftime('%H')}.csv"
        return day_dir / filename

    def _append_to_csv(self, path: Path, row: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        file_exists = path.exists()
        with open(path, 'a', newline='') as file_handle:
            writer = csv.DictWriter(file_handle, fieldnames=self.fieldnames)
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)
            file_handle.flush()
            os.fsync(file_handle.fileno())

    def _log_session_marker(self, event: str) -> None:
        wall_dt = datetime.now(timezone.utc).astimezone()
        row = {
            'record_index': self.record_index,
            'event': event,
            'session_id': self.session_identifier,
            'ros_stamp': '',
            'wall_time_iso': wall_dt.isoformat(),
            'system_time_valid': int(wall_dt.year >= self.valid_year_threshold),
            'monotonic_time_s': f"{time.monotonic() - self.start_monotonic:.3f}",
            'session_distance_m': f"{self.session_distance:.4f}",
            'distance_increment_m': '0.0000',
            'distance_increment_xy_m': '0.0000',
            'position_x': '',
            'position_y': '',
            'position_z': '',
            'frame_id': '',
            'notes': '',
        }
        self._append_to_csv(self.session_file, row)
        self.record_index += 1

    def finalize(self, note: str = 'flush_on_shutdown') -> None:
        if self._shutdown_complete:
            return
        self._shutdown_complete = True
        if self.pending_distance > 0.0 and self.last_msg is not None:
            self._emit_record(
                self.last_msg,
                self.pending_distance,
                self.pending_distance_xy,
                event='data',
                note=note,
            )
            self.pending_distance = 0.0
            self.pending_distance_xy = 0.0
        self._log_session_marker('stop')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DistanceLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


### 走行距離の記録と集計
# - Cub3/mCubの`launch_at_boot.launch.py`では、ホイールオドメトリ(`/odom`)から走行距離を積算してCSVに保存する`cub_bringup`パッケージの`distance_logger`ノードが自動的に起動する。
# - 走行ログはDockerコンテナ内のホームディレクトリにある`~/distance_logs/`以下へ保存される(ホスト側では`docker/home/distance_logs/`)。
#   - `sessions/<session_id>/distance_log.csv`: 起動毎の全ログ。`event`列が`data`の行の`distance_increment_m`を積算するとその起動で移動した距離になる。
#   - `by_day/YYYY-MM-DD/HH.csv`: システム時刻が正しいと判断できた時点以降のログを日付・時刻ごとに分割したもの。`system_time_valid`列が`1`になっている行のみが格納される。
#   - `pending/<session_id>.csv`: RTCが無く時刻が1970年付近で始まる場合など、時刻が信用できない間の走行ログ。後から日付が分かったら適宜整理する。
# - 書き込み毎に`fsync`しているため、記録中に電源が切れてもこれまで書かれた距離データは残る。
# - 日毎の移動距離合計をざっと確認したい場合の例:
#   ```python
#   import csv
#   from pathlib import Path

#   root = Path.home() / 'distance_logs' / 'by_day'
#   for day_dir in sorted(root.glob('*')):
#       total = 0.0
#       for csv_path in sorted(day_dir.glob('*.csv')):
#           with csv_path.open() as f:
#               reader = csv.DictReader(f)
#               total += sum(float(row['distance_increment_m']) for row in reader if row['event'] == 'data')
#       print(f"{day_dir.name}: {total:.2f} m")
#   ```
#   `by_day`が空の場合は`pending`配下のCSVから必要な期間を手動で選んで同様に集計する。
