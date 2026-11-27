#!/usr/bin/env python3
# Copyright 2026 CoderDojo Musashikosugi / Cub_ROS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Import backend library from submodule or installed package
try:
    from cub_um982.extended_client import AntennaGgaState, ExtendedUM982Client, NtripStats
    from cub_um982.usb_reset import UsbDeviceInfo, reset_usb_device
    from um982.types import PositionData
except ImportError:
    import sys
    from pathlib import Path

    # Candidates for locating thirdparty/UM982-RTK-GPS-Library
    current_dir = Path(__file__).resolve().parent
    candidates = [
        current_dir.parent / 'thirdparty' / 'UM982-RTK-GPS-Library',
        current_dir / 'thirdparty' / 'UM982-RTK-GPS-Library',
    ]

    try:
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = Path(get_package_prefix('cub_um982'))
        ws_src = pkg_prefix.parent.parent / 'src'
        candidates.extend([
            ws_src / 'cub' / 'cub_um982' / 'thirdparty' / 'UM982-RTK-GPS-Library',
            ws_src / 'cub_um982' / 'thirdparty' / 'UM982-RTK-GPS-Library',
        ])
    except Exception:
        pass

    for candidate in candidates:
        if candidate.exists() and (candidate / 'um982').exists():
            sys.path.insert(0, str(candidate))
            break

    from cub_um982.extended_client import AntennaGgaState, ExtendedUM982Client, NtripStats
    from cub_um982.usb_reset import UsbDeviceInfo, reset_usb_device
    from um982.types import PositionData


def heading_pitch_to_quaternion(
    heading_deg: float,
    pitch_deg: float,
    heading_offset_deg: float = 0.0,
    pitch_offset_deg: float = 0.0,
):
    """
    Convert UM982 heading (clock-wise from True North, 0..360 deg) and pitch (deg)
    to ROS ENU quaternion (counter-clockwise from East).

    ENU:
      East  = 0 rad (0 deg)
      North = pi/2 rad (90 deg)
      West  = pi rad (180 deg)
      South = -pi/2 rad (-90 deg)
    """
    effective_heading = (heading_deg + heading_offset_deg) % 360.0
    effective_pitch = pitch_deg + pitch_offset_deg

    # Heading (0=North, 90=East) -> ENU Yaw (0=East, 90=North)
    yaw_deg = 90.0 - effective_heading
    yaw_deg = (yaw_deg + 180.0) % 360.0 - 180.0

    yaw_rad = math.radians(yaw_deg)
    pitch_rad = math.radians(effective_pitch)

    # Roll is assumed 0 (dual-antenna cannot measure roll)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)

    qx = -sp * sy
    qy = sp * cy
    qz = cp * sy
    qw = cp * cy

    return qx, qy, qz, qw


def build_navsatfix_message(
    pos: PositionData,
    frame_id: str,
    stamp,
) -> NavSatFix:
    """Build NavSatFix message from PositionData."""
    fix_msg = NavSatFix()
    fix_msg.header.stamp = stamp
    fix_msg.header.frame_id = frame_id

    fix_msg.latitude = float(pos.lat)
    fix_msg.longitude = float(pos.lon)
    fix_msg.altitude = float(pos.alt) if pos.alt is not None else 0.0

    # Map RTK state to NavSatStatus
    if pos.rtk_state == 'rtk_fix':
        fix_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
    elif pos.rtk_state in ('rtk_float', 'dgps'):
        fix_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
    elif pos.rtk_state == 'standalone':
        fix_msg.status.status = NavSatStatus.STATUS_FIX
    else:
        fix_msg.status.status = NavSatStatus.STATUS_NO_FIX

    fix_msg.status.service = (
        NavSatStatus.SERVICE_GPS
        | NavSatStatus.SERVICE_GLONASS
        | NavSatStatus.SERVICE_COMPASS
        | NavSatStatus.SERVICE_GALILEO
    )

    # Approximate horizontal covariance using HDOP if available
    hdop = pos.hdop if (pos.hdop is not None and pos.hdop > 0) else 1.0
    if pos.rtk_state == 'rtk_fix':
        h_acc = 0.02 * hdop  # ~2cm base
    elif pos.rtk_state == 'rtk_float':
        h_acc = 0.3 * hdop   # ~30cm base
    elif pos.rtk_state == 'dgps':
        h_acc = 1.0 * hdop
    else:
        h_acc = 3.0 * hdop

    pos_cov = [0.0] * 9
    pos_cov[0] = h_acc ** 2
    pos_cov[4] = h_acc ** 2
    pos_cov[8] = (2.0 * h_acc) ** 2
    fix_msg.position_covariance = pos_cov
    fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

    return fix_msg


def _fmt(value, fmt: str) -> str:
    return fmt.format(value) if value is not None else "N/A"


def classify_antenna_status(
    link_problem: Optional[str],
    gga_state: Optional[AntennaGgaState],
    now: float,
    timeout: float,
    sentence: str = 'GGA',
) -> Tuple[bytes, str]:
    """
    Decide the diagnostic level of one antenna.

    - STALE: cannot communicate with the UM982 (link_problem is set)
    - ERROR: communicating, but no {sentence} output or no position (no coordinates)
    - WARN:  position available but not RTK Fix
    - OK:    RTK Fix

    Args:
        link_problem: Why the receiver cannot be communicated with, or None.
        gga_state: Latest GGA/GGAH state of the antenna, or None if never received.
        now: Current time.monotonic().
        timeout: Seconds after which the GGA/GGAH state is considered outdated.
        sentence: Sentence name used in messages ('GGA' or 'GGAH').
    """
    if link_problem is not None:
        return DiagnosticStatus.STALE, f"No communication with UM982 ({link_problem})"
    if gga_state is None or now - gga_state.time > timeout:
        return (
            DiagnosticStatus.ERROR,
            f"No {sentence} output from UM982; check receiver output configuration",
        )
    if not gga_state.has_position:
        return DiagnosticStatus.ERROR, "No position fix; check antenna sky view / cabling"
    if gga_state.rtk_state == 'rtk_fix':
        return DiagnosticStatus.OK, "RTK Fix"
    return DiagnosticStatus.WARN, f"Position available, not RTK Fix ({gga_state.rtk_state})"


class UM982Node(Node):
    """ROS2 driver node for Unicore UM982 GNSS receiver."""

    def __init__(self):
        super().__init__('um982_node')

        # Serial / Hardware parameters
        self.port = self.declare_parameter('port', '/dev/ttyGPS').value
        self.baudrate = self.declare_parameter('baudrate', 115200).value
        self.serial_timeout = self.declare_parameter('serial_timeout', 1.0).value
        self.output_rate = self.declare_parameter('output_rate', 10).value
        self.configure_on_start = self.declare_parameter('configure_on_start', True).value
        self.enable_rmc = self.declare_parameter('enable_rmc', False).value
        self.write_timeout = self.declare_parameter('write_timeout', 1.0).value

        # Link recovery parameters
        self.serial_watchdog_timeout = self.declare_parameter('serial_watchdog_timeout', 5.0).value
        self.usb_reset_enabled = self.declare_parameter('usb_reset_enabled', True).value
        self.usb_reset_after_failures = self.declare_parameter('usb_reset_after_failures', 2).value
        self.usb_reset_min_interval = self.declare_parameter('usb_reset_min_interval', 30.0).value

        # RTK / NTRIP parameters
        self.enable_ntrip = self.declare_parameter('enable_ntrip', False).value
        self.ntrip_host = self.declare_parameter('ntrip_host', '').value
        self.ntrip_port = self.declare_parameter('ntrip_port', 2101).value
        self.ntrip_mountpoint = self.declare_parameter('ntrip_mountpoint', '').value
        self.ntrip_user = self.declare_parameter('ntrip_user', '').value
        self.ntrip_password = self.declare_parameter('ntrip_password', '').value
        self.ntrip_gga_interval = self.declare_parameter('ntrip_gga_interval', 5.0).value
        self.ntrip_log_interval = self.declare_parameter('ntrip_log_interval', 5.0).value

        # Frame and offset parameters
        self.frame_id = self.declare_parameter('frame_id', 'gnss_link').value
        self.secondary_frame_id = self.declare_parameter(
            'secondary_frame_id', 'gnss_secondary_link'
        ).value
        self.publish_secondary_fix = self.declare_parameter(
            'publish_secondary_fix', True
        ).value
        self.heading_offset_deg = self.declare_parameter('heading_offset_deg', 0.0).value
        self.pitch_offset_deg = self.declare_parameter('pitch_offset_deg', 0.0).value
        self.default_heading_stddev_deg = self.declare_parameter('default_heading_stddev_deg', 0.5).value
        self.default_pitch_stddev_deg = self.declare_parameter('default_pitch_stddev_deg', 0.5).value

        # Diagnostics parameters
        self.hardware_id = self.declare_parameter('hardware_id', 'um982').value
        self.diagnostic_update_period = self.declare_parameter('diagnostic_update_period', 1.0).value
        self.diagnostic_timeout = self.declare_parameter('diagnostic_timeout', 2.0).value
        self.diagnostic_name_prefix = self.declare_parameter('diagnostic_name_prefix', 'UM982').value

        # Publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '~/pose', 10)
        self.fix_pub = self.create_publisher(NavSatFix, '~/fix', 10)
        self.fix_secondary_pub = self.create_publisher(NavSatFix, '~/fix_secondary', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Internal state
        self._lock = threading.Lock()
        self._last_pos: Optional[PositionData] = None
        self._prev_rtk_state: str = 'unknown'
        self._connection_error_msg: Optional[str] = None
        self.client: Optional[ExtendedUM982Client] = None

        # Link recovery state
        self._consecutive_failures = 0
        self._reconnect_count = 0
        self._usb_reset_count = 0
        self._last_usb_reset_time: Optional[float] = None  # time.monotonic()
        self._last_link_error: Optional[str] = None

        # Start diagnostic timer
        self.diag_timer = self.create_timer(
            self.diagnostic_update_period, self._publish_diagnostics
        )

        # Periodic NTRIP stats log timer
        if self.enable_ntrip and self.ntrip_log_interval > 0:
            self.ntrip_log_timer = self.create_timer(
                self.ntrip_log_interval, self._log_ntrip_stats
            )
        else:
            self.ntrip_log_timer = None

        # Connection check / retry timer
        self._try_connect()
        self.connect_timer = self.create_timer(2.0, self._check_connection)

    def _on_ntrip_log(self, level: str, msg: str):
        """Callback for NTRIP events from ExtendedNtripClient."""
        lvl = level.lower()
        if lvl == 'info':
            self.get_logger().info(f"[NTRIP] {msg}")
        elif lvl == 'warn':
            self.get_logger().warn(f"[NTRIP] {msg}")
        elif lvl == 'error':
            self.get_logger().error(f"[NTRIP] {msg}")
        else:
            self.get_logger().debug(f"[NTRIP] {msg}")

    def _try_connect(self):
        """Attempt to open and initialize connection to UM982."""
        if self.client is not None:
            return

        self.get_logger().info(
            f"Connecting to UM982 on {self.port} at {self.baudrate} baud (rate: {self.output_rate} Hz)..."
        )
        client = None
        try:
            client = ExtendedUM982Client(
                port=self.port,
                baud=self.baudrate,
                timeout=self.serial_timeout,
                output_rate=self.output_rate,
                write_timeout=self.write_timeout if self.write_timeout > 0 else None,
            )
            client.set_position_callback(self._on_position_callback)
            client.set_secondary_position_callback(self._on_secondary_position_callback)
            client.set_ntrip_log_callback(self._on_ntrip_log)
            client.start(configure=self.configure_on_start)

            if self.enable_rmc:
                client.set_output_rate(self.output_rate, enable_rmc=True)

            self.get_logger().info("UM982 serial reader started successfully.")

            # Start NTRIP client if enabled
            if self.enable_ntrip:
                if not self.ntrip_host or not self.ntrip_mountpoint:
                    self.get_logger().error(
                        "NTRIP is enabled but host or mountpoint is empty. Skipping NTRIP."
                    )
                else:
                    self.get_logger().info(
                        f"Starting NTRIP client ({self.ntrip_host}:{self.ntrip_port}/{self.ntrip_mountpoint})..."
                    )
                    client.start_ntrip(
                        host=self.ntrip_host,
                        port=self.ntrip_port,
                        mountpoint=self.ntrip_mountpoint,
                        user=self.ntrip_user,
                        password=self.ntrip_password,
                        gga_interval=self.ntrip_gga_interval,
                    )

            self.client = client
            self._connection_error_msg = None
        except Exception as e:
            self._connection_error_msg = f"Failed to connect to {self.port}: {e}"
            self.get_logger().warn(f"{self._connection_error_msg}. Will retry...")
            # Make sure partially started threads/serial port are released
            # so that retries do not accumulate reader threads.
            if client is not None:
                usb_device = client.usb_device
                self._stop_client(client)
                # The port opened but initialization failed (e.g. configuration
                # write timed out on a stalled link): count it as a link failure
                # so that USB reset escalation also covers this case.
                if usb_device is not None:
                    self._consecutive_failures += 1
                    self._last_link_error = str(e)
                    self._maybe_reset_usb(usb_device)

    def _check_connection(self):
        """Supervise the serial link: detect failures, recover, or retry connecting."""
        client = self.client
        if client is None:
            self._try_connect()
            return

        problem = client.get_health_problem(self.serial_watchdog_timeout)
        if problem is None:
            if self._consecutive_failures and client.last_line_time is not None:
                self.get_logger().info(
                    f"UM982 link recovered after {self._consecutive_failures} failure(s)."
                )
                self._consecutive_failures = 0
            return

        self._handle_link_failure(client, problem)

    def _handle_link_failure(self, client: ExtendedUM982Client, reason: str):
        """Tear down a broken client; the next timer tick reconnects."""
        self._consecutive_failures += 1
        self._reconnect_count += 1
        self._last_link_error = reason
        self._connection_error_msg = (
            f"UM982 link lost ({reason}); reconnecting "
            f"(consecutive failures: {self._consecutive_failures})"
        )
        self.get_logger().error(f"{self._connection_error_msg}. Closing {self.port}...")

        usb_device = client.usb_device
        self.client = None
        self._stop_client(client)
        self._maybe_reset_usb(usb_device)

    def _stop_client(self, client: ExtendedUM982Client):
        """
        Stop a client. If its threads cannot be stopped (e.g. stuck in a system
        call), exit the process so that launch respawns a clean node.
        """
        try:
            stopped = client.stop()
        except Exception as e:
            self.get_logger().warn(f"Error while stopping UM982 client: {e}")
            stopped = True
        if not stopped:
            self.get_logger().fatal(
                "UM982 client threads did not terminate; exiting so the node can be respawned."
            )
            raise SystemExit(1)

    def _maybe_reset_usb(self, usb_device: Optional[UsbDeviceInfo]):
        """Escalate to a USB port reset after repeated link failures."""
        if not self.usb_reset_enabled or usb_device is None:
            return
        if self._consecutive_failures < self.usb_reset_after_failures:
            return
        now = time.monotonic()
        if (self._last_usb_reset_time is not None
                and now - self._last_usb_reset_time < self.usb_reset_min_interval):
            self.get_logger().info(
                f"Skipping USB reset of {usb_device.describe()}: last reset was "
                f"{now - self._last_usb_reset_time:.0f}s ago "
                f"(min interval {self.usb_reset_min_interval:.0f}s)."
            )
            return

        self._last_usb_reset_time = now
        self.get_logger().warn(
            f"Resetting USB device {usb_device.describe()} after "
            f"{self._consecutive_failures} consecutive link failures..."
        )
        try:
            node = reset_usb_device(usb_device)
            self._usb_reset_count += 1
            self.get_logger().warn(f"USB reset of {node} succeeded; waiting for re-enumeration.")
        except Exception as e:
            self.get_logger().error(f"USB reset of {usb_device.describe()} failed: {e}")

    def _on_position_callback(self, pos: PositionData):
        """Called when a new primary antenna position/heading packet is parsed."""
        now = self.get_clock().now()

        # Track RTK state transitions
        if pos.rtk_state != self._prev_rtk_state:
            sats_str = f"{pos.num_sats}" if pos.num_sats is not None else "?"
            hdop_str = f"{pos.hdop:.2f}" if pos.hdop is not None else "?"
            diff_str = f"{pos.diff_age:.1f}s" if pos.diff_age is not None else "N/A"
            self.get_logger().info(
                f"[RTK Status Changed] {self._prev_rtk_state} -> {pos.rtk_state} "
                f"(Sats: {sats_str}, HDOP: {hdop_str}, DiffAge: {diff_str})"
            )
            if pos.rtk_state == 'rtk_fix':
                self.get_logger().info("★★★ [RTK Status] RTK FIX ACQUIRED! High-precision cm-level solution active. ★★★")
            self._prev_rtk_state = pos.rtk_state

        with self._lock:
            self._last_pos = pos

        now_msg = now.to_msg()

        # 1. Publish PoseWithCovarianceStamped (Orientation & Covariance)
        if pos.heading is not None and not math.isnan(pos.heading):
            pitch_val = pos.pitch if (pos.pitch is not None and not math.isnan(pos.pitch)) else 0.0

            qx, qy, qz, qw = heading_pitch_to_quaternion(
                heading_deg=pos.heading,
                pitch_deg=pitch_val,
                heading_offset_deg=self.heading_offset_deg,
                pitch_offset_deg=self.pitch_offset_deg,
            )

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = now_msg
            pose_msg.header.frame_id = self.frame_id

            pose_msg.pose.pose.orientation.x = qx
            pose_msg.pose.pose.orientation.y = qy
            pose_msg.pose.pose.orientation.z = qz
            pose_msg.pose.pose.orientation.w = qw

            cov = [0.0] * 36
            cov[0] = 1e6
            cov[7] = 1e6
            cov[14] = 1e6
            cov[21] = 1e6

            if pos.pitch_stddev is not None and not math.isnan(pos.pitch_stddev) and pos.pitch_stddev > 0:
                cov[28] = math.radians(pos.pitch_stddev) ** 2
            else:
                cov[28] = math.radians(self.default_pitch_stddev_deg) ** 2

            if pos.heading_stddev is not None and not math.isnan(pos.heading_stddev) and pos.heading_stddev > 0:
                cov[35] = math.radians(pos.heading_stddev) ** 2
            else:
                cov[35] = math.radians(self.default_heading_stddev_deg) ** 2

            pose_msg.pose.covariance = cov
            self.pose_pub.publish(pose_msg)

        # 2. Publish NavSatFix for Primary Antenna
        if pos.is_valid:
            fix_msg = build_navsatfix_message(pos, self.frame_id, now_msg)
            self.fix_pub.publish(fix_msg)

    def _on_secondary_position_callback(self, pos: PositionData):
        """Called when a new secondary/slave antenna position packet is parsed."""
        now = self.get_clock().now()

        if self.publish_secondary_fix and pos.is_valid:
            fix_msg = build_navsatfix_message(pos, self.secondary_frame_id, now.to_msg())
            self.fix_secondary_pub.publish(fix_msg)

    def _log_ntrip_stats(self):
        """Periodically log NTRIP connection & RTCM transfer summary."""
        if not self.client:
            return
        stats = self.client.get_ntrip_stats()
        if not stats.connected:
            err_msg = f" (Error: {stats.last_error})" if stats.last_error else ""
            self.get_logger().info(f"[NTRIP Status] Disconnected / Connecting to {stats.host}:{stats.port}{err_msg}")
            return

        with self._lock:
            primary_pos = self._last_pos

        rtk_state = primary_pos.rtk_state if primary_pos else 'no_data'
        if primary_pos is None and self._receiver_has_no_fix():
            rtk_state = (
                f"NO POSITION FIX (receiver outputs empty GGA x{self.client.no_fix_count}; "
                f"check antenna sky view/cabling)"
            )
        diff_age = f"{primary_pos.diff_age:.1f}s" if primary_pos and primary_pos.diff_age is not None else "N/A"
        rx_kb = stats.total_rx_bytes / 1024.0
        written_kb = stats.total_written_bytes / 1024.0
        rate_kbs = stats.rx_rate_bps / 1024.0

        types_list = sorted(stats.recent_rtcm_types.keys())
        types_str = f"{types_list}" if types_list else "None yet"

        last_gga_sec = f"{time.time() - stats.last_gga_sent_time:.1f}s ago" if stats.last_gga_sent_time else "Never"
        last_rx_sec = f"{time.time() - stats.last_rx_time:.1f}s ago" if stats.last_rx_time else "Never"

        self.get_logger().info(
            f"[NTRIP Status] Connected: Yes | Rx: {rx_kb:.1f} KB ({rate_kbs:.1f} KB/s, last: {last_rx_sec}) | "
            f"Serial written: {written_kb:.1f} KB (errors: {stats.write_errors}) | "
            f"RTCM Types: {types_str} (CRC errors: {stats.rtcm_crc_errors}) | "
            f"GGA sent: {last_gga_sec} | RTK State: {rtk_state} (Diff Age: {diff_age})"
        )

    def _receiver_has_no_fix(self) -> bool:
        """True if the receiver is alive but recently reported empty (no-fix) GGA."""
        client = self.client
        if client is None or client.last_no_fix_time is None:
            return False
        return (time.time() - client.last_no_fix_time) <= max(self.diagnostic_timeout, 2.0)

    def _link_problem(self, now: float) -> Optional[str]:
        """Return why the UM982 cannot be communicated with, or None if data is flowing."""
        client = self.client
        if client is None:
            return self._connection_error_msg or "not connected"
        problem = client.get_health_problem(self.serial_watchdog_timeout)
        if problem is not None:
            return problem
        if client.last_line_time is None:
            return "waiting for first data"
        age = now - client.last_line_time
        if age > self.diagnostic_timeout:
            return f"no data for {age:.1f}s"
        return None

    def _build_antenna_status(
        self, name: str, secondary: bool, link_problem: Optional[str], now: float
    ) -> DiagnosticStatus:
        """Build the diagnostic status of one antenna (main and secondary share the same rules)."""
        client = self.client
        state = client.get_gga_state(secondary) if client else None
        level, message = classify_antenna_status(
            link_problem, state, now, self.diagnostic_timeout,
            'GGAH' if secondary else 'GGA',
        )
        gga = state.gga if state else None
        # Heading is measured by the antenna pair (latest HEADINGA)
        pos = client.get_position() if client else None
        no_fix_count = (client.secondary_no_fix_count if secondary else client.no_fix_count) if client else 0
        line_age = now - client.last_line_time if client and client.last_line_time is not None else None

        stat = DiagnosticStatus()
        stat.name = name
        stat.hardware_id = self.hardware_id
        stat.level = level
        stat.message = message
        stat.values = [
            KeyValue(key="RTK State", value=state.rtk_state if state else "N/A"),
            KeyValue(key="Has Position", value=str(state.has_position if state else False)),
            KeyValue(key="Satellites", value=_fmt(gga.num_sats if gga else None, "{}")),
            KeyValue(key="HDOP", value=_fmt(gga.hdop if gga else None, "{:.2f}")),
            KeyValue(key="Correction Age (s)", value=_fmt(gga.diff_age if gga else None, "{:.1f}")),
            KeyValue(key="Latitude", value=_fmt(gga.lat if gga else None, "{:.8f}")),
            KeyValue(key="Longitude", value=_fmt(gga.lon if gga else None, "{:.8f}")),
            KeyValue(key="Altitude (m)", value=_fmt(gga.alt if gga else None, "{:.2f}")),
            KeyValue(key="Heading (deg)", value=_fmt(pos.heading if pos else None, "{:.2f}")),
            KeyValue(key="Heading StdDev (deg)", value=_fmt(pos.heading_stddev if pos else None, "{:.3f}")),
            KeyValue(key="Pitch (deg)", value=_fmt(pos.pitch if pos else None, "{:.2f}")),
            KeyValue(key="Pitch StdDev (deg)", value=_fmt(pos.pitch_stddev if pos else None, "{:.3f}")),
            KeyValue(key="Baseline (m)", value=_fmt(pos.baseline_m if pos else None, "{:.3f}")),
            KeyValue(key="No Fix Count", value=str(no_fix_count)),
            KeyValue(key="Last GGA Age (s)", value=_fmt(now - state.time if state else None, "{:.2f}")),
            KeyValue(key="Last Line Age (s)", value=_fmt(line_age, "{:.2f}")),
            KeyValue(key="Serial Reconnects", value=str(self._reconnect_count)),
            KeyValue(key="USB Resets", value=str(self._usb_reset_count)),
            KeyValue(key="Last Link Error", value=str(self._last_link_error or "None")),
        ]
        return stat

    def _publish_diagnostics(self):
        """Periodically publish diagnostics compatible with cub_diagnostics."""
        now = time.monotonic()
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # ----------------------------------------------------
        # 1. Antenna Diagnostics (main / secondary use the same rules)
        # ----------------------------------------------------
        link_problem = self._link_problem(now)
        diag_array.status.append(self._build_antenna_status(
            f"{self.diagnostic_name_prefix} Main Antenna", False, link_problem, now))
        if self.publish_secondary_fix:
            diag_array.status.append(self._build_antenna_status(
                f"{self.diagnostic_name_prefix} Secondary Antenna", True, link_problem, now))

        # ----------------------------------------------------
        # 2. NTRIP & RTCM Diagnostics (if enabled)
        # ----------------------------------------------------
        if self.enable_ntrip and self.client:
            ntrip_stat = DiagnosticStatus()
            ntrip_stat.name = f"{self.diagnostic_name_prefix} NTRIP"
            ntrip_stat.hardware_id = self.hardware_id

            stats = self.client.get_ntrip_stats()
            time_now = time.time()

            if not stats.connected:
                ntrip_stat.level = DiagnosticStatus.ERROR
                ntrip_stat.message = f"Disconnected from caster ({stats.last_error or 'Reconnecting...'})"
            elif stats.total_rx_bytes == 0:
                ntrip_stat.level = DiagnosticStatus.WARN
                ntrip_stat.message = "Connected to caster, waiting for RTCM data..."
            else:
                sec_since_rx = time_now - stats.last_rx_time if stats.last_rx_time else 999.0
                if sec_since_rx > 10.0:
                    ntrip_stat.level = DiagnosticStatus.WARN
                    ntrip_stat.message = f"RTCM stream stalled ({sec_since_rx:.1f}s since last packet)"
                else:
                    ntrip_stat.level = DiagnosticStatus.OK
                    ntrip_stat.message = "Receiving RTCM correction stream normally"

            types_summary = ", ".join(
                f"{k}({v})" for k, v in sorted(stats.recent_rtcm_types.items())
            ) or "None"

            gga_age_str = f"{time_now - stats.last_gga_sent_time:.1f}s ago" if stats.last_gga_sent_time else "Never"
            rtcm_age_str = f"{time_now - stats.last_rx_time:.1f}s ago" if stats.last_rx_time else "Never"

            ntrip_stat.values = [
                KeyValue(key="Connected", value=str(stats.connected)),
                KeyValue(key="Caster Host", value=f"{stats.host}:{stats.port}"),
                KeyValue(key="Mountpoint", value=stats.mountpoint),
                KeyValue(key="Caster Response", value=str(stats.caster_response or "N/A")),
                KeyValue(key="Total Received (KB)", value=f"{stats.total_rx_bytes / 1024.0:.2f}"),
                KeyValue(key="Total Serial Written (KB)", value=f"{stats.total_written_bytes / 1024.0:.2f}"),
                KeyValue(key="Serial Write Errors", value=str(stats.write_errors)),
                KeyValue(key="RTCM CRC Errors", value=str(stats.rtcm_crc_errors)),
                KeyValue(key="Receive Rate (KB/s)", value=f"{stats.rx_rate_bps / 1024.0:.2f}"),
                KeyValue(key="Last RTCM Received", value=rtcm_age_str),
                KeyValue(key="Last GGA Sent", value=gga_age_str),
                KeyValue(key="Last GGA Quality", value=str(stats.last_gga_quality if stats.last_gga_quality is not None else "N/A")),
                KeyValue(key="Detected RTCM Types", value=types_summary),
                KeyValue(key="Last Error", value=str(stats.last_error or "None")),
            ]
            diag_array.status.append(ntrip_stat)

        self.diag_pub.publish(diag_array)

    def destroy_node(self):
        """Cleanup upon node termination."""
        if hasattr(self, 'client') and self.client is not None:
            if rclpy.ok():
                self.get_logger().info("Stopping UM982 client...")
            self.client.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UM982Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # The launch system's SIGINT may already have shut down the context
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
