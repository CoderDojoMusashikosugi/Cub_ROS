#!/usr/bin/env python3
"""
extended_client.py

Extended UM982 client and NTRIP client without modifying thirdparty submodule.
- Distinguishes primary antenna (GGA) from secondary/slave antenna (GGAH).
- Provides dedicated callbacks and position objects for both antennas.
- Monitors and logs NTRIP connection, RTCM reception, packet types, and serial writes.
"""

import base64
import logging
import math
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

# Try importing base client from submodule or workspace
try:
    from um982.client import UM982Client
    from um982.nmea import (
        determine_rtk_state,
        gps_week_seconds_to_unix,
        parse_gga,
        parse_gpsutc,
        parse_rectime,
        parse_rmc,
        parse_uniheading,
    )
    from um982.types import GGAData, PositionData, UniheadingData
except ImportError:
    from pathlib import Path
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

    from um982.client import UM982Client
    from um982.nmea import (
        determine_rtk_state,
        gps_week_seconds_to_unix,
        parse_gga,
        parse_gpsutc,
        parse_rectime,
        parse_rmc,
        parse_uniheading,
    )
    from um982.types import GGAData, PositionData, UniheadingData

logger = logging.getLogger(__name__)

# Minimum interval between repeated serial write error logs (seconds)
WRITE_ERROR_LOG_INTERVAL = 5.0


def parse_receiver_heading(sentence: str, leap_seconds: int) -> Optional[UniheadingData]:
    """Parse Unicore week/ms headers and compatible HEADINGA week/seconds headers."""
    header, separator, body = sentence.partition(';')
    header_fields = header.split(',')
    if not separator or header_fields[0] not in ('#HEADINGA', '#UNIHEADINGA'):
        return None
    fields = body.split('*', 1)[0].split(',')
    # An unsolved heading contains zero angles, which are not a valid attitude.
    if len(fields) < 8 or fields[0] != 'SOL_COMPUTED' or fields[1] == 'NONE':
        return None
    if len(header_fields) < 6:
        return None
    if header_fields[2] != 'GPS':
        return parse_uniheading(sentence, leap_seconds=leap_seconds)

    def number(index, converter=float):
        if index >= len(fields) or not fields[index]:
            return None
        try:
            value = converter(fields[index])
            return value if math.isfinite(value) else None
        except ValueError:
            return None

    try:
        week = int(header_fields[4])
        seconds = float(header_fields[5]) / 1000.0
        if week <= 0 or not 0 <= seconds < 604800:
            return None
        timestamp = gps_week_seconds_to_unix(week, seconds, leap_seconds)
    except (ValueError, OverflowError):
        return None
    return UniheadingData(
        length_m=number(2), heading_deg=number(3), pitch_deg=number(4),
        heading_stddev=number(6), pitch_stddev=number(7),
        stn_id=fields[8].strip('"') if len(fields) > 8 and fields[8] else None,
        num_svs=number(9, int), num_soln_svs=number(10, int),
        num_obs=number(11, int), num_multi=number(12, int), timestamp=timestamp,
    )


def _build_crc24q_table() -> list:
    """CRC-24Q (poly 0x864CFB) lookup table used by RTCM3."""
    table = []
    for i in range(256):
        crc = i << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        table.append(crc & 0xFFFFFF)
    return table


_CRC24Q_TABLE = _build_crc24q_table()


def crc24q(data: bytes) -> int:
    """Compute RTCM3 CRC-24Q."""
    crc = 0
    for b in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ _CRC24Q_TABLE[(crc >> 16) ^ b]
    return crc


class Rtcm3StreamParser:
    """
    Stateful RTCM3 frame parser.

    Frames may be split across TCP chunks, so received bytes are buffered.
    Each frame is validated by CRC-24Q, so corrupted data / false preambles
    are never counted as valid messages.
    """

    MAX_FRAME_LEN = 3 + 1023 + 3

    def __init__(self):
        self._buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes) -> list:
        """Feed bytes, return list of message type numbers of valid frames."""
        self._buf.extend(data)
        types = []
        buf = self._buf
        while True:
            idx = buf.find(b'\xd3')
            if idx < 0:
                buf.clear()
                break
            if idx > 0:
                del buf[:idx]
            if len(buf) < 3:
                break
            length = ((buf[1] & 0x03) << 8) | buf[2]
            total = 3 + length + 3
            if len(buf) < total:
                break
            frame = bytes(buf[:total])
            if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], 'big'):
                if length >= 2:
                    types.append((frame[3] << 4) | (frame[4] >> 4))
                del buf[:total]
            else:
                # False preamble or corrupted frame: resync from next byte
                self.crc_errors += 1
                del buf[:1]
        return types


@dataclass
class NtripStats:
    """Detailed NTRIP connection and RTCM transfer statistics."""
    connected: bool = False
    host: str = ""
    port: int = 2101
    mountpoint: str = ""
    last_connect_time: Optional[float] = None
    last_disconnect_time: Optional[float] = None
    caster_response: Optional[str] = None
    last_error: Optional[str] = None
    total_rx_bytes: int = 0
    total_written_bytes: int = 0
    write_errors: int = 0
    rtcm_crc_errors: int = 0
    last_rx_time: Optional[float] = None
    last_gga_sent_time: Optional[float] = None
    last_gga_sent_raw: Optional[str] = None
    last_gga_quality: Optional[int] = None
    rx_rate_bps: float = 0.0
    # Replaced (copy-on-write) on every update so that readers in other
    # threads can safely iterate over a consistent snapshot.
    recent_rtcm_types: dict = field(default_factory=dict)


class ExtendedNtripClient(threading.Thread):
    """
    Enhanced NTRIP client with rich logging, RTCM frame analysis,
    and verified serial write monitoring.
    """

    def __init__(
        self,
        host: str,
        port: int,
        mountpoint: str,
        user: str,
        password: str,
        on_rtcm_data: Callable[[bytes], Optional[int]],
        get_gga: Optional[Callable[[], Optional[str]]] = None,
        gga_interval: float = 5.0,
        on_log: Optional[Callable[[str, str], None]] = None,
        stats: Optional[NtripStats] = None,
    ):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.mountpoint = mountpoint
        self.user = user
        self.password = password
        # Must return number of bytes actually written, or raise on failure
        self.on_rtcm_data = on_rtcm_data
        self.get_gga = get_gga
        self.gga_interval = gga_interval
        self.on_log = on_log
        self.stats = stats or NtripStats(host=host, port=port, mountpoint=mountpoint)

        # NOTE: Do not name this `_stop`: threading.Thread has an internal
        # method with that name that must not be overwritten.
        self._stop_event = threading.Event()
        self._rate_calc_time = time.time()
        self._rate_calc_bytes = 0
        self._last_write_err_log = 0.0
        self._rtcm_parser = Rtcm3StreamParser()

    def _log(self, level: str, msg: str):
        if self.on_log:
            try:
                self.on_log(level, msg)
            except Exception:
                pass
        else:
            print(f"[NTRIP {level.upper()}] {msg}", file=sys.stderr)

    @property
    def is_connected(self) -> bool:
        return self.stats.connected

    def stop(self):
        self._stop_event.set()

    def run(self):
        backoff = 1
        while not self._stop_event.is_set():
            try:
                self._run_once()
                backoff = 1
            except Exception as exc:
                self.stats.connected = False
                self.stats.last_disconnect_time = time.time()
                self.stats.last_error = str(exc)
                self._log("warn", f"Disconnected/Error: {exc}. Reconnecting in {backoff}s...")
                # Interruptible sleep
                self._stop_event.wait(backoff)
                backoff = min(backoff * 2, 30)

    def _read_caster_response(self, sock: socket.socket):
        """
        Read the caster's response header.

        Returns:
            (status_line, leftover_bytes) where leftover_bytes are RTCM bytes
            that arrived in the same packets as the header.
        """
        response = sock.recv(1024)
        if not response:
            raise RuntimeError("Connection closed by caster before response")

        if response.startswith(b"ICY"):
            # NTRIP v1: "ICY 200 OK\r\n" followed directly by data
            line, _, rest = response.partition(b"\r\n")
            if rest.startswith(b"\r\n"):
                rest = rest[2:]
            return line.decode(errors="ignore").strip(), rest

        # HTTP-style response: read until end of header
        while b"\r\n\r\n" not in response and len(response) < 8192:
            more = sock.recv(1024)
            if not more:
                break
            response += more
        header, _, rest = response.partition(b"\r\n\r\n")
        line = header.split(b"\r\n", 1)[0].decode(errors="ignore").strip()
        return line, rest

    def _run_once(self):
        auth = base64.b64encode(f"{self.user}:{self.password}".encode()).decode()
        request = (
            f"GET /{self.mountpoint} HTTP/1.0\r\n"
            f"User-Agent: NTRIP cub_um982_client/1.0\r\n"
            f"Authorization: Basic {auth}\r\n"
            f"\r\n"
        ).encode()

        self._log("info", f"Connecting to caster {self.host}:{self.port}/{self.mountpoint}...")
        sock = socket.create_connection((self.host, self.port), timeout=10)
        try:
            sock.sendall(request)
            status_line, leftover = self._read_caster_response(sock)
            self.stats.caster_response = status_line[:120]

            if status_line.upper().startswith("SOURCETABLE"):
                raise RuntimeError(
                    f"Mountpoint '{self.mountpoint}' not found "
                    f"(caster returned sourcetable: {status_line})"
                )
            is_icy_ok = status_line.startswith("ICY 200")
            is_http_ok = status_line.startswith("HTTP/") and " 200" in status_line
            if not (is_icy_ok or is_http_ok):
                raise RuntimeError(f"NTRIP caster rejected request: {status_line}")

            self.stats.connected = True
            self.stats.last_connect_time = time.time()
            self.stats.last_error = None
            self._rtcm_parser = Rtcm3StreamParser()
            self._log(
                "info",
                f"Connected successfully to {self.host}:{self.port}/{self.mountpoint} ({status_line})"
            )

            sock.settimeout(2.0)
            last_gga = 0.0
            warned_no_gga = False

            if leftover:
                self._handle_rtcm(leftover)

            while not self._stop_event.is_set():
                now = time.time()

                # Periodically send GGA to caster
                if self.get_gga and self.gga_interval > 0 and now - last_gga >= self.gga_interval:
                    gga = self.get_gga()
                    if gga:
                        try:
                            sock.sendall((gga + "\r\n").encode("ascii", errors="ignore"))
                            last_gga = now
                            self.stats.last_gga_sent_time = now
                            self.stats.last_gga_sent_raw = gga
                            parts = gga.split(",")
                            quality = int(parts[6]) if len(parts) > 6 and parts[6].isdigit() else None
                            self.stats.last_gga_quality = quality
                            warned_no_gga = False
                            self._log("debug", f"Sent GGA position report to caster (Quality={quality})")
                        except Exception as e:
                            self._log("warn", f"Failed to send GGA to caster: {e}")
                    elif not warned_no_gga:
                        self._log(
                            "warn",
                            "Receiver position (GGA) not available yet. Caster may not provide "
                            "corrections until position is sent."
                        )
                        warned_no_gga = True

                # Receive RTCM data
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    continue

                if not data:
                    raise RuntimeError("Connection closed by NTRIP caster")

                self._handle_rtcm(data)

        finally:
            self.stats.connected = False
            self.stats.last_disconnect_time = time.time()
            sock.close()

    def _handle_rtcm(self, data: bytes):
        """Account for received RTCM bytes, analyse frames and forward to serial."""
        data_len = len(data)
        now = time.time()
        self.stats.total_rx_bytes += data_len
        self.stats.last_rx_time = now
        self._rate_calc_bytes += data_len

        # Calculate byte rate every 2 seconds
        dt = now - self._rate_calc_time
        if dt >= 2.0:
            self.stats.rx_rate_bps = self._rate_calc_bytes / dt
            self._rate_calc_bytes = 0
            self._rate_calc_time = now

        # Inspect RTCM3 frames (copy-on-write so readers never see a changing dict)
        types = self._rtcm_parser.feed(data)
        if types:
            new_types = dict(self.stats.recent_rtcm_types)
            for t in types:
                new_types[t] = new_types.get(t, 0) + 1
            self.stats.recent_rtcm_types = new_types
        self.stats.rtcm_crc_errors = self._rtcm_parser.crc_errors

        # Forward to UM982 via serial
        try:
            written = self.on_rtcm_data(data)
            self.stats.total_written_bytes += data_len if written is None else int(written)
        except Exception as write_err:
            self.stats.write_errors += 1
            if now - self._last_write_err_log >= WRITE_ERROR_LOG_INTERVAL:
                self._last_write_err_log = now
                self._log(
                    "error",
                    f"Failed to write RTCM to serial port: {write_err} "
                    f"(total write errors: {self.stats.write_errors})"
                )


class ExtendedUM982Client(UM982Client):
    """
    Subclass of UM982Client that:
    1. Separates Primary antenna (GPGGA/GNGGA) and Secondary antenna (GPGGAH/GNGGAH).
    2. Prevents secondary GGAH from corrupting primary position or NTRIP reporting.
    3. Provides detailed NTRIP status and RTCM write tracking.
    """

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        timeout: float = 0.5,
        output_rate: int = 10,
    ):
        super().__init__(port=port, baud=baud, timeout=timeout, output_rate=output_rate)

        # Secondary (slave) antenna state
        self._gga_secondary: Optional[GGAData] = None
        self._on_secondary_position: Optional[Callable[[PositionData], None]] = None

        # Empty GGA sentences (e.g. "$GNGGA,,,,,,0,,,,,,,,*78") mean that the
        # receiver currently has no position solution at all.
        self.no_fix_count = 0
        self.last_no_fix_time: Optional[float] = None
        self.secondary_no_fix_count = 0
        self.last_secondary_no_fix_time: Optional[float] = None

        # NTRIP logging & stats
        self._ntrip_log_callback: Optional[Callable[[str, str], None]] = None
        self._ntrip_stats = NtripStats()

    def set_output_rate(self, rate: int, enable_rmc: bool = False):
        """Request both antenna outputs using UM982 abbreviated ASCII commands."""
        self.output_rate = rate
        interval = 1.0 / rate if rate > 0 else 1.0
        commands = [
            f'GPGGA {interval:g}',
            f'GPGGAH {interval:g}',
            f'HEADINGA {interval:g}',
            'GPSUTCA ONCHANGED',
            'RECTIMEA 60',
        ]
        if enable_rmc:
            commands.append(f'GPRMC {interval:g}')
        for command in commands:
            self._write_line(command)
            time.sleep(0.1)

    def _record_no_fix(self, secondary: bool):
        if secondary:
            self.secondary_no_fix_count += 1
            self.last_secondary_no_fix_time = time.time()
        else:
            self.no_fix_count += 1
            self.last_no_fix_time = time.time()

    def set_secondary_position_callback(self, callback: Optional[Callable[[PositionData], None]]):
        """Set callback for secondary/slave antenna position updates."""
        self._on_secondary_position = callback

    def set_ntrip_log_callback(self, callback: Optional[Callable[[str, str], None]]):
        """Set logging callback for NTRIP events."""
        self._ntrip_log_callback = callback

    def get_ntrip_stats(self) -> NtripStats:
        """Get latest NTRIP connection and transmission statistics."""
        return self._ntrip_stats

    def get_secondary_position(self) -> Optional[PositionData]:
        """Get latest secondary antenna position data."""
        with self._data_lock:
            gga = self._gga_secondary
            uni = self._uniheading

        if gga is None:
            return None

        rtk_state = determine_rtk_state(gga, None)

        return PositionData(
            lat=gga.lat,
            lon=gga.lon,
            alt=gga.alt,
            heading=uni.heading_deg if uni else None,
            pitch=uni.pitch_deg if uni else None,
            speed_knots=None,
            course=None,
            rtk_state=rtk_state,
            num_sats=gga.num_sats,
            hdop=gga.hdop,
            baseline_m=uni.length_m if uni else None,
            timestamp=gga.timestamp,
            diff_age=gga.diff_age,
            heading_stddev=uni.heading_stddev if uni else None,
            pitch_stddev=uni.pitch_stddev if uni else None,
        )

    def _write_rtcm(self, data: bytes) -> int:
        """
        Write RTCM bytes to the receiver and return the number of bytes written.
        Unlike the base _write_bytes, this raises if the port is not open so that
        failed writes are never counted as successful.
        """
        if not self._ser:
            raise RuntimeError("serial port is not open")
        with self._write_lock:
            written = self._ser.write(data)
        written = len(data) if written is None else written
        self._update_rtcm_count(written)
        return written

    def start_ntrip(
        self,
        host: str,
        port: int,
        mountpoint: str,
        user: str,
        password: str,
        gga_interval: float = 5.0,
    ):
        """Start enhanced NTRIP client."""
        self._ntrip_stats.host = host
        self._ntrip_stats.port = port
        self._ntrip_stats.mountpoint = mountpoint

        self._ntrip_client = ExtendedNtripClient(
            host=host,
            port=port,
            mountpoint=mountpoint,
            user=user,
            password=password,
            on_rtcm_data=self._write_rtcm,
            get_gga=self._get_latest_gga_raw,
            gga_interval=gga_interval,
            on_log=self._ntrip_log_callback,
            stats=self._ntrip_stats,
        )
        self._ntrip_client.start()

    def _reader_loop(self):
        """
        Overridden serial reader loop:
        Accurately differentiates GGA (Primary) and GGAH (Secondary).
        """
        while not self._stop.is_set():
            try:
                line = self._readline()
            except Exception as e:
                # Serial error (e.g. device unplugged): avoid busy loop
                logger.error("Serial read error: %s", e)
                self._stop.wait(0.5)
                continue
            if not line:
                continue

            # Leap seconds parsing
            if line.startswith("#GPSUTC"):
                leap = parse_gpsutc(line)
                if leap is not None:
                    self._apply_leap_seconds(leap, "GPSUTC")
                continue
            if line.startswith("#RECTIME"):
                leap = parse_rectime(line)
                if leap is not None:
                    self._apply_leap_seconds(leap, "RECTIME")
                continue

            # Receiver without position solution outputs GGA/RMC with an empty
            # UTC time field. The thirdparty parser logs an error for each such
            # sentence (flooding the console), so handle them here instead.
            if line.startswith("$"):
                fields = line.split(",", 2)
                header = fields[0].upper()
                if (header.endswith(('GGA', 'GGAH', 'RMC'))
                        and len(fields) > 1 and not fields[1]):
                    if header.endswith(('GGA', 'GGAH')):
                        self._record_no_fix(header.endswith('GGAH'))
                    continue

            # GGA & GGAH parsing
            gga = parse_gga(line)
            if gga:
                # Sentence header token: e.g. "$GPGGA", "$GNGGA", "$GPGGAH"
                first_token = line.split(",", 1)[0].strip()
                is_secondary = first_token.upper().endswith("GGAH")

                if gga.quality == 0 or gga.lat is None or gga.lon is None:
                    self._record_no_fix(is_secondary)

                if is_secondary:
                    with self._data_lock:
                        self._gga_secondary = gga
                    if self._on_secondary_position:
                        sec_pos = self.get_secondary_position()
                        if sec_pos:
                            self._on_secondary_position(sec_pos)
                else:
                    with self._data_lock:
                        self._gga = gga
                    if self._on_position:
                        pos = self.get_position()
                        if pos:
                            self._on_position(pos)
                continue

            rmc = parse_rmc(line)
            if rmc:
                with self._data_lock:
                    self._rmc = rmc
                continue

            if line.split(',', 1)[0] in ('#HEADINGA', '#UNIHEADINGA'):
                uni = parse_receiver_heading(line, leap_seconds=self._leap_seconds)
                with self._data_lock:
                    self._uniheading = uni
