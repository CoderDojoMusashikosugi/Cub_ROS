#!/usr/bin/env python3
"""Spresense IMU USB watchdog.

Spresense IMU(USBシリアルCP2102N, /dev/ttyMULIMU)からの受信が止まる現象への対策ノード。
dial_ncm_watchdog.py と同じ考え方で、/imu/data_raw が一定時間届かなくなったら、
USBのGET_STATUS要求への応答と、ハブのポート状態を見てリセット対象を決める。

正常時、spresense_imu_nodeは/imu/data_rawを240Hzで配信する。spresense_imu_node自身も
受信が5秒止まるとポートを開き直すが、CP2102NがUSBの要求に応答しなくなった場合や、
抜き差し後にUSBの認識(エニュメレーション)に失敗した場合は、開き直しでは直らない。

  IMU(CP2102N)       | 同じハブのフルスピード機器 | ハブ  | 判断                         | 対処
  -------------------+---------------------------+-------+------------------------------+--------------
  応答               | -                         | 応答  | シリアル/Spresense側の停止    | IMUをリセット
  無応答             | 応答 (または兄弟なし)       | 応答  | IMU単体のUSB停止             | IMUをリセット
  無応答             | 全て無応答                 | 応答  | ハブのTT(変換回路)の詰まり    | ハブをリセット
  未認識(ポートに接続あり) | -                    | 応答  | USBの認識に失敗              | ハブをリセット
  未認識(ポートに接続なし) | -                    | 応答  | 抜かれている                  | 何もしない
  -                  | -                         | 無応答 | ハブ自体/上流の問題           | 何もしない(要物理対処)

IMUのリセットで直らない場合(受信が戻らない、または再認識されない)は、ハブのリセットへ進む。
ハブのリセットでは、同じハブに繋がる他の機器も一瞬切断される。

first_reset='hub' にすると、上の表で「IMUをリセット」になる場合も、最初からハブをリセットする。
CP2102N単体のUSBリセットの後に、CP2102NがUSBの要求に応答しなくなる例があったため。
ただし、ハブにIMU以外の機器が繋がっている場合は、それらを巻き込まないようにIMUのリセットから始める。
/imu/data_rawの配信者(spresense_imu_node)がいない間は、ノードが止まっているだけなのでリセットしない。

コンテナ内(privileged, パスワード無しsudo)で動かす前提。
"""

import ctypes
import errno
import fcntl
import glob
import os
import struct
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu


class UsbCtrlTransfer(ctypes.Structure):
    """struct usbdevfs_ctrltransfer (linux/usbdevice_fs.h)"""
    _fields_ = [
        ('bRequestType', ctypes.c_uint8),
        ('bRequest', ctypes.c_uint8),
        ('wValue', ctypes.c_uint16),
        ('wIndex', ctypes.c_uint16),
        ('wLength', ctypes.c_uint16),
        ('timeout', ctypes.c_uint32),
        ('data', ctypes.c_void_p),
    ]


# _IOWR('U', 0, struct usbdevfs_ctrltransfer)
USBDEVFS_CONTROL = (3 << 30) | (ctypes.sizeof(UsbCtrlTransfer) << 16) | (ord('U') << 8) | 0
# USBの応答が無いことを示すエラー。権限不足などはここに含めず「判定不能」として扱う。
NO_RESPONSE_ERRNOS = {errno.ETIMEDOUT, errno.EPROTO, errno.EPIPE, errno.EIO, errno.ENODEV,
                      errno.ESHUTDOWN, errno.EILSEQ}
# ハブのポート状態(wPortStatus)のビット
PORT_CONNECTION = 0x0001
PORT_ENABLE = 0x0002


def read_text(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def usb_bus_dev(usb_device):
    busnum = read_text(os.path.join(usb_device, 'busnum'))
    devnum = read_text(os.path.join(usb_device, 'devnum'))
    if busnum is None or devnum is None:
        return None
    return f'{int(busnum):03d}/{int(devnum):03d}'


def usb_control_read(usb_device, request_type, request, value, index, length, timeout_ms=1000):
    """読み取りのみのコントロール転送を送る。応答データ(bytes), False=応答なし, None=判定不能。"""
    target = usb_bus_dev(usb_device)
    if target is None:
        return False
    buf = ctypes.create_string_buffer(length)
    ctrl = UsbCtrlTransfer(request_type, request, value, index, length, timeout_ms,
                           ctypes.cast(buf, ctypes.c_void_p))
    try:
        fd = os.open(f'/dev/bus/usb/{target}', os.O_RDWR)
    except OSError:
        return None
    try:
        fcntl.ioctl(fd, USBDEVFS_CONTROL, ctrl)
        return buf.raw
    except OSError as e:
        return False if e.errno in NO_RESPONSE_ERRNOS else None
    finally:
        os.close(fd)


def usb_probe(usb_device, timeout_ms=1000):
    """標準要求GET_STATUS(読み取り専用)を送る。True=応答あり, False=応答なし, None=判定不能。"""
    result = usb_control_read(usb_device, 0x80, 0x00, 0, 0, 2, timeout_ms)
    if isinstance(result, bytes):
        return True
    return result


def hub_port_connected(hub_device, port, timeout_ms=1000):
    """ハブのポートに機器が電気的に繋がっているか(ハブクラス要求GET_STATUS, 読み取り専用)。

    True=接続あり, False=接続なし, None=判定不能。lsusb -v の Hub Port Status と同じ情報。
    """
    if hub_device is None or port is None:
        return None
    result = usb_control_read(hub_device, 0xA3, 0x00, 0, port, 4, timeout_ms)
    if not isinstance(result, bytes):
        return None
    status, _change = struct.unpack('<HH', result)
    return bool(status & PORT_CONNECTION)


def usb_device_of_tty(tty, sys_root='/sys'):
    """ttyの名前(ttyUSB2等)から、そのUSB機器のsysfsパスを返す。見つからなければNone。"""
    device = os.path.join(sys_root, 'class', 'tty', tty, 'device')
    if not os.path.exists(device):
        return None
    path = os.path.realpath(device)
    # usb-serialは .../1-2.3.1/1-2.3.1:1.0/ttyUSB2、cdc-acmは .../1-2.3.1/1-2.3.1:1.0 を指すので、
    # busnum/devnumを持つ階層(USB機器)まで遡る
    while path != os.path.dirname(path):
        if os.path.isfile(os.path.join(path, 'busnum')) and os.path.isfile(os.path.join(path, 'devnum')):
            return path
        path = os.path.dirname(path)
    return None


def port_number(usb_device):
    """USB機器のsysfs名(1-2.3.1等)から、親ハブのポート番号を返す。"""
    name = os.path.basename(usb_device)
    try:
        if '.' in name:
            return int(name.rsplit('.', 1)[1])
        return int(name.split('-', 1)[1])
    except (IndexError, ValueError):
        return None


def hub_children(hub_device, exclude):
    """ハブ直下に繋がっているUSB機器(速度を問わない)のsysfsパス一覧。excludeは除く。"""
    hub_name = os.path.basename(hub_device)
    children = []
    for path in sorted(glob.glob(os.path.join(hub_device, f'{hub_name}.*'))):
        name = os.path.basename(path)
        if ':' in name or path == exclude:
            continue
        children.append(path)
    return children


def full_speed_siblings(hub_device, exclude):
    """ハブ直下のフルスピード/ロースピード機器(=ハブのTTを経由する機器)のsysfsパス一覧。"""
    return [path for path in hub_children(hub_device, exclude)
            if read_text(os.path.join(path, 'speed')) in ('1.5', '12')]


def choose_action(imu_present, imu_ok, hub_ok, sibling_results, port_connected, last_action,
                  hub_reset_enabled, prefer_hub=False):
    """リセット対象を決める。戻り値は ('imu'|'hub'|'none', 理由)。

    imu_present は、IMUがUSB機器として認識されているか。
    imu_ok/hub_ok/sibling_results の各値は usb_probe() の結果 (True/False/None)。
    port_connected は hub_port_connected() の結果 (True/False/None)。IMUが認識されている間は使わない。
    last_action は、前回のリセットで復旧しなかった場合のその対象 ('imu'|'hub'|None)。
    prefer_hub が True なら、IMUのリセットの代わりに最初からハブをリセットする。
    """
    if hub_ok is False:
        return 'none', 'parent hub is not responding; physical action required'
    if not imu_present:
        if port_connected is False:
            return 'none', 'nothing is connected to the hub port (IMU unplugged?)'
        if not hub_reset_enabled:
            return 'none', 'IMU is not enumerated, but hub reset is disabled'
        if port_connected is True:
            return 'hub', 'IMU is connected to the hub port but not enumerated'
        if last_action in ('imu', 'hub'):
            return 'hub', f'IMU not re-enumerated after previous {last_action} reset'
        return 'none', 'IMU is not enumerated and the hub port state is unknown'
    if last_action in ('imu', 'hub'):
        if hub_reset_enabled:
            return 'hub', f'previous {last_action} reset did not recover the IMU'
        return 'imu', f'previous {last_action} reset did not recover the IMU (hub reset disabled)'
    if prefer_hub and hub_reset_enabled:
        return 'hub', 'first_reset=hub and the IMU is the only device on the hub'
    if imu_ok is False and sibling_results and all(r is False for r in sibling_results):
        if hub_reset_enabled:
            return 'hub', 'IMU and all full-speed devices on the hub are not responding (hub TT stuck)'
        return 'imu', 'hub TT seems stuck, but hub reset is disabled'
    if imu_ok is True:
        return 'imu', 'IMU responds on USB; serial link or Spresense seems stuck'
    return 'imu', 'IMU is not responding on USB'


class ImuUsbWatchdog(Node):
    def __init__(self, **kwargs):
        super().__init__('imu_usb_watchdog', **kwargs)
        self.topic = self.declare_parameter('topic', '/imu/data_raw').value
        self.serial_port = self.declare_parameter('serial_port', '/dev/ttyMULIMU').value
        self.check_period_sec = self.declare_parameter('check_period_sec', 5.0).value
        self.stall_timeout_sec = self.declare_parameter('stall_timeout_sec', 30.0).value
        self.reset_cooldown_sec = self.declare_parameter('reset_cooldown_sec', 120.0).value
        self.max_reset_cooldown_sec = self.declare_parameter('max_reset_cooldown_sec', 1800.0).value
        self.hub_reset_enabled = self.declare_parameter('hub_reset_enabled', True).value
        # 'imu': IMUのリセットから始める, 'hub': 最初からハブをリセットする(ハブにIMUしか無い時だけ)
        self.first_reset = self.declare_parameter('first_reset', 'imu').value
        if self.first_reset not in ('imu', 'hub'):
            self.get_logger().warn(f'Unknown first_reset "{self.first_reset}", using "imu"')
            self.first_reset = 'imu'
        self.dry_run = self.declare_parameter('dry_run', False).value
        self.now = time.monotonic  # テストで差し替えられるように

        self.usb_device = None      # IMU(CP2102N)のsysfsパス
        self.usb_present = False
        self.hub_device = None      # IMUが繋がっているハブのsysfsパス(IMUが消えても使う)
        self.port = None            # ハブのポート番号(IMUが消えても使う)
        self.missing_since = None

        now = self.now()
        self.baseline = now         # 受信停止の計測を始めた時刻(起動時、検出時、リセット後に更新)
        self.last_data = None       # 最後に/imu/data_rawを受信した時刻
        self.count = 0              # 前回のcheckからの受信数
        self.last_check = now

        self.reset_proc = None      # 実行中のusbreset
        self.reset_target = None
        self.last_reset_done = None  # 直近のリセット(またはリセット判断)が終わった時刻
        self.last_action = None     # 復旧していない直近のリセット対象 ('imu'|'hub')
        self.consecutive_resets = 0
        self.total_resets = 0

        # 240Hzで届くので、デシリアライズせずに受信時刻だけを記録する
        self.sub = self.create_subscription(Imu, self.topic, self.on_data, qos_profile_sensor_data,
                                            raw=True)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(self.check_period_sec, self.check)
        self.get_logger().info(
            f'Watching {self.topic} from {self.serial_port}: stall_timeout={self.stall_timeout_sec}s, '
            f'cooldown={self.reset_cooldown_sec}s, hub_reset={self.hub_reset_enabled}, '
            f'first_reset={self.first_reset}, dry_run={self.dry_run}')

    def on_data(self, _msg):
        self.last_data = self.now()
        self.count += 1

    def find_imu(self):
        """serial_port(udevのシンボリックリンク)が指すIMUのUSB機器のsysfsパスを返す。"""
        if not os.path.exists(self.serial_port):
            return None
        return usb_device_of_tty(os.path.basename(os.path.realpath(self.serial_port)))

    def hub_resettable(self):
        # ルートハブ(usb1等)はリセットしない。全USB機器が切断されるため。
        return self.hub_device is not None and '-' in os.path.basename(self.hub_device)

    def current_cooldown(self):
        if self.last_action == 'imu' and self.hub_reset_enabled and self.hub_resettable():
            # IMUのリセットで直らなかった場合は、待たずにハブのリセットへ進む
            return self.stall_timeout_sec
        # 同じ対処を繰り返す場合は、リセット間隔を倍々に延ばす
        cooldown = self.reset_cooldown_sec * (2 ** max(self.consecutive_resets - 1, 0))
        return min(cooldown, self.max_reset_cooldown_sec)

    def cooldown_elapsed(self, now):
        return self.last_reset_done is None or now - self.last_reset_done >= self.current_cooldown()

    def poll_reset_proc(self):
        """実行中のusbresetがあれば終了を確認する。実行中ならTrue。"""
        if self.reset_proc is None:
            return False
        if self.reset_proc.poll() is None:
            return True
        output = ' '.join(self.reset_proc.communicate()[0].split())
        code = self.reset_proc.returncode
        if code == 0:
            self.get_logger().info(f'usbreset {self.reset_target} done: {output}')
        else:
            self.get_logger().error(f'usbreset {self.reset_target} failed (code {code}): {output}')
        self.reset_proc = None
        self.last_reset_done = self.now()
        return False

    def check(self):
        now = self.now()
        resetting = self.poll_reset_proc()
        usb_device = self.find_imu()
        publishers = self.count_publishers(self.topic)
        period = max(now - self.last_check, 1e-3)
        rate = self.count / period
        self.count = 0
        self.last_check = now

        if usb_device is None:
            self.check_missing(now, resetting, publishers)
            return

        if not self.usb_present or usb_device != self.usb_device:
            # 初回検出時、リセットや抜き差しで認識し直された時は計測をやり直す
            self.get_logger().info(f'Found IMU {self.serial_port} at {usb_device}')
            self.baseline = now
        self.usb_present = True
        self.missing_since = None
        self.usb_device = usb_device
        self.hub_device = os.path.dirname(usb_device)
        self.port = port_number(usb_device)

        if self.last_data is not None and self.last_data > self.baseline:
            if self.consecutive_resets > 0:
                self.get_logger().info(
                    f'IMU recovered: receiving {self.topic} again '
                    f'(after {self.consecutive_resets} reset(s), last: {self.last_action})')
                self.consecutive_resets = 0
                self.last_action = None

        stalled_sec = now - max(self.last_data or self.baseline, self.baseline)
        values = self.diag_values(publishers, rate, stalled_sec)

        if resetting:
            self.publish_diag(DiagnosticStatus.WARN, f'USB reset {self.reset_target} in progress', values)
            return
        if stalled_sec < self.stall_timeout_sec:
            self.publish_diag(DiagnosticStatus.OK, 'Receiving', values)
            return
        if publishers == 0:
            # spresense_imu_nodeが止まっているだけなのでリセットしない
            self.publish_diag(DiagnosticStatus.WARN, f'No publisher on {self.topic}', values)
            return
        if not self.cooldown_elapsed(now):
            self.publish_diag(DiagnosticStatus.ERROR,
                              f'No IMU data, waiting cooldown ({self.current_cooldown():.0f}s)', values)
            return

        self.escalate(f'no {self.topic} for {stalled_sec:.0f}s')
        self.publish_diag(DiagnosticStatus.ERROR, 'No IMU data, USB reset issued', values)

    def check_missing(self, now, resetting, publishers):
        """IMUがUSB機器として見えない時の処理。"""
        if self.usb_present:
            self.get_logger().warn(f'IMU {self.serial_port} disappeared ({self.usb_device})')
            self.missing_since = now
        self.usb_present = False
        values = {
            'serial_port': self.serial_port,
            'usb_device': '(missing)',
            'hub': os.path.basename(self.hub_device) if self.hub_device else '-',
            'publishers': str(publishers),
            'last_action': self.last_action or '-',
            'consecutive_resets': str(self.consecutive_resets),
            'total_resets': str(self.total_resets),
        }
        if resetting:
            self.publish_diag(DiagnosticStatus.WARN, f'USB reset {self.reset_target} in progress', values)
            return
        if self.hub_device is None:
            # 起動してから一度も見つかっていないので、どのハブかわからない
            self.publish_diag(DiagnosticStatus.WARN, f'IMU {self.serial_port} not found', values)
            return
        if self.missing_since is None:
            self.missing_since = now
        missing_sec = now - max(self.missing_since, self.last_reset_done or self.missing_since)
        values['seconds_missing'] = f'{missing_sec:.0f}'
        if missing_sec >= self.stall_timeout_sec and publishers > 0 and self.cooldown_elapsed(now):
            self.escalate(f'IMU {self.serial_port} missing for {missing_sec:.0f}s')
        self.publish_diag(DiagnosticStatus.ERROR, 'IMU USB device not enumerated', values)

    def escalate(self, reason):
        """GET_STATUSの応答とハブのポート状態から対象を決めてリセットする。"""
        present = self.usb_present and self.usb_device is not None and os.path.exists(self.usb_device)
        imu_ok = usb_probe(self.usb_device) if present else False
        hub_ok = usb_probe(self.hub_device)
        siblings = full_speed_siblings(self.hub_device, self.usb_device)
        sibling_results = [usb_probe(s) for s in siblings]
        port_connected = None if present else hub_port_connected(self.hub_device, self.port)
        # ハブに他の機器が繋がっていれば、巻き込まないようにハブ優先をやめる
        others = hub_children(self.hub_device, self.usb_device)
        prefer_hub = self.first_reset == 'hub' and not others
        action, why = choose_action(present, imu_ok, hub_ok, sibling_results, port_connected,
                                    self.last_action, self.hub_reset_enabled and self.hub_resettable(),
                                    prefer_hub)
        if self.first_reset == 'hub' and others:
            why += f' (first_reset=hub skipped: {len(others)} other device(s) on the hub)'
        probe_summary = ', '.join(
            [f'imu={imu_ok if present else "missing"}', f'hub={hub_ok}'] +
            ([] if present else [f'port{self.port}_connected={port_connected}']) +
            [f'{os.path.basename(s)}={r}' for s, r in zip(siblings, sibling_results)])
        self.get_logger().error(f'{reason}. GET_STATUS: {probe_summary}. Action: {action} ({why})')

        self.consecutive_resets += 1
        if action == 'none':
            self.last_reset_done = self.now()
            return
        device = self.hub_device if action == 'hub' else self.usb_device
        self.last_action = action
        self.start_reset(device)

    def start_reset(self, device):
        target = usb_bus_dev(device)
        name = read_text(os.path.join(device, 'product')) or os.path.basename(device)
        if target is None:
            self.get_logger().error(f'Cannot reset {name}: device not found')
            self.last_reset_done = self.now()
            return
        self.reset_target = f'{target} ({name})'
        # 再認識後の受信停止は、そこから計測し直す
        self.baseline = self.now()
        if self.dry_run:
            self.get_logger().warn(f'dry_run: skipped usbreset {self.reset_target}')
            self.last_reset_done = self.now()
            return
        self.total_resets += 1
        # 応答しない機器のリセットは数十秒かかることがあるため、完了を待たずに次の周期で確認する
        self.get_logger().warn(f'Running usbreset {self.reset_target}')
        try:
            self.reset_proc = subprocess.Popen(['sudo', '-n', 'usbreset', target],
                                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                               text=True)
        except OSError as e:
            self.get_logger().error(f'usbreset {self.reset_target} failed to start: {e}')
            self.last_reset_done = self.now()

    def diag_values(self, publishers, rate, stalled_sec):
        return {
            'serial_port': self.serial_port,
            'usb_device': os.path.basename(self.usb_device) if self.usb_device else '-',
            'hub': os.path.basename(self.hub_device) if self.hub_device else '-',
            'publishers': str(publishers),
            'rate_hz': f'{rate:.1f}',
            'seconds_since_data': f'{stalled_sec:.0f}',
            'last_action': self.last_action or '-',
            'consecutive_resets': str(self.consecutive_resets),
            'total_resets': str(self.total_resets),
        }

    def publish_diag(self, level, message, values):
        status = DiagnosticStatus()
        status.level = level
        status.name = 'imu_usb_watchdog: Spresense IMU USB'
        status.message = message
        status.hardware_id = self.serial_port
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diag_pub.publish(array)


def main():
    rclpy.init()
    node = ImuUsbWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
