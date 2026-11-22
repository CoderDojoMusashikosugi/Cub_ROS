#!/usr/bin/env python3
"""M5Dial USB NCM watchdog.

M5DialのUSBネットワーク(NCM)で、Dial→PC方向の受信が止まる現象への対策ノード。
正常なDialは、ROS接続中は/dial/buttonを10Hzで送り、Agent停止中も1秒ごとにpingを送り、
PCがIP未取得ならDHCPに応答するため、受信パケット数(rx_packets)は常に増え続ける。
受信が一定時間止まったら、USBのGET_STATUS要求への応答を見てリセット対象を決める。

  Dial | 同じハブのフルスピード機器 | ハブ | 判断                       | 対処
  -----+---------------------------+------+----------------------------+---------------
  応答 | -                         | -    | NCM/ファームウェアの停止    | Dialをリセット
  無応答| 応答 (または兄弟なし)       | 応答 | Dial単体のUSB停止          | Dialをリセット
  無応答| 全て無応答                 | 応答 | ハブのTT(変換回路)の詰まり  | ハブをリセット
  -    | -                         | 無応答| ハブ自体/上流の問題         | 何もしない(要物理対処)

Dialのリセットで直らない場合(受信が戻らない、または再認識されない)は、ハブのリセットへ進む。
ハブのリセットでは、同じハブに繋がる他の機器(UM982等)も一瞬切断される。

コンテナ内(network_mode: host, privileged, パスワード無しsudo)で動かす前提。
"""

import ctypes
import errno
import fcntl
import glob
import os
import socket
import struct
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

SIOCGIFADDR = 0x8915


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


def read_text(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def ipv4_address(ifname):
    """インターフェースのIPv4アドレスを返す。未設定なら空文字。"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            packed = fcntl.ioctl(s.fileno(), SIOCGIFADDR,
                                 struct.pack('256s', ifname.encode()[:15]))
        return socket.inet_ntoa(packed[20:24])
    except OSError:
        return ''


def usb_bus_dev(usb_device):
    busnum = read_text(os.path.join(usb_device, 'busnum'))
    devnum = read_text(os.path.join(usb_device, 'devnum'))
    if busnum is None or devnum is None:
        return None
    return f'{int(busnum):03d}/{int(devnum):03d}'


def usb_probe(usb_device, timeout_ms=1000):
    """標準要求GET_STATUS(読み取り専用)を送る。True=応答あり, False=応答なし, None=判定不能。"""
    target = usb_bus_dev(usb_device)
    if target is None:
        return False
    buf = ctypes.create_string_buffer(2)
    ctrl = UsbCtrlTransfer(0x80, 0x00, 0, 0, 2, timeout_ms, ctypes.cast(buf, ctypes.c_void_p))
    try:
        fd = os.open(f'/dev/bus/usb/{target}', os.O_RDWR)
    except OSError:
        return None
    try:
        fcntl.ioctl(fd, USBDEVFS_CONTROL, ctrl)
        return True
    except OSError as e:
        return False if e.errno in NO_RESPONSE_ERRNOS else None
    finally:
        os.close(fd)


def full_speed_siblings(hub_device, exclude):
    """ハブ直下のフルスピード/ロースピード機器(=ハブのTTを経由する機器)のsysfsパス一覧。"""
    hub_name = os.path.basename(hub_device)
    siblings = []
    for path in sorted(glob.glob(os.path.join(hub_device, f'{hub_name}.*'))):
        name = os.path.basename(path)
        if ':' in name or path == exclude:
            continue
        if read_text(os.path.join(path, 'speed')) in ('1.5', '12'):
            siblings.append(path)
    return siblings


def choose_action(dial_ok, hub_ok, sibling_results, last_action, hub_reset_enabled):
    """リセット対象を決める。戻り値は ('dial'|'hub'|'none', 理由)。

    dial_ok/hub_ok/sibling_results の各値は usb_probe() の結果 (True/False/None)。
    last_action は、前回のリセットで復旧しなかった場合のその対象 ('dial'|'hub'|None)。
    """
    if hub_ok is False:
        return 'none', 'parent hub is not responding; physical action required'
    if last_action in ('dial', 'hub'):
        if hub_reset_enabled:
            return 'hub', f'previous {last_action} reset did not recover the Dial'
        return 'dial', f'previous {last_action} reset did not recover the Dial (hub reset disabled)'
    if dial_ok is False and sibling_results and all(r is False for r in sibling_results):
        if hub_reset_enabled:
            return 'hub', 'Dial and all full-speed devices on the hub are not responding (hub TT stuck)'
        return 'dial', 'hub TT seems stuck, but hub reset is disabled'
    if dial_ok is True:
        return 'dial', 'Dial responds on USB; NCM/firmware seems stuck'
    return 'dial', 'Dial is not responding on USB'


class DialNcmWatchdog(Node):
    def __init__(self):
        super().__init__('dial_ncm_watchdog')
        self.product_name = self.declare_parameter('product_name', 'M5Dial micro-ROS NCM').value
        self.check_period_sec = self.declare_parameter('check_period_sec', 5.0).value
        self.stall_timeout_sec = self.declare_parameter('stall_timeout_sec', 30.0).value
        self.reset_cooldown_sec = self.declare_parameter('reset_cooldown_sec', 120.0).value
        self.max_reset_cooldown_sec = self.declare_parameter('max_reset_cooldown_sec', 1800.0).value
        self.hub_reset_enabled = self.declare_parameter('hub_reset_enabled', True).value
        self.dry_run = self.declare_parameter('dry_run', False).value

        self.ifname = None
        self.usb_device = None      # Dialのsysfsパス
        self.hub_device = None      # Dialが繋がっているハブのsysfsパス(Dialが消えても使う)
        self.last_rx = None
        self.last_rx_change = None
        self.missing_since = None

        self.reset_proc = None      # 実行中のusbreset
        self.reset_target = None
        self.last_reset_done = None  # 直近のリセット(またはリセット判断)が終わった時刻
        self.last_action = None     # 復旧していない直近のリセット対象 ('dial'|'hub')
        self.consecutive_resets = 0
        self.total_resets = 0

        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(self.check_period_sec, self.check)
        self.get_logger().info(
            f'Watching "{self.product_name}": stall_timeout={self.stall_timeout_sec}s, '
            f'cooldown={self.reset_cooldown_sec}s, hub_reset={self.hub_reset_enabled}, '
            f'dry_run={self.dry_run}')

    def find_dial(self):
        """Dialのネットワークインターフェース名と、Dial本体のsysfsパスを返す。"""
        for net in sorted(glob.glob('/sys/class/net/*')):
            device = os.path.join(net, 'device')
            if not os.path.exists(device):
                continue
            usb_device = os.path.dirname(os.path.realpath(device))
            if read_text(os.path.join(usb_device, 'product')) == self.product_name:
                return os.path.basename(net), usb_device
        return None, None

    def hub_resettable(self):
        # ルートハブ(usb1等)はリセットしない。全USB機器が切断されるため。
        return self.hub_device is not None and '-' in os.path.basename(self.hub_device)

    def current_cooldown(self):
        if self.last_action == 'dial' and self.hub_reset_enabled and self.hub_resettable():
            # Dialのリセットで直らなかった場合は、待たずにハブのリセットへ進む
            return self.stall_timeout_sec
        # 同じ対処を繰り返す場合は、リセット間隔を倍々に延ばす
        cooldown = self.reset_cooldown_sec * (2 ** max(self.consecutive_resets - 1, 0))
        return min(cooldown, self.max_reset_cooldown_sec)

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
        self.last_reset_done = time.monotonic()
        return False

    def check(self):
        now = time.monotonic()
        resetting = self.poll_reset_proc()
        ifname, usb_device = self.find_dial()

        if ifname is None:
            if self.ifname is not None:
                self.get_logger().warn(f'Dial interface {self.ifname} disappeared')
                self.missing_since = now
            self.ifname = None
            self.last_rx = None
            if resetting:
                self.publish_diag(DiagnosticStatus.WARN, f'USB reset {self.reset_target} in progress', {})
            elif self.last_action is not None and self.hub_device is not None:
                # 自分がリセットした後に再認識されない場合は、次の段階へ進む
                missing_sec = now - max(self.missing_since or now, self.last_reset_done or now)
                if missing_sec >= self.stall_timeout_sec and self.cooldown_elapsed(now):
                    self.escalate(None, None, f'Dial not re-enumerated for {missing_sec:.0f}s after reset')
                self.publish_diag(DiagnosticStatus.ERROR, 'Dial not re-enumerated after reset',
                                  {'last_action': self.last_action})
            else:
                self.publish_diag(DiagnosticStatus.WARN, 'Dial not connected', {})
            return

        self.missing_since = None
        self.usb_device = usb_device
        self.hub_device = os.path.dirname(usb_device)
        rx = int(read_text(f'/sys/class/net/{ifname}/statistics/rx_packets') or 0)
        if ifname != self.ifname or self.last_rx is None or rx < self.last_rx:
            # 初回検出時、リセット等でインターフェースが作り直された時は計測をやり直す
            if ifname != self.ifname:
                self.get_logger().info(f'Found Dial interface {ifname} ({usb_device})')
            self.ifname = ifname
            self.last_rx = rx
            self.last_rx_change = now
        elif rx > self.last_rx:
            if self.consecutive_resets > 0:
                self.get_logger().info(
                    f'Dial recovered: receiving again on {ifname} '
                    f'(after {self.consecutive_resets} reset(s), last: {self.last_action})')
                self.consecutive_resets = 0
                self.last_action = None
            self.last_rx = rx
            self.last_rx_change = now

        stalled_sec = now - self.last_rx_change
        ipv4 = ipv4_address(ifname)
        values = {
            'interface': ifname,
            'ipv4': ipv4 or '(none)',
            'rx_packets': str(rx),
            'seconds_since_rx': f'{stalled_sec:.0f}',
            'last_action': self.last_action or '-',
            'consecutive_resets': str(self.consecutive_resets),
            'total_resets': str(self.total_resets),
        }

        if resetting:
            self.publish_diag(DiagnosticStatus.WARN, f'USB reset {self.reset_target} in progress', values)
            return
        if stalled_sec < self.stall_timeout_sec:
            self.publish_diag(DiagnosticStatus.OK, 'Receiving', values)
            return
        if not self.cooldown_elapsed(now):
            self.publish_diag(DiagnosticStatus.ERROR,
                              f'No packets from Dial, waiting cooldown ({self.current_cooldown():.0f}s)',
                              values)
            return

        self.escalate(rx, ipv4, f'no packets from Dial on {ifname} for {stalled_sec:.0f}s')
        self.publish_diag(DiagnosticStatus.ERROR, 'No packets from Dial, USB reset issued', values)

    def cooldown_elapsed(self, now):
        return self.last_reset_done is None or now - self.last_reset_done >= self.current_cooldown()

    def escalate(self, rx, ipv4, reason):
        """GET_STATUSの応答から対象を決めてリセットする。"""
        dial_ok = usb_probe(self.usb_device) if self.usb_device and os.path.exists(self.usb_device) else False
        hub_ok = usb_probe(self.hub_device)
        siblings = full_speed_siblings(self.hub_device, self.usb_device)
        sibling_results = [usb_probe(s) for s in siblings]
        action, why = choose_action(dial_ok, hub_ok, sibling_results, self.last_action,
                                    self.hub_reset_enabled and self.hub_resettable())
        probe_summary = ', '.join(
            [f'dial={dial_ok}', f'hub={hub_ok}'] +
            [f'{os.path.basename(s)}={r}' for s, r in zip(siblings, sibling_results)])
        self.get_logger().error(
            f'{reason} (rx_packets={rx}, ipv4={ipv4 or "none"}). GET_STATUS: {probe_summary}. '
            f'Action: {action} ({why})')

        self.consecutive_resets += 1
        if action == 'none':
            self.last_reset_done = time.monotonic()
            return
        device = self.hub_device if action == 'hub' else self.usb_device
        self.last_action = action
        self.start_reset(device)

    def start_reset(self, device):
        target = usb_bus_dev(device)
        name = read_text(os.path.join(device, 'product')) or os.path.basename(device)
        if target is None:
            self.get_logger().error(f'Cannot reset {name}: device not found')
            self.last_reset_done = time.monotonic()
            return
        self.reset_target = f'{target} ({name})'
        if self.dry_run:
            self.get_logger().warn(f'dry_run: skipped usbreset {self.reset_target}')
            self.last_reset_done = time.monotonic()
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
            self.last_reset_done = time.monotonic()
        # インターフェースが作り直されるので、次回の検出から計測をやり直す
        self.last_rx = None

    def publish_diag(self, level, message, values):
        status = DiagnosticStatus()
        status.level = level
        status.name = 'dial_ncm_watchdog: M5Dial USB NCM'
        status.message = message
        status.hardware_id = self.product_name
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diag_pub.publish(array)


def main():
    rclpy.init()
    node = DialNcmWatchdog()
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
