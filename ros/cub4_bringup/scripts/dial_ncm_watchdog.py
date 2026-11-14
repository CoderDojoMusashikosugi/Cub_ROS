#!/usr/bin/env python3
"""M5Dial USB NCM watchdog.

M5DialのUSBネットワーク(NCM)で、Dial→PC方向の受信が止まる現象への対策ノード。
正常なDialは、ROS接続中は/dial/buttonを10Hzで送り、Agent停止中も1秒ごとにpingを送り、
PCがIP未取得ならDHCPに応答するため、受信パケット数(rx_packets)は常に増え続ける。
受信が一定時間止まったら、Dialの送信が止まったと判断してUSBバスリセットをかける。

コンテナ内(network_mode: host, privileged, パスワード無しsudo)で動かす前提。
"""

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


class DialNcmWatchdog(Node):
    def __init__(self):
        super().__init__('dial_ncm_watchdog')
        self.product_name = self.declare_parameter('product_name', 'M5Dial micro-ROS NCM').value
        self.check_period_sec = self.declare_parameter('check_period_sec', 5.0).value
        self.stall_timeout_sec = self.declare_parameter('stall_timeout_sec', 30.0).value
        self.reset_cooldown_sec = self.declare_parameter('reset_cooldown_sec', 120.0).value
        self.max_reset_cooldown_sec = self.declare_parameter('max_reset_cooldown_sec', 1800.0).value
        self.dry_run = self.declare_parameter('dry_run', False).value

        self.ifname = None
        self.last_rx = None
        self.last_rx_change = None
        self.last_reset = None
        self.consecutive_resets = 0
        self.total_resets = 0

        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(self.check_period_sec, self.check)
        self.get_logger().info(
            f'Watching "{self.product_name}": stall_timeout={self.stall_timeout_sec}s, '
            f'cooldown={self.reset_cooldown_sec}s, dry_run={self.dry_run}')

    def find_dial(self):
        """Dialのネットワークインターフェース名と、親USB機器のsysfsパスを返す。"""
        for net in sorted(glob.glob('/sys/class/net/*')):
            device = os.path.join(net, 'device')
            if not os.path.exists(device):
                continue
            usb_device = os.path.dirname(os.path.realpath(device))
            if read_text(os.path.join(usb_device, 'product')) == self.product_name:
                return os.path.basename(net), usb_device
        return None, None

    def current_cooldown(self):
        # 連続でリセットしても直らない場合は、リセット間隔を倍々に延ばす
        cooldown = self.reset_cooldown_sec * (2 ** max(self.consecutive_resets - 1, 0))
        return min(cooldown, self.max_reset_cooldown_sec)

    def check(self):
        now = time.monotonic()
        ifname, usb_device = self.find_dial()

        if ifname is None:
            if self.ifname is not None:
                self.get_logger().warn(f'Dial interface {self.ifname} disappeared')
            self.ifname = None
            self.last_rx = None
            self.publish_diag(DiagnosticStatus.WARN, 'Dial not connected', {})
            return

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
                    f'(after {self.consecutive_resets} reset(s))')
                self.consecutive_resets = 0
            self.last_rx = rx
            self.last_rx_change = now

        stalled_sec = now - self.last_rx_change
        ipv4 = ipv4_address(ifname)
        values = {
            'interface': ifname,
            'ipv4': ipv4 or '(none)',
            'rx_packets': str(rx),
            'seconds_since_rx': f'{stalled_sec:.0f}',
            'consecutive_resets': str(self.consecutive_resets),
            'total_resets': str(self.total_resets),
        }

        if stalled_sec < self.stall_timeout_sec:
            self.publish_diag(DiagnosticStatus.OK, 'Receiving', values)
            return

        cooldown = self.current_cooldown()
        if self.last_reset is not None and now - self.last_reset < cooldown:
            self.publish_diag(DiagnosticStatus.ERROR,
                              f'No packets from Dial, waiting cooldown ({cooldown:.0f}s)', values)
            return

        self.reset_dial(ifname, usb_device, rx, ipv4, stalled_sec)
        self.publish_diag(DiagnosticStatus.ERROR, 'No packets from Dial, USB reset issued', values)

    def reset_dial(self, ifname, usb_device, rx, ipv4, stalled_sec):
        busnum = int(read_text(os.path.join(usb_device, 'busnum')) or 0)
        devnum = int(read_text(os.path.join(usb_device, 'devnum')) or 0)
        target = f'{busnum:03d}/{devnum:03d}'
        self.get_logger().error(
            f'No packets from Dial on {ifname} for {stalled_sec:.0f}s '
            f'(rx_packets={rx}, ipv4={ipv4 or "none"}). USB reset {target}'
            f'{" (dry_run: skipped)" if self.dry_run else ""}')

        self.last_reset = time.monotonic()
        self.consecutive_resets += 1
        if self.dry_run:
            return

        self.total_resets += 1
        try:
            result = subprocess.run(['sudo', '-n', 'usbreset', target],
                                    capture_output=True, text=True, timeout=15)
            output = (result.stdout + result.stderr).strip()
            if result.returncode == 0:
                self.get_logger().info(f'usbreset {target} done: {output}')
            else:
                self.get_logger().error(
                    f'usbreset {target} failed (code {result.returncode}): {output}')
        except (OSError, subprocess.TimeoutExpired) as e:
            self.get_logger().error(f'usbreset {target} failed: {e}')
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
