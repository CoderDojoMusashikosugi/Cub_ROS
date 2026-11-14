"""Serial link failure detection and USB reset targeting."""

import os
from pathlib import Path
import sys
import threading
from unittest.mock import Mock, patch

import pytest
import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cub_um982.extended_client import ExtendedUM982Client
from cub_um982.usb_reset import (
    USBDEVFS_RESET,
    UsbDeviceInfo,
    find_usb_device_for_tty,
    reset_usb_device,
)


def make_running_client():
    client = ExtendedUM982Client('/unused')
    client._ser = Mock()
    client._reader_thread = Mock(is_alive=Mock(return_value=True))
    client.open_time = 100.0
    return client


def test_read_error_stops_reader_and_is_reported():
    client = ExtendedUM982Client('/unused')
    error = serial.SerialException('device reports readiness to read but returned no data')
    with patch.object(client, '_readline', side_effect=error):
        client._reader_loop()  # must return instead of retrying forever
    assert 'serial read failed' in client.serial_error
    assert client.get_health_problem(5.0) == client.serial_error


def test_write_error_is_recorded_and_raised():
    client = make_running_client()
    client._ser.write.side_effect = serial.SerialException('write failed: [Errno 5] Input/output error')
    with pytest.raises(serial.SerialException):
        client._write_rtcm(b'\xd3\x00\x00')
    assert 'Errno 5' in client.get_health_problem(5.0)


def test_write_timeout_is_recorded():
    client = make_running_client()
    client._ser.write.side_effect = serial.SerialTimeoutException('Write timeout')
    with pytest.raises(serial.SerialTimeoutException):
        client._write_rtcm(b'\xd3')
    assert 'Write timeout' in client.serial_error


def test_errors_while_stopping_are_not_reported():
    client = make_running_client()
    client._stop.set()
    client._ser.write.side_effect = serial.SerialException('closed')
    with pytest.raises(serial.SerialException):
        client._write_rtcm(b'\xd3')
    assert client.serial_error is None


def test_watchdog_detects_silent_link():
    client = make_running_client()
    with patch('cub_um982.extended_client.time.monotonic', return_value=104.0):
        assert client.get_health_problem(5.0) is None
    with patch('cub_um982.extended_client.time.monotonic', return_value=106.0):
        assert 'no data' in client.get_health_problem(5.0)
        assert client.get_health_problem(0) is None  # disabled
    client.last_line_time = 105.5
    with patch('cub_um982.extended_client.time.monotonic', return_value=106.0):
        assert client.get_health_problem(5.0) is None


def test_received_line_updates_watchdog():
    client = ExtendedUM982Client('/unused')
    lines = iter(['garbage'])

    def read():
        try:
            return next(lines)
        except StopIteration:
            client._stop.set()
            return None

    with patch.object(client, '_readline', side_effect=read):
        client._reader_loop()
    assert client.last_line_time is not None


def test_dead_reader_thread_is_reported():
    client = make_running_client()
    client._reader_thread.is_alive.return_value = False
    assert 'reader thread' in client.get_health_problem(5.0)


def test_stop_reports_stuck_reader_thread():
    client = ExtendedUM982Client('/unused')
    release = threading.Event()
    client._reader_thread = threading.Thread(target=release.wait, daemon=True)
    client._reader_thread.start()
    try:
        assert client.stop(join_timeout=0.05) is False
    finally:
        release.set()
    client._reader_thread.join()
    assert client.stop(join_timeout=0.05) is True


def make_sysfs(tmp_path, device_class='00', vendor='1a86', product='7523'):
    """Build a minimal sysfs tree: hub 1-2.2 -> device 1-2.2.1 -> interface -> ttyUSB3."""
    sys_root = tmp_path / 'sys'
    hub = sys_root / 'devices' / 'platform' / 'usb1' / '1-2' / '1-2.2'
    device = hub / '1-2.2.1'
    interface = device / '1-2.2.1:1.0'
    tty_dir = interface / 'ttyUSB3'
    tty_dir.mkdir(parents=True)
    for d, attrs in (
        (hub, {'busnum': '1', 'devnum': '5', 'idVendor': '1a40', 'idProduct': '0101',
               'bDeviceClass': '09'}),
        (device, {'busnum': '1', 'devnum': '17', 'idVendor': vendor, 'idProduct': product,
                  'bDeviceClass': device_class}),
    ):
        for name, value in attrs.items():
            (d / name).write_text(value + '\n')
    tty_class = sys_root / 'class' / 'tty' / 'ttyUSB3'
    tty_class.mkdir(parents=True)
    (tty_class / 'device').symlink_to(interface)

    dev = tmp_path / 'dev'
    dev.mkdir()
    (dev / 'ttyUSB3').touch()
    (dev / 'ttyUM982').symlink_to(dev / 'ttyUSB3')
    return sys_root, dev, device


def test_find_usb_device_follows_symlink_to_device_not_hub(tmp_path):
    sys_root, dev, device = make_sysfs(tmp_path)
    info = find_usb_device_for_tty(str(dev / 'ttyUM982'), sysfs_root=str(sys_root))
    assert info == UsbDeviceInfo(os.path.realpath(device), '1a86', '7523')


def test_find_usb_device_returns_none_for_non_usb_tty(tmp_path):
    sys_root, _, _ = make_sysfs(tmp_path)
    assert find_usb_device_for_tty('/dev/ttyTHS1', sysfs_root=str(sys_root)) is None


def test_reset_targets_bus_and_device_number(tmp_path):
    sys_root, dev, _ = make_sysfs(tmp_path)
    info = find_usb_device_for_tty(str(dev / 'ttyUM982'), sysfs_root=str(sys_root))
    with patch('cub_um982.usb_reset.os.open', return_value=42) as os_open, \
            patch('cub_um982.usb_reset.os.close') as os_close, \
            patch('cub_um982.usb_reset.fcntl.ioctl') as ioctl:
        node = reset_usb_device(info, dev_root='/dev/bus/usb')
    assert node == '/dev/bus/usb/001/017'
    os_open.assert_called_once_with('/dev/bus/usb/001/017', os.O_WRONLY)
    ioctl.assert_called_once_with(42, USBDEVFS_RESET, 0)
    os_close.assert_called_once_with(42)
    assert USBDEVFS_RESET == 0x5514


def test_reset_refuses_replaced_device(tmp_path):
    sys_root, dev, device = make_sysfs(tmp_path)
    info = find_usb_device_for_tty(str(dev / 'ttyUM982'), sysfs_root=str(sys_root))
    (device / 'idProduct').write_text('6001\n')
    with patch('cub_um982.usb_reset.fcntl.ioctl') as ioctl:
        with pytest.raises(RuntimeError, match='changed'):
            reset_usb_device(info)
    ioctl.assert_not_called()


def test_reset_skips_device_being_enumerated(tmp_path):
    sys_root, dev, device = make_sysfs(tmp_path)
    info = find_usb_device_for_tty(str(dev / 'ttyUM982'), sysfs_root=str(sys_root))
    (device / 'devnum').write_text('0\n')
    with patch('cub_um982.usb_reset.fcntl.ioctl') as ioctl:
        with pytest.raises(RuntimeError, match='re-enumerated'):
            reset_usb_device(info)
    ioctl.assert_not_called()


def test_reset_refuses_hub(tmp_path):
    sys_root, dev, _ = make_sysfs(tmp_path, device_class='09')
    info = find_usb_device_for_tty(str(dev / 'ttyUM982'), sysfs_root=str(sys_root))
    with patch('cub_um982.usb_reset.fcntl.ioctl') as ioctl:
        with pytest.raises(RuntimeError, match='hub'):
            reset_usb_device(info)
    ioctl.assert_not_called()
