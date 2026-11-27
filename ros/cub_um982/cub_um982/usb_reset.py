#!/usr/bin/env python3
"""
usb_reset.py

Helpers to locate the USB device behind a serial tty and to issue a USB port
reset (USBDEVFS_RESET) to that single device.

A port reset makes the parent hub drive a bus reset on the device's port only,
so other devices on the same hub are not disconnected. The device returns to
its default state (clearing e.g. a stalled bulk endpoint) and is re-enumerated
by the kernel. For usb-serial drivers such as ch341 the kernel unbinds and
rebinds the driver, so the tty node is recreated and must be reopened.
"""

import fcntl
import os
from dataclasses import dataclass
from typing import Optional

# _IO('U', 20) from <linux/usbdevice_fs.h>
USBDEVFS_RESET = (ord('U') << 8) | 20

USB_CLASS_HUB = 0x09


@dataclass(frozen=True)
class UsbDeviceInfo:
    """Identity of the USB device that provides a serial tty."""
    sysfs_path: str
    vendor_id: str
    product_id: str

    def describe(self) -> str:
        return f"{os.path.basename(self.sysfs_path)} ({self.vendor_id}:{self.product_id})"


def _read_attr(device_dir: str, name: str) -> Optional[str]:
    try:
        with open(os.path.join(device_dir, name), 'r') as f:
            return f.read().strip()
    except OSError:
        return None


def find_usb_device_for_tty(port: str, sysfs_root: str = '/sys') -> Optional[UsbDeviceInfo]:
    """
    Resolve the USB device (not interface) that provides the given tty.

    Args:
        port: Serial device path; symlinks such as /dev/ttyUM982 are resolved.
        sysfs_root: Root of sysfs (overridable for tests).

    Returns:
        UsbDeviceInfo, or None if the tty is not backed by a USB device.
    """
    tty_name = os.path.basename(os.path.realpath(port))
    tty_link = os.path.join(sysfs_root, 'class', 'tty', tty_name, 'device')
    if not os.path.exists(tty_link):
        return None

    devices_root = os.path.realpath(os.path.join(sysfs_root, 'devices'))
    current = os.path.realpath(tty_link)
    # Walk up from the tty's interface until the USB device directory
    # (the one exposing busnum/devnum) is found.
    while current.startswith(devices_root + os.sep):
        if (os.path.isfile(os.path.join(current, 'busnum'))
                and os.path.isfile(os.path.join(current, 'devnum'))):
            vendor = _read_attr(current, 'idVendor')
            product = _read_attr(current, 'idProduct')
            if vendor is None or product is None:
                return None
            return UsbDeviceInfo(sysfs_path=current, vendor_id=vendor, product_id=product)
        current = os.path.dirname(current)
    return None


def reset_usb_device(device: UsbDeviceInfo, dev_root: str = '/dev/bus/usb') -> str:
    """
    Issue USBDEVFS_RESET to the given USB device.

    The device identity is re-validated right before the reset so that a
    different device that was plugged into the same port is never reset, and
    hubs are refused because resetting a hub disconnects all of its children.

    Returns:
        Path of the usbfs node that was reset.

    Raises:
        RuntimeError / OSError on validation or ioctl failure.
    """
    path = device.sysfs_path
    vendor = _read_attr(path, 'idVendor')
    product = _read_attr(path, 'idProduct')
    if vendor is None or product is None:
        raise RuntimeError(f"USB device {os.path.basename(path)} is no longer present")
    if (vendor, product) != (device.vendor_id, device.product_id):
        raise RuntimeError(
            f"USB device at {os.path.basename(path)} changed "
            f"({device.vendor_id}:{device.product_id} -> {vendor}:{product}); refusing to reset"
        )

    device_class = _read_attr(path, 'bDeviceClass')
    if device_class is not None and int(device_class, 16) == USB_CLASS_HUB:
        raise RuntimeError(f"{os.path.basename(path)} is a USB hub; refusing to reset")

    busnum = _read_attr(path, 'busnum')
    devnum = _read_attr(path, 'devnum')
    if busnum is None or devnum is None:
        raise RuntimeError(f"Cannot read bus/device number of {os.path.basename(path)}")
    if int(devnum) <= 0:
        # devnum reads 0 while the kernel is (re-)enumerating the device,
        # e.g. right after a reset or replug. It is already being reset.
        raise RuntimeError(
            f"{os.path.basename(path)} is being re-enumerated by the kernel; skipping reset"
        )

    node = os.path.join(dev_root, f"{int(busnum):03d}", f"{int(devnum):03d}")
    fd = os.open(node, os.O_WRONLY)
    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    finally:
        os.close(fd)
    return node
