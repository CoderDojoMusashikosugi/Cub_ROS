#!/bin/bash

echo "This script setups host settings."
echo "run as sudo"
echo "for example: sudo ./mcub_host_settings.sh"
echo 'KERNEL=="ttyUSB*",  ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttyBNO055"' > /etc/udev/rules.d/99-bno055.rules
echo "reboot to apply"
