#!/bin/bash

echo "This script setups host settings."
echo "run as sudo"
echo "for example: sudo ./mcub_host_settings.sh"
echo 'KERNEL=="ttyUSB*",  ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyATOM"' > /etc/udev/rules.d/99-atom.rules
echo 'KERNEL=="ttyUSB*",  ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ttySLC1"' > /etc/udev/rules.d/99-slc1.rules
echo "reboot to apply"
