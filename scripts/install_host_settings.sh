#!/bin/bash

echo "This script setups host settings."
echo "run as sudo"
echo "for example: sudo ./install_host_settings.sh"
echo 'KERNEL=="ttyCH341USB*",  ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttyBNO055"' > /etc/udev/rules.d/99-bno055.rules
echo 'KERNEL=="ttyUSB*",  ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyATOM"' > /etc/udev/rules.d/99-atom.rules
echo 'KERNEL=="ttyACM*",  ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="ttyGPS"' > /etc/udev/rules.d/99-gps.rules
echo 'KERNEL=="ttyUSB*", ENV{ID_SERIAL_SHORT}=="a6124a42485bed1185344fe81c62bc44", SYMLINK+="ttySLC1L"' > /etc/udev/rules.d/99-slc1-l.rules
echo 'KERNEL=="ttyUSB*", ENV{ID_SERIAL_SHORT}=="6ab98ed24e5bed11940851e81c62bc44", SYMLINK+="ttySLC1R"' > /etc/udev/rules.d/99-slc1-r.rules
echo 'KERNEL=="ttyUSB*", ENV{ID_SERIAL_SHORT}=="b69c7db1d29de8118347301338b01545", SYMLINK+="ttyMULIMU"' > /etc/udev/rules.d/99-multiIMU.rules
echo "reboot to apply"
