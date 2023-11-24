echo "This script fixes name of serial port."
echo "run as sudo"
echo "for example: sudo ./fix_serial.sh"
echo 'KERNEL=="ttyUSB*",  ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyATOM"' > /etc/udev/rules.d/99-atom.rules
