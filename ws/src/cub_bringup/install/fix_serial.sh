echo "This script fixes name of serial port."
echo "run as sudo"
echo "for example: sudo ./fix_serial.sh"
apt -y remove brltty
apt install python3-serial
echo 'KERNEL=="ttyUSB*",  ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyATOM"' > /etc/udev/rules.d/99-atom.rules
echo 'KERNEL=="ttyUSB*",  ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttyBNO055"' > /etc/udev/rules.d/99-bno055.rules
