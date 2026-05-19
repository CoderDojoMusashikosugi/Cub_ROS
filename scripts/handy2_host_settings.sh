#!/bin/bash

echo "This script setups host settings."
if [ $(whoami) != "root" ]; then
    echo "[error] run as sudo"
    echo "for example: sudo ./handy2_host_settings.sh"
    exit
fi

raspi-config nonint do_serial_hw 0
raspi-config nonint do_serial_cons 1

raspi-config nonint do_onewire 0
echo 'ACTION=="add", SUBSYSTEM=="w1", KERNEL=="2d-*", RUN+="/bin/sh -c \"chmod 666 /sys/bus/w1/devices/%k/eeprom\""' > /etc/udev/rules.d/99-pico-1wire.rules

sudo apt install dhcpcd5

echo "interface wwan0" >> /etc/dhcpcd.conf
echo "metric 4000" >> /etc/dhcpcd.conf

echo "interface eth0" >> /etc/dhcpcd.conf
echo "static ip_address=192.168.1.2/24" >> /etc/dhcpcd.conf

if ! grep -q "dtoverlay=pps-gpio,gpiopin=23" /boot/firmware/config.txt; then
    echo "dtoverlay=pps-gpio,gpiopin=23" >> /boot/firmware/config.txt
fi
if ! grep -q "gpio=17=op,dh" /boot/firmware/config.txt; then
    echo "gpio=17=op,dh" >> /boot/firmware/config.txt
    echo "dtoverlay=gpio-poweroff,gpiopin=17,active_low=1" >> /boot/firmware/config.txt
fi

echo 'KERNEL=="pps0", OWNER="root", GROUP="root", MODE="0666"' > /etc/udev/rules.d/99-pps.rules

systemctl disable systemd-timesyncd
systemctl stop systemd-timesyncd

mkdir -p ~/.config/autostart
cat > ~/.config/autostart/cub-ros-journal.desktop <<'EOF'
[Desktop Entry]
Type=Application
Name=ROS Logs (cub_ros)
Comment=Open a terminal that tails the cub_ros service journal
Exec=/usr/bin/x-terminal-emulator -T "ROS Logs (cub_ros)" -e bash -lc 'journalctl -f -u cub_ros'
Terminal=false
X-GNOME-Autostart-enabled=true
EOF

echo "setting completed. reboot to apply."

