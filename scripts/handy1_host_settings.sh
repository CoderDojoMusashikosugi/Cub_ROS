#!/bin/bash

echo "This script setups host settings."
if [ $(whoami) != "root" ]; then
    echo "[error] run as sudo"
    echo "for example: sudo ./handy1_host_settings.sh"
    exit
fi

raspi-config nonint do_serial_hw 0
raspi-config nonint do_serial_cons 1

sudo apt install dhcpcd5

echo "interface wwan0" >> /etc/dhcpcd.conf
echo "metric 4000" >> /etc/dhcpcd.conf

echo "interface eth0" >> /etc/dhcpcd.conf
echo "static ip_address=192.168.1.2/24" >> /etc/dhcpcd.conf

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

