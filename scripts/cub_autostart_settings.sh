#!/bin/bash

echo "This script setups host autostart settings."
if [ $(whoami) != "root" ]; then
    echo "[error] run as sudo"
    echo "for example: sudo ./cub_autostart_settings.sh"
    exit
fi

USER_NAME=$SUDO_USER
CUB_ROS_WS=$(cd $(dirname $0);cd ..;pwd)

echo "[Unit]
Description=Cub_ROS_docker
After=docker.service docker.socket gdm.service

[Service]
User=$USER_NAME
Type=simple
WorkingDirectory=$CUB_ROS_WS
ExecStartPre=/bin/sleep 5
ExecStartPre=$CUB_ROS_WS/run.sh
ExecStart=$CUB_ROS_WS/docker/internal/docker_exec.sh /bin/bash -c 'source /home/cub/colcon_ws/install/setup.bash && source ~/.user_config.bash && ros2 launch cub_bringup launch_at_boot.launch.py'

[Install]
WantedBy=graphical.target" > /etc/systemd/system/cub_ros.service
systemctl daemon-reload
systemctl enable cub_ros.service

echo "setting completed. reboot to apply."
