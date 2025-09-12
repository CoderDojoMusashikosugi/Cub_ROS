#!/bin/bash

export DEBIAN_FRONTEND=noninteractive
apt update

echo "Installing ROS from apt source..."
apt update && apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
apt -y install /tmp/ros2-apt-source.deb
rm /tmp/ros2-apt-source.deb

apt update
apt -y upgrade

apt -y install ros-humble-desktop
apt -y install ros-humble-foxglove-bridge ros-dev-tools
rosdep init
rosdep update --rosdistro=humble
