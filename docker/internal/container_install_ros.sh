#!/bin/bash

# First argument indicates whether full ROS installation is needed
NEEDS_ROS_INSTALL=${1:-true}

export DEBIAN_FRONTEND=noninteractive
apt update

if [ "$NEEDS_ROS_INSTALL" = "true" ]; then
    echo "Installing ROS from apt source..."
    sudo apt update && sudo apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
    sudo apt install /tmp/ros2-apt-source.deb

    apt update
    apt upgrade

    apt -y install ros-humble-desktop
else
    echo "ROS is already installed in base image, skipping apt source setup..."
fi
apt -y install ros-humble-foxglove-bridge ros-dev-tools
