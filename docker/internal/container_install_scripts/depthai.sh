#!/bin/bash

export DEBIAN_FRONTEND=noninteractive
apt update && apt -y install ros2-testing-apt-source && apt update && apt -y install ros-${ROS_DISTRO}-depthai-ros-v3
