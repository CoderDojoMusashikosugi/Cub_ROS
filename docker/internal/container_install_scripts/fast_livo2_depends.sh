#!/bin/bash
export DEBIAN_FRONTEND=noninteractive
#https://github.com/Rhymer-Lcy/FAST-LIVO2-ROS2-MID360-Fisheye/tree/main

# Sophus
cd /
git clone https://github.com/strasdat/Sophus.git -b 1.22.10
cd Sophus
mkdir build && cd build && cmake ..
make -j
make install
cd / && rm -rf Sophus

# Vikit
apt update && apt -y install libopencv-dev
source /opt/ros/humble/setup.bash
mkdir -p /fast_livo2_depends_ws/src && cd /fast_livo2_depends_ws/src
git clone https://github.com/Rhymer-Lcy/rpg_vikit_ros2_fisheye.git
cd rpg_vikit_ros2_fisheye && rm -rf vikit_py

cd /fast_livo2_depends_ws
rosdep install --from-paths src --ignore-src -y --skip-keys="cmake_modules"
colcon build
rm -rf log/ build/ src/

# コンテナ側の.bashrcに追加
echo "source /fast_livo2_depends_ws/install/local_setup.bash" >> /etc/user.bashrc
