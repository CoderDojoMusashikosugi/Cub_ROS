# ここにインストールしたいパッケージを書く
add-apt-repository -y ppa:openrobotics/gazebo11-non-amd64
echo "Package: *
Pin: release o=LP-PPA-openrobotics-gazebo11-non-amd64
Pin-Priority: 499" > /etc/apt/preferences.d/gazebo11-non-amd64
apt update
# apt -y install
apt -y install libpcap0.8 ros-humble-librealsense2 ros-humble-realsense2-camera ros-humble-bno055 ros-humble-nmea-navsat-driver ros-humble-rosbag2-storage-mcap
apt -y install ros-humble-turtlebot3-navigation2
sed -i -e "s/differential/nav2_amcl::DifferentialMotionModel/g" /opt/ros/humble/share/turtlebot3_navigation2/param/burger.yaml
