# ここにインストールしたいパッケージを書く
add-apt-repository -y ppa:openrobotics/gazebo11-non-amd64
apt update
# apt -y install
apt -y install libpcap0.8 ros-humble-librealsense2 ros-humble-realsense2-camera ros-humble-bno055 ros-humble-nmea-navsat-driver ros-humble-rosbag2-storage-mcap