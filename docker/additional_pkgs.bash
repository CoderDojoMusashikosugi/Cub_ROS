# ここにインストールしたいパッケージを書く
apt update
apt -y install ros-humble-turtlebot3-navigation2 ros-humble-turtlebot3-cartographer
sed -i -e "s/differential/nav2_amcl::DifferentialMotionModel/g" /opt/ros/humble/share/turtlebot3_navigation2/param/burger.yaml
sed -i -e "s/name='cartographer_node',/name='cartographer_node',remappings=[('odom', '\/diff_drive_base_controller\/odom')],/g" /opt/ros/humble/share/turtlebot3_cartographer/launch/cartographer.launch.py

# apt -y install
apt -y install libpcap0.8 ros-humble-librealsense2 ros-humble-realsense2-camera ros-humble-bno055 ros-humble-nmea-navsat-driver ros-humble-rosbag2-storage-mcap
