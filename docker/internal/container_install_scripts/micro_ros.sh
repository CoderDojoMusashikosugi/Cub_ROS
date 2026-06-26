set -e
export DEBIAN_FRONTEND=noninteractive
apt update && apt-get install -y  curl gpg 
source /opt/ros/${ROS_DISTRO}/setup.bash
mkdir -p /prebuilt_ws/src && cd /prebuilt_ws/src
git clone --depth 1 -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git
cd /prebuilt_ws && . /opt/ros/${ROS_DISTRO}/setup.sh
rosdep install --from-paths src --ignore-src -y
colcon build
. /prebuilt_ws/install/local_setup.sh
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
rm -rf log/ build/ src/

# ROSのセットアップをコンテナ側の.bashrcに追加
echo "source /prebuilt_ws/install/local_setup.bash" >> /etc/user.bashrc
