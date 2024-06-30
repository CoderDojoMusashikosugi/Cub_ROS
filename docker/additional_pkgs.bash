# ここにインストールしたいパッケージを書く
add-apt-repository -y ppa:openrobotics/gazebo11-non-amd64
echo "Package: *
Pin: release o=LP-PPA-openrobotics-gazebo11-non-amd64
Pin-Priority: 499" > /etc/apt/preferences.d/gazebo11-non-amd64
apt update
# apt -y install
