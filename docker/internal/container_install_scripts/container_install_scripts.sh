#!/bin/bash
apt-get update

# コンテナ側にuser.bashrcを作成
touch /etc/user.bashrc
# コンテナ側の.bashrcに、user.bashrcを登録
echo "source /etc/user.bashrc" >> /etc/bash.bashrc
# ROSのセットアップをuser.bashrcに最初に追加
echo "if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi" >> /etc/user.bashrc
echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi" >> /etc/user.bashrc
# この後container_install_scriptsによりuser.bashrcに対してprebuilt系のセットアップが増えていく想定

for script in "$@"; do
  echo "running: $script"
  ls "/container_install_scripts/${script}"
  "/container_install_scripts/${script}"
done
