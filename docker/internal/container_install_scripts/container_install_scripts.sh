#!/bin/bash
apt-get update

# ROSのセットアップをコンテナ側の.bashrcに最初に追加
echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /etc/bash.bashrc
# この後container_install_scriptsによりprebuilt系のセットアップが増えていく想定

for script in "$@"; do
  echo "running: $script"
  ls "/container_install_scripts/${script}"
  "/container_install_scripts/${script}"
done
