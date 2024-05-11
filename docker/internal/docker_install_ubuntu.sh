#!/bin/bash
sudo apt-get update
sudo apt-get install -y \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
COMPOSE_VERSION=$(curl -s https://api.github.com/repos/docker/compose/releases/latest | grep 'tag_name' | cut -d\" -f4)
sudo curl -L "https://github.com/docker/compose/releases/download/"$COMPOSE_VERSION"/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
sudo ln -s /usr/local/bin/docker-compose /usr/bin/docker-compose
sudo docker run hello-world

# ctrl-pを二度押ししなくて良くなる設定
mkdir /home/$USER/.docker/
echo -e "{\n    \"detachKeys\": \"ctrl-\\\\\\\"\n}" > /home/$USER/.docker/config.json

sudo gpasswd -a $USER docker
sudo service docker restart
echo "sudo無しでdockerコマンドを使用するには、コンピューターを再起動してください。"

#参考文献
# 最新版のdocker-composeの取得
# https://thr3a.hatenablog.com/entry/20190328/1553730108
# ctrl-pを二度押ししなくて良くなる設定
