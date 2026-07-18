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
sudo docker run hello-world

# ctrl-pを二度押ししなくて良くなる設定
mkdir /home/$USER/.docker/
echo -e "{\n    \"detachKeys\": \"ctrl-\\\\\\\"\n}" > /home/$USER/.docker/config.json

sudo gpasswd -a $USER docker
sudo service docker restart
echo "sudo無しでdockerコマンドを使用するには、コンピューターを再起動してください。"
