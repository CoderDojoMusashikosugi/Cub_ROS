#!/bin/bash

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get -y install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get -y install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo docker run hello-world

# ctrl-pを二度押ししなくて良くなる設定
mkdir /home/$USER/.docker/
echo -e "{\n    \"detachKeys\": \"ctrl-\\\\\\\"\n}" > /home/$USER/.docker/config.json

sudo gpasswd -a $USER docker
sudo service docker restart
echo "sudo無しでdockerコマンドを使用するには、コンピューターを再起動してください。"
