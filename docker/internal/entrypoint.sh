#!/bin/sh

# username: cub
# password: same as username   (openssl passwd $USER_NAME)
groupadd -g $HOST_GID $USER_NAME
useradd -u $HOST_UID -g $HOST_GID -m $USER_NAME --password '$1$1MParYB8$R8c8vAbst5CsASYDC8Bmw1' -s /bin/bash
echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
gpasswd -a $USER_NAME sudo
su - $USER_NAME -c "/bin/bash -c 'source /opt/ros/humble/setup.bash &&
	    source ~/.user_config.bash &&
	        ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765'"
/bin/bash
