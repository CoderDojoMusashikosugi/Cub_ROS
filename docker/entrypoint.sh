#!/bin/sh

# username: cub
# password: same as username   (openssl passwd $USER_NAME)
groupadd -g $HOST_GID $USER_NAME
useradd -u $HOST_UID -g $HOST_GID -m $USER_NAME --password '$1$1MParYB8$R8c8vAbst5CsASYDC8Bmw1'
echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
gpasswd -a $USER_NAME sudo

/bin/bash
