#!/bin/bash

if [ -f /etc/os-release ]; then
  OS_NAME=`lsb_release -i -s`
  if [ $OS_NAME = "Debian" ]; then
    echo Debian
    ./scripts/docker_install_raspi.sh
  elif [ $OS_NAME = "Ubuntu" ]; then
    ./scripts/docker_install_ubuntu.sh
  else
    echo 未対応環境なので自力でdocker入れて下さい・・・
  fi
elif [ -f /System/Library/CoreServices/SystemVersion.plist ]; then
  echo https://matsuand.github.io/docs.docker.jp.onthefly/desktop/mac/install/
  echo ↑の方法でインストールしてください。
else
  echo 未対応環境なので自力でdocker入れて下さい・・・
fi
