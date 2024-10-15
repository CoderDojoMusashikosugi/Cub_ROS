#!/bin/bash
# dockerイメージの中身のうち、mac向けのVNC部分のビルドだけをするスクリプト。開発者が実行する機会はそんなに無いはず。

set -e

./stop.sh

source docker/internal/docker_util.sh

if [ ${1:-update} != "stay" ]; then
    export VER_VNC=`date "+%Y%m%d_%H%M%S"`
fi

$docker_compose build cub_ros_vnc

if [ ${1:-update} != "stay" ]; then
    echo VER_VNC=$VER_VNC > docker/ver_vnc.env
fi
