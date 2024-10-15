#!/bin/bash
# コンテナをpushするためのスクリプト。
# docker loginは手動でやってから、これを実行してほしい。

set -e

source docker/internal/docker_util.sh
$docker_compose push cub_ros_base
$docker_compose push cub_ros

if [ $USE_VNC_ENV -ne 0 ]; then # VNC対応コンテナを利用する場合
    $docker_compose push cub_ros_vnc
fi

