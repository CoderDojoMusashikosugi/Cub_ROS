#!/bin/bash
# dockerイメージの中身のうち、mac向けのVNC部分のビルドだけをするスクリプト。開発者が実行する機会はそんなに無いはず。

set -e

./stop.sh

source docker/internal/docker_util.sh

echo "Building VNC image for target: $CUB_TARGET"
echo "VNC image will use same version as main image: $CONFIG_IMAGE_VERSION"

$docker_compose build cub_ros_vnc

echo "VNC build successful! VNC image version matches main image: ${CONFIG_IMAGE_VERSION}"
