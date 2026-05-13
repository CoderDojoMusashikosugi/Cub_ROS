#!/bin/bash
source docker/internal/docker_util.sh
./docker/internal/colcon_ignore.sh all
$docker_compose down

# コンテナ終了時にリンクを削除する
rm -f docker/home/ext_storage
