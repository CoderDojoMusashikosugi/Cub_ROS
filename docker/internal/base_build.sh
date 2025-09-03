#!/bin/bash
# dockerイメージの中身のうち、ベース部分のビルドだけをするスクリプト。開発者が実行する機会はそんなに無いはず。
# 開発者がdockerイメージにパッケージを追加するたび全部のビルドが走ると時間かかるので、どのイメージでも共通で入っているべきROSなどを入れたベースイメージを作って配布することにした。それを生成するためのスクリプト。

set -e

./stop.sh

source docker/internal/docker_util.sh
docker/internal/colcon_ignore.sh

UPDATE_VERSION=false
if [ ${1:-update} != "stay" ]; then
    export CONFIG_BASE_IMAGE_VERSION=`date "+%Y%m%d_%H%M%S"`
    echo The tag name will be ${CONFIG_BASE_IMAGE_VERSION}.
    UPDATE_VERSION=true
fi

$docker_compose build cub_ros_base

# Update version only after successful build
if [ "$UPDATE_VERSION" = true ]; then
    # Source config_utils.sh to get access to update functions
    source docker/internal/config_utils.sh
    
    # Update BASE_IMAGE_VERSION for the config file matching the IMAGE_TYPE
    update_base_image_version "docker/environment/${CONFIG_IMAGE_TYPE}.conf" "$CONFIG_BASE_IMAGE_VERSION"
    
    echo "Base build successful! Updated BASE_IMAGE_VERSION to ${CONFIG_BASE_IMAGE_VERSION} for ${CONFIG_IMAGE_TYPE}.conf"
fi
