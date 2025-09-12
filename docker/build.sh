#!/bin/bash
# configuration-based docker image builder

set -e

./stop.sh

source docker/internal/docker_util.sh
docker/internal/colcon_ignore.sh

UPDATE_VERSION=false
if [ ${1:-update} != "stay" ]; then
    # Generate version based on current timestamp for main image only
    export CONFIG_IMAGE_VERSION=`date "+%Y%m%d_%H%M%S"`
    # Use existing base image version from config file
    # CONFIG_BASE_IMAGE_VERSION is already loaded from config_utils.sh
    UPDATE_VERSION=true
fi

git submodule update --init --recursive # 使うパッケージを全部取得しておく。rosdepでソースパッケージを見るので。

echo "Building images for target: $CUB_TARGET"
echo "Base image: $CONFIG_BASE_IMAGE"
echo "Image type: $CONFIG_IMAGE_TYPE"
echo "Additional packages: $CONFIG_ADDITIONAL_PKGS"

echo "Trying to pull base image: ghcr.io/coderdojomusashikosugi/cub_ros_base:${CONFIG_BASE_IMAGE_VERSION}_${CONFIG_IMAGE_TYPE}_${ARCH}"
if ! $docker_compose up cub_ros_base --no-start; then
    echo "Base image pull failed. Base image will be built automatically when building main image."
fi
$docker_compose down cub_ros_base # buildじゃないのでコンテナ出来ちゃうから、終わったら落としておく

echo "Building main image: ${CONFIG_IMAGE_NAME}"
$docker_compose build cub_ros

if [ $USE_VNC_ENV -eq 0 ]; then # 通常のイメージ
    echo "skip building VNC image"
else # VNC対応イメージ
    $docker_compose build cub_ros_vnc
fi

# Update version only after successful build
if [ "$UPDATE_VERSION" = true ]; then
    # Source config_utils.sh to get access to update functions
    source docker/internal/config_utils.sh
    
    # Update IMAGE_VERSION for the config file matching the IMAGE_TYPE
    update_image_version "docker/environment/${CONFIG_IMAGE_TYPE}.conf" "$CONFIG_IMAGE_VERSION"
    
    # Note: Base image version is only updated by base_build.sh
    # build.sh uses existing base image version and doesn't update it
    
    echo "Build successful! Updated IMAGE_VERSION to $CONFIG_IMAGE_VERSION for ${CONFIG_IMAGE_TYPE}.conf"
    echo "  BASE_IMAGE_VERSION: $CONFIG_BASE_IMAGE_VERSION (unchanged)"
fi
