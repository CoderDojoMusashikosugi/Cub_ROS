#!/bin/bash
# 設定に基づくイメージ&名前のないイメージを削除する。
# ただし、念のために手元でビルドした最新版だけは保持する

source docker/internal/docker_util.sh

echo "Removing old images for: $CONFIG_IMAGE_NAME"

# Function to remove old images for a given image pattern
remove_old_images() {
    local image_pattern=$1
    echo "Processing: $image_pattern"
    
    # 最新のイメージを検索
    images=`docker images $image_pattern -q`
    latest_image=`echo $images | awk '{print $1}'`
    
    # 最新のイメージ以外を削除
    if [ -n "$latest_image" ]; then
        old_images=`docker images $image_pattern --filter "before=$latest_image" -q`
        if [ -n "$old_images" ]; then
            echo "Removing old images for $image_pattern"
            docker rmi -f $old_images
        else
            echo "No old images found for $image_pattern"
        fi
    else
        echo "No images found for $image_pattern"
    fi
}

# Remove old images for current configuration
remove_old_images "ghcr.io/coderdojomusashikosugi/${CONFIG_IMAGE_NAME}"
remove_old_images "ghcr.io/coderdojomusashikosugi/${CONFIG_IMAGE_NAME}_base"
remove_old_images "ghcr.io/coderdojomusashikosugi/${CONFIG_IMAGE_NAME}_vnc"

# Remove dangling images
dangling_images=`docker images -f "dangling=true" -q`
if [ -n "$dangling_images" ]; then
    echo "Removing dangling images"
    docker rmi -f $dangling_images
else
    echo "No dangling images found"
fi