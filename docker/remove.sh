#!/bin/bash
# cub_rosと名のつくイメージ&名前のないイメージを全削除する。
# ただし、念のために手元でビルドした最新版だけは保持する


# 最新のcub_rosイメージを検索
images=`docker images ghcr.io/coderdojomusashikosugi/cub_ros -q`
latest_image=`echo $images | awk '{print $1}'`

# 最新のcub_rosイメージ以外を削除
if [ -n "$latest_image" ]; then
    docker rmi -f `docker images ghcr.io/coderdojomusashikosugi/cub_ros --filter "before=$latest_image" -q`
fi

# 最新のcub_ros_baseイメージを検索
images=`docker images ghcr.io/coderdojomusashikosugi/cub_ros_base -q`
latest_image=`echo $images | awk '{print $1}'`

# 最新のcub_ros_baseイメージ以外を削除
if [ -n "$latest_image" ]; then
    docker rmi -f `docker images ghcr.io/coderdojomusashikosugi/cub_ros_base --filter "before=$latest_image" -q`
fi

# 最新のcub_ros_vncイメージを検索
images=`docker images ghcr.io/coderdojomusashikosugi/cub_ros_vnc -q`
latest_image=`echo $images | awk '{print $1}'`

# 最新のcub_ros_vncイメージ以外を削除
if [ -n "$latest_image" ]; then
    docker rmi -f `docker images ghcr.io/coderdojomusashikosugi/cub_ros_vnc --filter "before=$latest_image" -q`
fi