#!/bin/bash
# additional_pkgs.bash への変更をdockerイメージに反映するために実行するスクリプト

set -e

./stop.sh

source docker/internal/docker_util.sh

if [ ${1:-update} != "stay" ]; then
    export VER=`date "+%Y%m%d_%H%M%S"`
    export VER_VNC=$VER
fi

git submodule update --init --recursive # 使うパッケージを全部取得しておく。rosdepでソースパッケージを見るので。

$docker_compose up cub_ros_base --no-start # buildじゃなくてupなのは、一旦pull出来ないか確認するため
$docker_compose down cub_ros_base # buildじゃないのでコンテナ出来ちゃうから、終わったら落としておく

$docker_compose build cub_ros

if [ $USE_VNC_ENV -eq 0 ]; then # 通常のイメージ
    echo "skip building VNC image"
else # VNC対応イメージ
    $docker_compose build cub_ros_vnc
fi

if [ ${1:-update} != "stay" ]; then
    echo VER=$VER > docker/ver.env
    echo VER_VNC=$VER_VNC > docker/ver_vnc.env # VNC側が取り残されないように、ビルドせずともバージョンは上げておく
fi
