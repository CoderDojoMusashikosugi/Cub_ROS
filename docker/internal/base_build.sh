#!/bin/bash
# dockerイメージの中身のうち、ベース部分のビルドだけをするスクリプト。開発者が実行する機会はそんなに無いはず。
# 開発者がdockerイメージにパッケージを追加するたび全部のビルドが走ると時間かかるので、どのイメージでも共通で入っているべきROSなどを入れたベースイメージを作って配布することにした。それを生成するためのスクリプト。

set -e

./stop.sh

source docker/internal/docker_util.sh

if [ ${1:-update} != "stay" ]; then
    export VER_BASE=`date "+%Y%m%d_%H%M%S"`
    echo The tag name will be ${VER_BASE}.
fi

$docker_compose build cub_ros_base

if [ ${1:-update} != "stay" ]; then
    echo VER_BASE=$VER_BASE > docker/ver_base.env
    cat docker/ver_base.env
    echo The tag name is ${VER_BASE}.
fi
