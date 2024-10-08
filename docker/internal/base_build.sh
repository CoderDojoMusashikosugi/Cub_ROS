#!/bin/bash
# dockerイメージの中身のうち、ベース部分のビルドだけをするスクリプト。開発者が実行する機会はそんなに無いはず。
# 開発者がdockerイメージにパッケージを追加するたび全部のビルドが走ると時間かかるので、どのイメージでも共通で入っているべきROSなどを入れたベースイメージを作って配布することにした。それを生成するためのスクリプト。

set -e

./stop.sh

source docker/internal/docker_util.sh

if [ ${1:-update} != "stay" ]; then
    export VER_BASE=`date "+%Y%m%d_%H%M%S"`
fi

$docker_compose --profile runtime_base build

if [ ${1:-update} != "stay" ]; then
    echo VER_BASE=$VER_BASE > docker/ver_base.env
fi
