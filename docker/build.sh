#!/bin/bash
# additional_pkgs.bash への変更をdockerイメージに反映するために実行するスクリプト

set -e

./stop.sh

source docker/internal/docker_util.sh

if [ ${1:-update} != "stay" ]; then
    export VER=`date "+%Y%m%d_%H%M%S"`
fi

$docker_compose --profile runtime_base up --no-start # buildじゃなくてupなのは、一旦pull出来ないか確認するため
$docker_compose --profile runtime_base down # buildじゃないのでコンテナ出来ちゃうから、終わったら落としておく

$docker_compose --profile runtime build

if [ ${1:-update} != "stay" ]; then
    echo VER=$VER > docker/ver.env
fi
