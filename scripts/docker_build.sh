#!/bin/bash
# additional_pkgs.bash への変更をdockerイメージに反映するために実行するスクリプト

set -e

./stop.sh

export TAG=`date "+%Y%m%d_%H%M%S"`

source scripts/docker_compose.sh
$docker_compose --profile runtime_base up --no-start # buildじゃなくてupなのは、一旦pull出来ないか確認するため
$docker_compose --profile runtime_base down # buildじゃないのでコンテナ出来ちゃうから、終わったら落としておく

$docker_compose --profile runtime build

echo TAG=$TAG > docker/ver.env
