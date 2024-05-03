#!/bin/bash
# dockerイメージの中身のうち、ベース部分のビルドだけをするスクリプト。開発者が実行する機会はそんなに無いはず。
# 開発者がdockerイメージにパッケージを追加するたび全部のビルドが走ると時間かかるので、どのイメージでも共通で入っているべきROSなどを入れたベースイメージを作って配布することにした。それを生成するためのスクリプト。

set -e

./stop.sh

export VER_BASE=`date "+%Y%m%d_%H%M%S"`

source scripts/docker_compose.sh
$docker_compose --profile runtime_base build

echo VER_BASE=$VER_BASE > docker/ver_base.env
