#!/bin/bash
# コンテナをpushするためのスクリプト。
# docker loginは手動でやってから、これを実行してほしい。

set -e

source docker/internal/docker_util.sh
$docker_compose --profile runtime_base push
$docker_compose --profile runtime      push
