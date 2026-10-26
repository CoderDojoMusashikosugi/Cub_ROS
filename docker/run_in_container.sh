#!/bin/bash

# このスクリプトは、プロジェクトのルートディレクトリから実行されることを想定しています。

set -e # エラーが発生したらスクリプトを停止

# 引数が渡されなかった場合は使い方を表示して終了
if [ $# -eq 0 ]; then
  echo "Error: No command provided."
  echo "Usage: $0 <command_to_run_in_container>"
  exit 1
fi

# コンテナの状態確認に必要なユーティリティを読み込む
source docker/internal/docker_util.sh

# docker_exec.shと同じサービスを確認する（macOSはVNCサービス）。
CONTAINER_SERVICE=cub_ros
if [ "$USE_VNC_ENV" -eq 1 ]; then
  CONTAINER_SERVICE=cub_ros_vnc
fi
RUNNING_CONTAINER_ID=$($docker_compose ps --status running -q "$CONTAINER_SERVICE" 2>/dev/null)

# コンテナが起動していなければ起動する
if [ -z "$RUNNING_CONTAINER_ID" ]; then
  echo "Container '$CONTAINER_SERVICE' is not running. Starting it now..."
  ./run.sh < /dev/null
  echo "Container started."
fi

# コンテナ内でコマンドを実行
echo "Executing command in container: $@"
# .bashrcを読み込むためにbash -icを使い、渡された引数をすべて単一のコマンド文字列として実行
./docker/internal/docker_exec.sh bash -ic "$*"
