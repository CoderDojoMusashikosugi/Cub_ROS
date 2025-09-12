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

# cub_rosサービスが起動しているか確認 (-qでコンテナIDのみ取得)
RUNNING_CONTAINER_ID=$($docker_compose ps -q cub_ros 2>/dev/null)

# コンテナが起動していなければ起動する
if [ -z "$RUNNING_CONTAINER_ID" ]; then
  echo "Container 'cub_ros' is not running. Starting it now..."
  ./run.sh < /dev/null
  echo "Container started."
fi

# コンテナ内でコマンドを実行
echo "Executing command in container: $@"
# .bashrcを読み込むためにbash -icを使い、渡された引数をすべて単一のコマンド文字列として実行
./docker/internal/docker_exec.sh bash -ic "$*"
