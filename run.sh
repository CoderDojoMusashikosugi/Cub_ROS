#!/bin/bash

# ユーザーごとの設定を書く用のファイルを設置
if [ ! -e docker/home/.user_config.bash ]; then
    cp docker/.default_do_not_edit/user_config.bash docker/home/.user_config.bash
fi

source scripts/docker_compose.sh

export HOST_UID=`id -u`
export HOST_GID=`id -g`

$docker_compose up -d --no-recreate
# -d : コンテナをバックグラウンドで起動
# --no-recreate : docker-composeの変更や環境変数の変更でコンテナを再作成しない。再作成には ./stop.sh を実行。逆に、これを外すとdocker-compose.yml変更のテストに便利。

./scripts/docker_exec.sh