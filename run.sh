#!/bin/bash

# ユーザーごとの設定を書く用のファイルを設置
if [ ! -e docker/home/.user_config.bash ]; then
    cp docker/.default_do_not_edit/user_config.bash docker/home/.user_config.bash
fi

source scripts/docker_compose.sh

export HOST_UID=`id -u`
export HOST_GID=`id -g`
export INPUT_GROUP_ID=`getent group input | cut -d: -f3` # ホストのinputのgroup idを取得する。コンテナ内外でinputに割り当てられたidが違う場合があるので、idで指定したい。

container_list=`$docker_compose ps -q`

if [ -z "$container_list" ]; then # コンテナ起動してなければ起動
    $docker_compose --profile runtime_base up --no-start --no-recreate
    $docker_compose --profile runtime_base down

    $docker_compose --profile runtime up -d --no-recreate
    # --profile runtime : runtime用(runtime_baseじゃない)のコンテナのみを起動する
    # -d : コンテナをバックグラウンドで起動
    # --no-recreate : docker-composeの変更や環境変数の変更でコンテナを再作成しない。再作成には ./stop.sh を実行。逆に、これを外すとdocker-compose.yml変更のテストに便利。

    sleep 1 # 低性能環境で、プロンプトが I have no name!@docker:/home/cub$ になっちゃう現象対策。それでも起きたら一旦exitしてから再度./run.shすると治る。
fi

./scripts/docker_exec.sh
