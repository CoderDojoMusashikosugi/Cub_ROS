#!/bin/bash

# ユーザーごとの設定を書く用のファイルを設置
if [ ! -e docker/home/.user_config.bash ]; then
    cp docker/.default_do_not_edit/user_config.bash docker/home/.user_config.bash
fi

source scripts/docker_util.sh

export HOST_UID=`id -u`
export HOST_GID=`id -g`
if [ -f /System/Library/CoreServices/SystemVersion.plist ]; then # macOSの場合
    # macでディスプレイ出力させるための設定、ただしrvizは出ない。
    export DISPLAY="docker.for.mac.host.internal:0"
    export INPUT_GROUP_ID="" #getentが無いので一旦無視、そもそもmacでjoystick使えるか知らない。
    export ENABLE_REMOTE_RVIZ=1 # rvizをvnc側で表示させる
else # macOS以外の場合
    export INPUT_GROUP_ID=`getent group input | cut -d: -f3` # ホストのinputのgroup idを取得する。コンテナ内外でinputに割り当てられたidが違う場合があるので、idで指定したい。
    export ENABLE_REMOTE_RVIZ=0 # rvizをホストで表示
fi

container_list=`$docker_compose ps -q`

if [ -z "$container_list" ]; then # コンテナ起動してなければ起動
    xhost + # ディスプレイ表示用
    $docker_compose --profile runtime_base up --no-start --no-recreate
    $docker_compose --profile runtime_base down

    $docker_compose --profile runtime up -d --no-recreate
    # --profile runtime : runtime用(runtime_baseじゃない)のコンテナのみを起動する
    # -d : コンテナをバックグラウンドで起動
    # --no-recreate : docker-composeの変更や環境変数の変更でコンテナを再作成しない。再作成には ./stop.sh を実行。逆に、これを外すとdocker-compose.yml変更のテストに便利。

    if [ -f /System/Library/CoreServices/SystemVersion.plist ]; then # macOSの場合
        $docker_compose --profile runtime_vnc up -d --no-recreate # rviz向けにvncコンテナ起動する
        echo rviz用VNC環境へは、こちら↓にアクセス!
        echo \ \ \ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ 
        echo →  http://localhost:6080  ←
        echo \ \ \ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ 
    fi

    sleep 1 # 低性能環境で、プロンプトが I have no name!@docker:/home/cub$ になっちゃう現象対策。それでも起きたら一旦exitしてから再度./run.shすると治る。
fi

./scripts/docker_exec.sh
