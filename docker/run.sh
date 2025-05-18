#!/bin/bash

# ユーザーごとの設定を書く用のファイルを設置
if [ ! -e docker/home/.user_config.bash ]; then
    cp docker/.default_do_not_edit/user_config.bash docker/home/.user_config.bash
fi

source docker/internal/docker_util.sh

export HOST_UID=`id -u`
export HOST_GID=`id -g`
container_list=`$docker_compose ps -q`

if [ -z "$container_list" ]; then # コンテナ起動してなければ起動
    git submodule update --init --recursive # dockerイメージをbuildする際には手元のROS pkgが依存するapt pkgを調べるので、その前に全部揃えておく。

    $docker_compose up cub_ros_base --no-start --no-recreate
    $docker_compose down cub_ros_base

    # VNC対応コンテナを使うか否かを設定。基本macOSではVNC側が起動。その他でもUSE_VNC_ENV=1を設定すると起動する。
    if [ $USE_VNC_ENV -eq 0 ]; then # 通常のコンテナ起動
        xhost + # ディスプレイ表示用
        $docker_compose up cub_ros -d --no-recreate
    else # VNC対応コンテナ起動。全てのグラフィックがVNC環境に出る。
        $docker_compose up cub_ros --no-start --no-recreate
        $docker_compose down cub_ros
        $docker_compose up cub_ros_vnc -d --no-recreate # macOS等、3Dアプリの画面表示できない環境向け。ネットワーク設定が違うのに注意。
        echo rviz用VNC環境へは、こちら↓にアクセス!
        echo \ \ \ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓
        echo →  http://localhost:6080  ←
        echo \ \ \ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑
    fi
    
    sleep 1 # 低性能環境で、プロンプトが I have no name!@docker:/home/cub$ になっちゃう現象対策。それでも起きたら一旦exitしてから再度./run.shすると治る。
fi

if [[ -z "$INVOCATION_ID" ]]; then
    ./docker/internal/docker_exec.sh /bin/bash # systemdからの起動でなければ、オマケとしてexecまで実行する。
fi
