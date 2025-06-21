#!/bin/bash

set -e

# ユーザーごとの設定を書く用のファイルを設置
if [ ! -e docker/home/.user_config.bash ]; then
    cp docker/.default_do_not_edit/user_config.bash docker/home/.user_config.bash
fi

docker/internal/set_target_env.sh
source docker/internal/docker_util.sh

export HOST_UID=`id -u`
export HOST_GID=`id -g`
container_list=`$docker_compose ps -q`

if [ -n "$container_list" ]; then # コンテナ起動していた場合、それが今の設定と合ってなかったら落とす。
    # CUB_TARGETがコンテナ内外で同じ設定でなければ落とす用の情報収集
    RUNNING_CUB_TARGET=`./docker/internal/docker_exec.sh env | grep CUB_TARGET`
    IDEAL_CUB_TARGET=`source ./target.env && echo $CUB_TARGET`
    IDEAL_CUB_TARGET="CUB_TARGET=$IDEAL_CUB_TARGET"

    # 実行中のイメージ名が設定のイメージ名と合ってなければ落とす用の情報収集
    # VNC環境かどうかに応じて適切なコンテナ名を使用
    if [ $USE_VNC_ENV -eq 0 ]; then
        RUNNING_IMAGE_NAME=`docker inspect --format='{{.Config.Image}}' cub_ros 2>/dev/null` || true
        IDEAL_IMAGE_NAME=ghcr.io/coderdojomusashikosugi/${CONFIG_IMAGE_NAME}:${CONFIG_IMAGE_VERSION}_${ARCH}
    else
        RUNNING_IMAGE_NAME=`docker inspect --format='{{.Config.Image}}' cub_ros_vnc 2>/dev/null` || true
        IDEAL_IMAGE_NAME=ghcr.io/coderdojomusashikosugi/${CONFIG_IMAGE_NAME}_vnc:${CONFIG_IMAGE_VERSION}_${ARCH}
    fi

    if [ $IDEAL_CUB_TARGET != $RUNNING_CUB_TARGET ]; then
        # CUB_TARGETがコンテナ内外で同じ設定でなければ落とす
        # echo "CUB_TARGET updated. recreating container..."
        # ./stop.sh
        # container_list=""
        # というのは止めて警告に留める
        echo "現在起動中のコンテナは、設定されたCUB_TARGETに対応したものではありません。"
        echo "一旦exitし、ホスト環境で./stop.shを実行してから再度./run.shを試してください。"
    elif [ -n "$RUNNING_IMAGE_NAME" ] && [ $RUNNING_IMAGE_NAME != $IDEAL_IMAGE_NAME ]; then
        # 実行中のイメージ名が設定のイメージ名と合ってなければ落とす。
        # イメージ名取得出来ない場合は一旦無視する。cub_ros以外、すなわちvncコンテナを起動してる場合を想定。
        # echo "image tag updated. recreating container..."
        # ./stop.sh
        # container_list=""
        # というのは止めて警告に留める
        echo "現在起動中のコンテナは、設定されたバージョンのものではありません。"
        echo "一旦exitし、ホスト環境で./stop.shを実行してから再度./run.shを試してください。"
    fi
fi

if [ -z "$container_list" ]; then # コンテナ起動してなければ起動
    git submodule update --init --recursive # dockerイメージをbuildする際には手元のROS pkgが依存するapt pkgを調べるので、その前に全部揃えておく。
    ./docker/internal/colcon_ignore.sh

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
