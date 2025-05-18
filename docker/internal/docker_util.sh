
docker_compose="docker compose -f docker/docker-compose-common.yml -f docker/docker-compose.yml --env-file docker/.env --env-file docker/ver.env --env-file docker/ver_base.env --env-file docker/ver_vnc.env --env-file target.env"

# アーキテクチャを確認
UNAME_M=`uname -m`
if [ $UNAME_M = "x86_64" ]; then
    export ARCH="x86_64"
elif [ $UNAME_M = "aarch64" ] || [ $UNAME_M = "arm64" ]; then
    export ARCH="arm64"
else
    export ARCH="wakaran"
fi

RUNTIME_NVIDIA=`docker info 2>/dev/null | grep nvidia || true`
if [ -n "$RUNTIME_NVIDIA" ]; then # runtime: nvidia を指定可能な環境の場合
    docker_compose=$docker_compose" -f docker/internal/docker-compose-nvidia.yml" # runtime: nvidiaをつける
fi

# 画面出力の表示先設定
if [ -f /System/Library/CoreServices/SystemVersion.plist ]; then # macOSの場合
    export USE_VNC_ENV=1
    export DISPLAY="" # macOSではホストの画面に出力するのを諦める。どうせVNC環境では上書きされるのでこの行の必要性も無いけど。
    export INPUT_GROUP_ID="" #getentが無いので一旦無視、そもそもmacでjoystick使えるか知らない。
else # macOS以外の場合
    export USE_VNC_ENV=0
    export INPUT_GROUP_ID=`getent group input | cut -d: -f3` # ホストのinputのgroup idを取得する。コンテナ内外でinputに割り当てられたidが違う場合があるので、idで指定したい。
    if [ -z "$DISPLAY" ]; then
        export DISPLAY=:0 # DISPLAY環境変数が無い場合は:0で決め打ちしてしまう。もっと良い方法あったら取り替えたい。
    fi
fi

# export USE_VNC_ENV=1 # これを有効化すると、VNC環境を強制オン
