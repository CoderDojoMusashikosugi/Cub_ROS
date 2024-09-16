docker_compose="docker compose -f docker/docker-compose.yml"

# --env-fileで読み込みたかったんだけど、古いdockerの環境で動かなかったので仕方なく...
# docker_compose="docker compose -f docker/docker-compose.yml --env-file docker/.env --env-file docker/ver.env --env-file docker/ver_base.env"
source docker/.env
source docker/ver.env
source docker/ver_base.env
source target.env
export USER_NAME=${USER_NAME}
export VER=${VER}
export VER_BASE=${VER_BASE}
export CUB_TARGET=${CUB_TARGET}

UNAME_M=`uname -m`
if [ $UNAME_M = "x86_64" ]; then
    export ARCH="x86_64"
elif [ $UNAME_M = "aarch64" ] || [ $UNAME_M = "arm64" ]; then
    export ARCH="arm64"
else
    export ARCH="wakaran"
fi
