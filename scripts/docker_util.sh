docker_compose="docker compose -f docker/docker-compose.yml --env-file docker/.env --env-file docker/ver.env --env-file docker/ver_base.env"

UNAME_M=`uname -m`
if [ $UNAME_M = "x86_64" ]; then
    export ARCH="x86_64"
elif [ $UNAME_M = "aarch64" ] || [ $UNAME_M = "arm64" ]; then
    export ARCH="arm64"
else
    export ARCH="wakaran"
fi
