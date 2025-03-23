#!/bin/bash
source docker/internal/docker_util.sh
# $docker_compose exec --user `id -u`:`id -g` cub_ros_vnc /bin/bash

if [ $USE_VNC_ENV -eq 0 ]; then
    $docker_compose exec --user `id -u`:`id -g` cub_ros "$@"
else
    $docker_compose exec --user `id -u`:`id -g` cub_ros_vnc "$@"
fi
