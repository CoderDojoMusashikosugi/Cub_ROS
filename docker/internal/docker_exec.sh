#!/bin/bash
source scripts/docker_util.sh
$docker_compose exec --user `id -u`:`id -g` cub_ros /bin/bash
