#!/bin/bash
set -e

./stop.sh

export TAG=`date "+%Y%m%d_%H%M%S"`

source scripts/docker_compose.sh
$docker_compose build

echo TAG=$TAG > docker/tag.env
# docker tag cub_ros:$TAG cub_ros:latest