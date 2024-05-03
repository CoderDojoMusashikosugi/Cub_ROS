#!/bin/bash
source scripts/docker_util.sh

container_list=`$docker_compose ps -q`

if [ -z "$container_list" ]; then
    echo nai
fi