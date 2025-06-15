#!/bin/bash
source docker/internal/docker_util.sh
./docker/internal/colcon_ignore.sh all
$docker_compose down
