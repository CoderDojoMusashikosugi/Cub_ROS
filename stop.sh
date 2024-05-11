#!/bin/bash
source docker/internal/docker_util.sh
$docker_compose --profile runtime down
$docker_compose --profile runtime_vnc down
