#!/bin/bash
source scripts/docker_util.sh
$docker_compose --profile runtime down
$docker_compose --profile runtime_vnc down
