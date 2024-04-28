#!/bin/bash
source scripts/docker_compose.sh
$docker_compose --profile runtime down
