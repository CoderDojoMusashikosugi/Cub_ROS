#!/bin/bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash
exec roslaunch map_cleaner cub_batch.launch config:=/output/config.yaml
