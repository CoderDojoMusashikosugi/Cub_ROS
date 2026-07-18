#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source_in_container="/home/cub/colcon_ws/src/cub/mcub_bringup/urdf/mcub.urdf.xacro"
destination="$repo_root/unity/CubSim/Assets/StreamingAssets/Robots/mcub.urdf"

cd "$repo_root"
./docker/run_in_container.sh true >/dev/null
container_source_root="$(docker inspect cub_ros --format '{{range .Mounts}}{{if eq .Destination "/home/cub/colcon_ws/src/cub"}}{{.Source}}{{end}}{{end}}')"
if [ -z "$container_source_root" ]; then
  echo "Could not find the cub_ros source bind mount." >&2
  exit 1
fi
temporary_on_host="$container_source_root/mcub_bringup/urdf/.mcub.unity.generated.urdf"

cleanup() {
  rm -f "$temporary_on_host"
}
trap cleanup EXIT

mkdir -p "$(dirname "$destination")"
rm -f "$temporary_on_host"
./docker/run_in_container.sh \
  "ros2 run xacro xacro '$source_in_container' -o '/home/cub/colcon_ws/src/cub/mcub_bringup/urdf/.mcub.unity.generated.urdf'"
test -s "$temporary_on_host"
cp "$temporary_on_host" "$destination"
echo "Updated $destination"
