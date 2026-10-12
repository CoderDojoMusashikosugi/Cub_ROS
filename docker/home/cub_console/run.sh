#!/bin/bash
# コンテナ内で実行: cub_console を起動する(ROS は source 済みであること。bash -i なら .bashrc で source される)。
set -euo pipefail
cd "$(dirname "$0")"
if [ -z "${ROS_DISTRO:-}" ]; then
  echo "警告: ROS_DISTRO が未設定です。診断の購読なしで起動します" >&2
fi
exec .venv/bin/python -m app.main "$@"
