#!/bin/bash
set -euo pipefail
exec python3 "$(dirname "$(realpath "$0")")/internal/map_cleaner.py" run "$@"
