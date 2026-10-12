#!/bin/bash
# コンテナ内で 1 回だけ実行: .venv を作って依存を入れる(インターネット接続が必要)。
#   ./setup.sh        … 実行用
#   ./setup.sh --dev  … テスト用(pytest, httpx)も入れる
set -euo pipefail
cd "$(dirname "$0")"

if [ ! -x .venv/bin/python ]; then
  # rclpy をシステム側から見せるため --system-site-packages
  if python3 -c 'import ensurepip' 2>/dev/null; then
    python3 -m venv --system-site-packages .venv
  else
    # python3-venv(ensurepip)が無いイメージ向け: pip 無しで作って get-pip.py で入れる
    python3 -m venv --system-site-packages --without-pip .venv
    curl -fsSL https://bootstrap.pypa.io/get-pip.py -o .venv/get-pip.py
    .venv/bin/python .venv/get-pip.py --quiet
    rm -f .venv/get-pip.py
  fi
fi

req=requirements.txt
[ "${1:-}" = "--dev" ] && req=requirements-dev.txt
.venv/bin/python -m pip install --quiet -r "$req"
echo "OK: $(.venv/bin/python -c 'import fastapi, uvicorn; print("fastapi", fastapi.__version__, "uvicorn", uvicorn.__version__)')"
