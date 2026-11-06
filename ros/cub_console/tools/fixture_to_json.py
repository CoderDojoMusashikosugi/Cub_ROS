#!/usr/bin/env python3
"""フィクスチャ YAML を、WebSocket で配るのと同じ形の JSON にする(node のテスト用)。

例: python3 tools/fixture_to_json.py tests/fixtures/diag_agg_sample.yaml > tests/fixtures/diag_agg_sample.json
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fake_diag_pub import load_fixture  # noqa: E402


def to_json(path: str) -> str:
    arr = load_fixture(path)[0]
    return json.dumps({"type": "diagnostics", "stamp": 0, "status": arr["status"]}, ensure_ascii=False, indent=1) + "\n"


if __name__ == "__main__":
    sys.stdout.write(to_json(sys.argv[1]))
