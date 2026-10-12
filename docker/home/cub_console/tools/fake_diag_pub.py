#!/usr/bin/env python3
"""フィクスチャ(`ros2 topic echo` の YAML)を DiagnosticArray として publish するテスト用ツール。

例: python3 tools/fake_diag_pub.py --jitter --stop-after 20
"""
from __future__ import annotations

import argparse
import random
import re
import sys
import time
from pathlib import Path
from typing import Any

import yaml

DEFAULT_FILE = Path(__file__).resolve().parent.parent / "tests" / "fixtures" / "diag_agg_sample.yaml"


def level_from_yaml(v: Any) -> int:
    """YAML 上の level("\\x03" のような 1 文字の文字列、または整数)を int にする。"""
    if isinstance(v, int):
        return v
    if isinstance(v, str) and len(v) == 1:
        return ord(v)
    raise ValueError(f"level を解釈できない: {v!r}")


def load_fixture(path: str | Path) -> list[dict[str, Any]]:
    """フィクスチャを読み、DiagnosticArray 相当の dict のリストを返す(level は int)。"""
    with open(path, encoding="utf-8") as f:
        docs = [d for d in yaml.safe_load_all(f) if isinstance(d, dict)]
    if not docs:
        raise ValueError(f"{path}: DiagnosticArray が見つからない")
    out = []
    for d in docs:
        status = []
        for s in d.get("status") or []:
            status.append({
                "name": str(s.get("name", "")),
                "level": level_from_yaml(s.get("level", 0)),
                "message": str(s.get("message", "")),
                "hardware_id": str(s.get("hardware_id", "")),
                "values": [{"key": str(kv["key"]), "value": str(kv["value"])} for kv in (s.get("values") or [])],
            })
        out.append({"status": status})
    return out


_NUM_RE = re.compile(r"^([-+]?\d+(?:\.\d+)?)(.*)$")


def _jitter_value(key: str, value: str) -> str:
    m = _NUM_RE.match(value.strip())
    if not m:
        return value
    num, rest = float(m.group(1)), m.group(2)
    decimals = len(m.group(1).split(".")[1]) if "." in m.group(1) else 0
    if key == "Usage":
        num = min(100.0, max(0.0, num + random.uniform(-3, 3)))
    elif key == "Actual frequency (Hz)":
        num = max(0.0, num * random.uniform(0.95, 1.05)) if num > 0 else 0.0
    else:
        return value
    return f"{num:.{decimals}f}{rest}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", nargs="?", default=str(DEFAULT_FILE), help="フィクスチャ YAML")
    ap.add_argument("--topic", default="/diagnostics_agg")
    ap.add_argument("--hz", type=float, default=1.0)
    ap.add_argument("--jitter", action="store_true", help="Usage や周波数を少し揺らす")
    ap.add_argument("--stop-after", type=float, default=None, metavar="N", help="N 秒で publish を止める(プロセスは残る)")
    args = ap.parse_args()

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    arrays = load_fixture(args.file)
    rclpy.init()
    node = rclpy.create_node("fake_diag_pub")
    pub = node.create_publisher(DiagnosticArray, args.topic, 10)
    t0 = time.monotonic()
    i = 0
    stopped = False

    def tick() -> None:
        nonlocal i, stopped
        if args.stop_after is not None and time.monotonic() - t0 >= args.stop_after:
            if not stopped:
                node.get_logger().warn(f"--stop-after {args.stop_after}: publish を停止しました")
                stopped = True
            return
        src = arrays[i % len(arrays)]
        i += 1
        msg = DiagnosticArray()
        msg.header.stamp = node.get_clock().now().to_msg()
        for s in src["status"]:
            st = DiagnosticStatus()
            st.level = bytes([s["level"]])
            st.name = s["name"]
            st.message = s["message"]
            st.hardware_id = s["hardware_id"]
            st.values = [
                KeyValue(key=kv["key"], value=_jitter_value(kv["key"], kv["value"]) if args.jitter else kv["value"])
                for kv in s["values"]
            ]
            msg.status.append(st)
        pub.publish(msg)

    node.create_timer(1.0 / args.hz, tick)
    node.get_logger().info(f"publishing {args.file} to {args.topic} at {args.hz} Hz")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
