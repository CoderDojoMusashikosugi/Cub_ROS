"""rclpy で /diagnostics_agg を購読し、asyncio 側の Hub へ渡す(購読のみ)。

rclpy が import できない環境でも本アプリは起動できる(診断購読だけ無効)。
"""
from __future__ import annotations

import asyncio
import json
import logging
import threading
import time
from typing import Any

log = logging.getLogger(__name__)

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from diagnostic_msgs.msg import DiagnosticArray

    HAVE_ROS = True
except ImportError as e:  # ROS を source していない環境
    HAVE_ROS = False
    _IMPORT_ERROR = e

NODE_NAME = "cub_console"
DIAG_TOPIC = "/diagnostics_agg"


def level_to_int(v: Any) -> int:
    # rclpy では byte 型フィールドは bytes で届く
    return v if isinstance(v, int) else v[0]


def diag_array_to_json(msg: Any) -> str:
    """DiagnosticArray を WebSocket 配信用の JSON 文字列にする(加工は level の int 化のみ)。"""
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    status = [
        {
            "name": s.name,
            "level": level_to_int(s.level),
            "message": s.message,
            "hardware_id": s.hardware_id,
            "values": [{"key": kv.key, "value": kv.value} for kv in s.values],
        }
        for s in msg.status
    ]
    return json.dumps(
        {"type": "diagnostics", "stamp": stamp, "status": status},
        ensure_ascii=False,
        separators=(",", ":"),
    )


class Hub:
    """最新の診断を保持し、接続中の WebSocket ごとのキューへ配る。asyncio スレッド専用。"""

    def __init__(self) -> None:
        self._latest: str | None = None
        self._last_rx: float | None = None
        self._queues: set[asyncio.Queue[str]] = set()

    @property
    def latest(self) -> str | None:
        return self._latest

    def diag_age_sec(self) -> float | None:
        if self._last_rx is None:
            return None
        return time.monotonic() - self._last_rx

    def subscribe(self) -> asyncio.Queue[str]:
        q: asyncio.Queue[str] = asyncio.Queue(maxsize=2)
        if self._latest is not None:
            q.put_nowait(self._latest)
        self._queues.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue[str]) -> None:
        self._queues.discard(q)

    def publish_diagnostics(self, text: str) -> None:
        self._latest = text
        self._last_rx = time.monotonic()
        self.broadcast(text)

    def broadcast(self, text: str) -> None:
        for q in self._queues:
            if q.full():
                # 遅いクライアントに引きずられないよう、古い値を捨てる
                q.get_nowait()
            q.put_nowait(text)


class RosBridge:
    """rclpy を別スレッドの SingleThreadedExecutor で回す。"""

    def __init__(self, hub: Hub, loop: asyncio.AbstractEventLoop, topic: str = DIAG_TOPIC) -> None:
        self._hub = hub
        self._loop = loop
        self._topic = topic
        self._thread: threading.Thread | None = None
        self._executor: Any = None
        self._node: Any = None
        self._context: Any = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> bool:
        if not HAVE_ROS:
            log.warning("rclpy を import できないため診断の購読を無効にします: %s", _IMPORT_ERROR)
            return False
        # シグナル処理は uvicorn に任せる(rclpy に SIGINT を奪わせない)
        self._context = rclpy.Context()
        rclpy.init(context=self._context, signal_handler_options=SignalHandlerOptions.NO)
        self._node = rclpy.create_node(NODE_NAME, context=self._context)
        self._node.create_subscription(DiagnosticArray, self._topic, self._on_diag, 10)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, name="ros_bridge", daemon=True)
        self._thread.start()
        log.info("subscribed %s as node %s", self._topic, NODE_NAME)
        return True

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except ExternalShutdownException:
            pass

    def _on_diag(self, msg: Any) -> None:
        # JSON 化はここで 1 回だけ。全クライアントに同じ文字列を配る
        text = diag_array_to_json(msg)
        self._loop.call_soon_threadsafe(self._hub.publish_diagnostics, text)

    def stop(self) -> None:
        if self._executor is None:
            return
        self._executor.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=5)
        self._node.destroy_node()
        self._context.try_shutdown()
        self._executor = None
