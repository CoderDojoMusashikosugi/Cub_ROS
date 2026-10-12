"""cub_console 本体: REST + WebSocket(状態配信)+ 静的ファイル配信を 1 プロセスで。

プロセス管理(app/processes.py)は枠組みを検討中のため、いったんアプリから外している。

起動: python -m app.main
"""
from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import os
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from .access import AccessControl, Network, parse_allow
from .ros_bridge import HAVE_ROS, Hub, RosBridge

log = logging.getLogger("cub_console")

BASE_DIR = Path(__file__).resolve().parent.parent
STATIC_DIR = BASE_DIR / "static"
HEARTBEAT_SEC = 1.0


async def _heartbeat(hub: Hub) -> None:
    while True:
        await asyncio.sleep(HEARTBEAT_SEC)
        age = hub.diag_age_sec()
        hub.broadcast(json.dumps({"type": "heartbeat", "diag_age_sec": None if age is None else round(age, 2)}))


def _allow_from_env() -> tuple[Network, ...]:
    return parse_allow(os.environ.get("CUB_CONSOLE_ALLOW", ""))


def create_app(*, enable_ros: bool = True, allow: tuple[Network, ...] | None = None) -> FastAPI:
    """allow は localhost 以外に接続を許す CIDR(既定は環境変数 CUB_CONSOLE_ALLOW)。
    """
    if allow is None:
        allow = _allow_from_env()

    @asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncIterator[None]:
        hub = Hub()
        app.state.hub = hub
        bridge = RosBridge(hub, asyncio.get_running_loop())
        app.state.ros_ok = bridge.start() if enable_ros else False
        hb = asyncio.create_task(_heartbeat(hub))
        try:
            yield
        finally:
            hb.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await hb
            bridge.stop()

    app = FastAPI(title="cub_console", lifespan=lifespan, docs_url=None, redoc_url=None, openapi_url=None)

    @app.get("/api/health")
    async def health() -> dict:
        age = app.state.hub.diag_age_sec()
        return {"ok": True, "ros": bool(app.state.ros_ok), "last_diag_age_sec": age}

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        hub: Hub = app.state.hub
        q = hub.subscribe()  # 最新の診断があれば最初に入っている

        async def sender() -> None:
            while True:
                await ws.send_text(await q.get())

        async def receiver() -> None:
            # クライアントからは何も来ない前提。切断(サーバ終了時の close を含む)を検知するためだけに読む
            while True:
                if (await ws.receive())["type"] == "websocket.disconnect":
                    return

        tasks = [asyncio.create_task(sender()), asyncio.create_task(receiver())]
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for t in done:
                exc = t.exception()
                if exc is not None and not isinstance(exc, WebSocketDisconnect):
                    raise exc
        finally:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            hub.unsubscribe(q)

    app.mount("/", StaticFiles(directory=STATIC_DIR, html=True), name="static")
    app.add_middleware(AccessControl, allow=allow)
    return app


app = create_app()


def main() -> None:
    import uvicorn

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    if not HAVE_ROS:
        log.warning("ROS 環境が見つかりません(source 済みか確認)。診断なしで起動します")
    host = os.environ.get("CUB_CONSOLE_HOST", "127.0.0.1")
    port = int(os.environ.get("CUB_CONSOLE_PORT", "8080"))
    allow = _allow_from_env()
    if host not in ("127.0.0.1", "::1", "localhost") and not allow:
        # 認証が無いので、LAN に出すときは接続元の許可リストを必須にする
        raise SystemExit(
            f"CUB_CONSOLE_HOST={host} で待ち受けるには CUB_CONSOLE_ALLOW(例: 192.168.63.0/24 または auto)が必要です"
            "(auto の場合は LAN の IPv4 アドレスが見つからなかった)"
        )
    if allow:
        log.warning("localhost 以外からの接続を許可: %s(認証なし。信頼できるネットワークでのみ使うこと)",
                    ", ".join(map(str, allow)))
    # proxy_headers=False: X-Forwarded-For で接続元を偽装させない
    uvicorn.run(app, host=host, port=port, log_level="info", access_log=False, proxy_headers=False)


if __name__ == "__main__":
    main()
