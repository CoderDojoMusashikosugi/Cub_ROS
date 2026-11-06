import sys
from pathlib import Path

# tests から app/ と tools/ を import できるように
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))


def as_client(app, ip: str):
    """接続元 IP を差し替えた ASGI アプリを返す。

    TestClient の接続元は "testclient"(IP ではない)で、差し替える client= 引数は
    Ubuntu 24.04 の apt 版 starlette(0.31)には無いため。
    """
    async def wrapped(scope, receive, send):
        if scope["type"] in ("http", "websocket"):
            scope = {**scope, "client": (ip, 50000)}
        await app(scope, receive, send)

    return wrapped
