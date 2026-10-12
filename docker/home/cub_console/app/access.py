"""接続元の制限(ASGI ミドルウェア)。認証の代わりではなく、LAN に出すときの最低限の囲い。

- 接続元 IP: localhost と、許可した CIDR(CUB_CONSOLE_ALLOW)だけ通す。
- Origin: ブラウザから来た POST / WebSocket は、Origin のホストが Host と一致するものだけ通す
  (他のサイトを開いたブラウザ経由で操作 API を叩かれるのを防ぐ)。
"""
from __future__ import annotations

import ipaddress
import json
import logging
from typing import Any, Awaitable, Callable
from urllib.parse import urlsplit

log = logging.getLogger(__name__)

Network = ipaddress.IPv4Network | ipaddress.IPv6Network
LOOPBACK: tuple[Network, ...] = (ipaddress.ip_network("127.0.0.0/8"), ipaddress.ip_network("::1/128"))


# auto で対象にしないインタフェース(Docker / 仮想ブリッジ)
_SKIP_IF_PREFIXES = ("lo", "docker", "br-", "veth", "virbr", "cni", "flannel")


def local_subnets() -> tuple[ipaddress.IPv4Network, ...]:
    """この機体の LAN インタフェースの IPv4 サブネット(Linux のみ。ip コマンド不要)。"""
    import fcntl
    import socket
    import struct

    SIOCGIFADDR, SIOCGIFNETMASK = 0x8915, 0x891B
    nets = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        for _, name in socket.if_nameindex():
            if name.startswith(_SKIP_IF_PREFIXES):
                continue
            req = struct.pack("256s", name.encode()[:15])
            try:
                addr = socket.inet_ntoa(fcntl.ioctl(s.fileno(), SIOCGIFADDR, req)[20:24])
                mask = socket.inet_ntoa(fcntl.ioctl(s.fileno(), SIOCGIFNETMASK, req)[20:24])
            except OSError:
                continue  # IPv4 アドレスが無いインタフェース
            nets.append(ipaddress.IPv4Network(f"{addr}/{mask}", strict=False))
    return tuple(dict.fromkeys(nets))


def parse_allow(spec: str) -> tuple[Network, ...]:
    """'192.168.63.0/24, 10.0.0.5' → ネットワークのタプル。不正な値は ValueError。
    'auto' は起動時点の LAN インタフェースのサブネットに展開する。
    """
    nets: list[Network] = []
    for item in spec.replace(",", " ").split():
        if item == "auto":
            nets.extend(local_subnets())
        else:
            nets.append(ipaddress.ip_network(item, strict=False))
    return tuple(nets)


def client_allowed(host: str | None, allow: tuple[Network, ...]) -> bool:
    if not host:
        return False
    try:
        addr = ipaddress.ip_address(host)
    except ValueError:
        return False
    if isinstance(addr, ipaddress.IPv6Address) and addr.ipv4_mapped is not None:
        addr = addr.ipv4_mapped
    return any(addr in net for net in (*LOOPBACK, *allow))


def origin_ok(headers: dict[str, str]) -> bool:
    """Origin が無い(curl 等)か、Origin のホスト:ポートが Host と一致すれば OK。"""
    origin = headers.get("origin")
    if origin is None:
        return True
    host = headers.get("host", "")
    try:
        return urlsplit(origin).netloc.lower() == host.lower()
    except ValueError:
        return False


ASGIApp = Callable[[dict[str, Any], Callable[[], Awaitable[dict]], Callable[[dict], Awaitable[None]]], Awaitable[None]]


class AccessControl:
    def __init__(self, app: ASGIApp, allow: tuple[Network, ...]) -> None:
        self.app = app
        self.allow = allow

    async def __call__(self, scope: dict[str, Any], receive: Any, send: Any) -> None:
        if scope["type"] not in ("http", "websocket"):
            await self.app(scope, receive, send)
            return
        client = scope.get("client")
        host = client[0] if client else None
        headers = {k.decode("latin-1").lower(): v.decode("latin-1") for k, v in scope.get("headers", [])}
        if not client_allowed(host, self.allow):
            log.warning("拒否: 許可していない接続元 %s %s", host, scope.get("path"))
            await self._deny(scope, send, "client not allowed")
            return
        needs_origin = scope["type"] == "websocket" or scope.get("method") not in ("GET", "HEAD", "OPTIONS")
        if needs_origin and not origin_ok(headers):
            log.warning("拒否: Origin 不一致 %s (from %s) %s", headers.get("origin"), host, scope.get("path"))
            await self._deny(scope, send, "origin not allowed")
            return
        await self.app(scope, receive, send)

    @staticmethod
    async def _deny(scope: dict[str, Any], send: Any, reason: str) -> None:
        if scope["type"] == "websocket":
            await send({"type": "websocket.close", "code": 1008})
            return
        body = json.dumps({"detail": reason}).encode()
        await send({"type": "http.response.start", "status": 403,
                    "headers": [(b"content-type", b"application/json"), (b"content-length", str(len(body)).encode())]})
        await send({"type": "http.response.body", "body": body})
