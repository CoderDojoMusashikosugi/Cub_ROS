"""アプリ(REST / WebSocket / 接続元制限)とフィクスチャの検証。"""
from __future__ import annotations

import json
import os
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from app.main import create_app


def test_health_and_static() -> None:
    app = create_app(enable_ros=False)
    with TestClient(app, client=("127.0.0.1", 50000)) as client:
        r = client.get("/api/health")
        assert r.status_code == 200
        assert r.json() == {"ok": True, "ros": False, "last_diag_age_sec": None}
        assert client.get("/").status_code == 200  # 静的配信
        assert client.get("/api/processes").status_code == 404  # プロセス管理は外している


def test_websocket_sends_latest_and_heartbeat() -> None:
    app = create_app(enable_ros=False)
    with TestClient(app, client=("127.0.0.1", 50000)) as client:
        app.state.hub.publish_diagnostics('{"type":"diagnostics","stamp":1.0,"status":[]}')
        with client.websocket_connect("/ws") as ws:
            assert json.loads(ws.receive_text())["type"] == "diagnostics"
            hb = json.loads(ws.receive_text())
            assert hb["type"] == "heartbeat" and hb["diag_age_sec"] is not None


def test_access_control() -> None:
    from app.access import client_allowed, origin_ok, parse_allow

    allow = parse_allow("192.168.63.0/24")
    assert client_allowed("127.0.0.1", ())
    assert client_allowed("::1", ())
    assert client_allowed("192.168.63.50", allow)
    assert client_allowed("::ffff:192.168.63.50", allow)
    assert not client_allowed("192.168.64.1", allow)
    assert not client_allowed("10.0.0.1", ())
    assert not client_allowed(None, allow)
    assert origin_ok({"host": "192.168.63.204:8080"})
    assert origin_ok({"host": "192.168.63.204:8080", "origin": "http://192.168.63.204:8080"})
    assert not origin_ok({"host": "192.168.63.204:8080", "origin": "http://evil.example"})
    with pytest.raises(ValueError):
        parse_allow("192.168.63.0/33")

    # TestClient の接続元は "testclient"(IP ではない)なので、client を差し替えて試す
    for client_ip, ok in (("192.168.63.10", True), ("192.168.1.10", False)):
        app = create_app(enable_ros=False, allow=allow)
        with TestClient(app, client=(client_ip, 50000)) as c:
            assert (c.get("/api/health").status_code == 200) is ok
    app = create_app(enable_ros=False, allow=allow)
    with TestClient(app, client=("192.168.63.10", 50000), base_url="http://192.168.63.204:8080") as c:
        # 他サイトからの WebSocket は拒否、同じオリジンからは通る
        with pytest.raises(WebSocketDisconnect):
            with c.websocket_connect("/ws", headers={"origin": "http://evil.example"}) as ws:
                ws.receive_text()
        # TestClient の WebSocket は base_url に関係なく Host: testserver を送る
        with c.websocket_connect("/ws", headers={"origin": "http://testserver"}) as ws:
            assert json.loads(ws.receive_text())["type"] == "heartbeat"
        # GET 以外は Origin を確認する
        assert c.post("/api/health", headers={"origin": "http://evil.example"}).status_code == 403


def test_fixture_json_in_sync() -> None:
    """node のテストが読む JSON がフィクスチャ YAML と一致していること。"""
    from fixture_to_json import to_json

    fx = Path(__file__).resolve().parent / "fixtures"
    for name in ("diag_agg_sample", "diag_agg_local_example"):
        assert (fx / f"{name}.json").read_text() == to_json(str(fx / f"{name}.yaml")), (
            f"python3 tools/fixture_to_json.py tests/fixtures/{name}.yaml > tests/fixtures/{name}.json で再生成すること"
        )


@pytest.mark.skipif(os.environ.get("ROS_DISTRO") is None, reason="ROS 環境なし")
def test_diag_array_to_json() -> None:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    from app.ros_bridge import diag_array_to_json

    msg = DiagnosticArray()
    msg.header.stamp.sec = 10
    msg.header.stamp.nanosec = 500_000_000
    msg.status.append(DiagnosticStatus(level=bytes([3]), name="/R/a: b", message="m", hardware_id="h",
                                       values=[KeyValue(key="k", value="1.0%")]))
    d = json.loads(diag_array_to_json(msg))
    assert d == {"type": "diagnostics", "stamp": 10.5, "status": [
        {"name": "/R/a: b", "level": 3, "message": "m", "hardware_id": "h", "values": [{"key": "k", "value": "1.0%"}]}]}


def test_allow_auto() -> None:
    import ipaddress

    from app.access import local_subnets, parse_allow

    subnets = local_subnets()
    assert all(isinstance(n, ipaddress.IPv4Network) and not n.is_loopback for n in subnets)
    assert parse_allow("auto") == subnets
    assert parse_allow("auto 10.1.0.0/16")[-1] == ipaddress.ip_network("10.1.0.0/16")
