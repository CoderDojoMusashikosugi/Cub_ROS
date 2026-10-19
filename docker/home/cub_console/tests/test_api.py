"""REST / WebSocket の検証(ROS は無効化して実行)。"""
from __future__ import annotations

import asyncio
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from app.main import create_app
from app.ros_bridge import Hub, diag_array_to_json, level_to_int

CMD_HEADERS = {"X-Requested-With": "cub_console"}


@pytest.fixture()
def client(tmp_path: Path):
    cfg = tmp_path / "procs.yaml"
    cfg.write_text(
        "backend: subprocess\n"
        "processes:\n"
        "  - id: sleeper\n"
        "    label: sleep\n"
        '    cmd: ["sleep", "60"]\n',
        encoding="utf-8",
    )
    app = create_app(config_path=cfg, enable_ros=False)
    with TestClient(app, base_url="http://127.0.0.1:8080") as c:
        yield c


def test_health(client: TestClient) -> None:
    r = client.get("/api/health")
    assert r.status_code == 200
    assert r.json() == {"ok": True, "ros": False, "last_diag_age_sec": None}


def test_process_lifecycle(client: TestClient) -> None:
    assert client.get("/api/processes").json() == [
        {"id": "sleeper", "label": "sleep", "state": "inactive"}
    ]
    assert client.post("/api/processes/sleeper/start", headers=CMD_HEADERS).json() == {"ok": True}
    assert client.get("/api/processes").json()[0]["state"] == "active"
    assert client.post("/api/processes/sleeper/restart", headers=CMD_HEADERS).status_code == 200
    assert client.post("/api/processes/sleeper/stop", headers=CMD_HEADERS).status_code == 200
    assert client.get("/api/processes").json()[0]["state"] == "inactive"


def test_unknown_id_and_action_are_404(client: TestClient) -> None:
    assert client.post("/api/processes/nope/start", headers=CMD_HEADERS).status_code == 404
    assert client.post("/api/processes/sleeper/kill", headers=CMD_HEADERS).status_code == 404


def test_command_requires_header_and_same_origin(client: TestClient) -> None:
    assert client.post("/api/processes/sleeper/start").status_code == 403
    r = client.post(
        "/api/processes/sleeper/start",
        headers={**CMD_HEADERS, "Origin": "http://evil.example"},
    )
    assert r.status_code == 403


def test_any_host_allowed_by_default(client: TestClient) -> None:
    r = client.get("/api/health", headers={"Host": "192.168.1.50:8080"})
    assert r.status_code == 200


def test_allowed_hosts_env_restricts(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("CUB_CONSOLE_ALLOWED_HOSTS", "127.0.0.1, robot.local")
    cfg = tmp_path / "procs.yaml"
    cfg.write_text("processes: []\n", encoding="utf-8")
    app = create_app(config_path=cfg, enable_ros=False)
    with TestClient(app, base_url="http://127.0.0.1:8080") as c:
        assert c.get("/api/health").status_code == 200
        assert c.get("/api/health", headers={"Host": "robot.local:8080"}).status_code == 200
        assert c.get("/api/health", headers={"Host": "evil.example:8080"}).status_code == 400


def test_ws_heartbeat(client: TestClient) -> None:
    with client.websocket_connect("/ws") as ws:
        msg = ws.receive_json()
        assert msg == {"type": "heartbeat", "diag_age_sec": None}


def test_static_index(client: TestClient) -> None:
    r = client.get("/")
    assert r.status_code == 200
    assert "cub_console" in r.text


# ---------------------------------------------------------------- ros_bridge(ROS 型に依存しない部分)


def test_level_to_int() -> None:
    assert level_to_int(b"\x00") == 0
    assert level_to_int(b"\x03") == 3
    assert level_to_int(2) == 2
    assert level_to_int(b"") == -1


def test_diag_array_to_json() -> None:
    import json

    kv = SimpleNamespace(key="Usage", value="47.8%")
    st = SimpleNamespace(name="/Robot/Linux", level=b"\x02", message="Error", hardware_id="", values=[kv])
    msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=500_000_000)), status=[st])
    out = json.loads(diag_array_to_json(msg))
    assert out["type"] == "diagnostics"
    assert out["stamp"] == pytest.approx(10.5)
    assert out["status"] == [
        {"name": "/Robot/Linux", "level": 2, "message": "Error", "hardware_id": "",
         "values": [{"key": "Usage", "value": "47.8%"}]}
    ]


def test_hub_drops_old_values() -> None:
    async def main() -> None:
        hub = Hub()
        hub.publish("a")
        q = hub.subscribe()
        assert q.qsize() == 1  # 接続直後に最新値
        hub.publish("b")
        hub.publish("c")
        assert [q.get_nowait(), q.get_nowait()] == ["b", "c"]
        assert hub.diag_age_sec() is not None
        hub.unsubscribe(q)
        assert hub.subscriber_count == 0

    asyncio.run(main())
