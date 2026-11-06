"""SubprocessBackend の起動/停止を検証する(ROS 不要)。

プロセス管理はいったんアプリから外しているが、バックエンド単体は残してテストしている。
"""
from __future__ import annotations

import asyncio
import textwrap
from pathlib import Path

import pytest

from app.processes import ConfigError, ProcessSpec, SubprocessBackend, UnknownProcess, _group_alive, load_config

SPECS = [
    ProcessSpec(id="sleeper", label="sleep", cmd=("sleep", "60")),
    ProcessSpec(id="family", label="子持ち", cmd=("sh", "-c", "sleep 60 & sleep 60 & wait")),
    # SIGINT と SIGTERM を無視する子 → SIGKILL まで段階的に進むこと
    ProcessSpec(id="stubborn", label="頑固", cmd=("sh", "-c", "trap '' INT TERM; sleep 60 & wait; sleep 60")),
    ProcessSpec(id="crash", label="異常終了", cmd=("sh", "-c", "exit 3")),
]


def run(coro):
    return asyncio.run(coro)


def make() -> SubprocessBackend:
    return SubprocessBackend(SPECS, stop_timeouts=(1.0, 1.0, 1.0))


async def state(be: SubprocessBackend, id: str) -> str:
    return {p["id"]: p["state"] for p in await be.list()}[id]


def test_start_stop_restart() -> None:
    async def main() -> None:
        be = make()
        assert await state(be, "sleeper") == "inactive"
        await be.start("sleeper")
        assert await state(be, "sleeper") == "active"
        pid1 = be.pid("sleeper")
        assert pid1 is not None
        await be.start("sleeper")  # 動作中の start は何もしない
        assert be.pid("sleeper") == pid1
        await be.restart("sleeper")
        pid2 = be.pid("sleeper")
        assert pid2 is not None and pid2 != pid1
        assert not _group_alive(pid1)
        await be.stop("sleeper")
        assert await state(be, "sleeper") == "inactive"
        assert not _group_alive(pid2)
        await be.stop("sleeper")  # 停止済みの stop も成功する

    run(main())


def test_stop_kills_children() -> None:
    async def main() -> None:
        be = make()
        await be.start("family")
        pgid = be.pid("family")
        assert pgid is not None
        await asyncio.sleep(0.3)  # sh が子を作るのを待つ
        assert _group_alive(pgid)
        await be.stop("family")
        assert await state(be, "family") == "inactive"
        assert not _group_alive(pgid), "stop 後に子プロセスが残っている"

    run(main())


def test_stop_escalates_to_sigkill() -> None:
    async def main() -> None:
        be = make()
        await be.start("stubborn")
        pgid = be.pid("stubborn")
        await asyncio.sleep(0.3)
        await be.stop("stubborn")
        assert not _group_alive(pgid)
        assert await state(be, "stubborn") == "inactive"

    run(main())


def test_failed_state() -> None:
    async def main() -> None:
        be = make()
        await be.start("crash")
        for _ in range(100):
            if (await state(be, "crash")) != "active":
                break
            await asyncio.sleep(0.02)
        assert await state(be, "crash") == "failed(rc=3)"

    run(main())


def test_unknown_process() -> None:
    async def main() -> None:
        be = make()
        for op in (be.start, be.stop, be.restart):
            with pytest.raises(UnknownProcess):
                await op("nope")

    run(main())


def test_shutdown_stops_all() -> None:
    async def main() -> None:
        be = make()
        await be.start("sleeper")
        await be.start("family")
        pgids = [be.pid("sleeper"), be.pid("family")]
        await be.shutdown()
        assert all(not _group_alive(p) for p in pgids)

    run(main())


def test_load_config_rejects_string_cmd(tmp_path: Path) -> None:
    p = tmp_path / "c.yaml"
    p.write_text(textwrap.dedent("""
        processes:
          - id: bad
            cmd: "ros2 run foo bar"
    """))
    with pytest.raises(ConfigError):
        load_config(p)


def test_load_default_config() -> None:
    cfg = load_config(Path(__file__).resolve().parent.parent / "config" / "managed_processes.yaml")
    assert cfg.backend == "subprocess"
    assert "demo_talker" in [s.id for s in cfg.processes]
