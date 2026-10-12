"""管理対象プロセスの起動/停止。

ROS 通信には一切依存しない(ROS が落ちていても操作できること)。
操作できるのは設定ファイルに書かれた id だけ(任意コマンド実行 API は作らない)。
"""
from __future__ import annotations

import asyncio
import logging
import os
import re
import signal
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import yaml

log = logging.getLogger(__name__)

_ID_RE = re.compile(r"^[A-Za-z0-9_\-]+$")


class UnknownProcess(KeyError):
    """設定に無い id が指定された。"""


class ConfigError(ValueError):
    """managed_processes.yaml の内容が不正。"""


@dataclass(frozen=True)
class ProcessSpec:
    id: str
    label: str
    cmd: tuple[str, ...]
    cwd: str | None = None


@dataclass(frozen=True)
class ProcessConfig:
    backend: str
    processes: tuple[ProcessSpec, ...]


def load_config(path: str | Path) -> ProcessConfig:
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        raise ConfigError(f"{path}: トップレベルは mapping であること")
    backend = raw.get("backend", "subprocess")
    items = raw.get("processes") or []
    if not isinstance(items, list):
        raise ConfigError(f"{path}: processes は配列であること")
    specs: list[ProcessSpec] = []
    seen: set[str] = set()
    for i, item in enumerate(items):
        if not isinstance(item, dict):
            raise ConfigError(f"{path}: processes[{i}] は mapping であること")
        pid = item.get("id")
        if not isinstance(pid, str) or not _ID_RE.match(pid):
            raise ConfigError(f"{path}: processes[{i}].id が不正: {pid!r}")
        if pid in seen:
            raise ConfigError(f"{path}: id が重複している: {pid}")
        seen.add(pid)
        cmd = item.get("cmd")
        # 文字列は受け付けない(シェル解釈の余地を作らない)
        if not isinstance(cmd, list) or not cmd or not all(isinstance(c, str) for c in cmd):
            raise ConfigError(f"{path}: {pid}.cmd は文字列の配列であること")
        cwd = item.get("cwd")
        if cwd is not None and not isinstance(cwd, str):
            raise ConfigError(f"{path}: {pid}.cwd は文字列であること")
        label = item.get("label", pid)
        specs.append(ProcessSpec(id=pid, label=str(label), cmd=tuple(cmd), cwd=cwd))
    return ProcessConfig(backend=str(backend), processes=tuple(specs))


class ProcessBackend(Protocol):
    async def list(self) -> list[dict[str, str]]: ...
    async def start(self, id: str) -> None: ...
    async def stop(self, id: str) -> None: ...
    async def restart(self, id: str) -> None: ...
    async def shutdown(self) -> None: ...


def _group_alive(pgid: int) -> bool:
    """プロセスグループにゾンビ以外のメンバーが残っているか。

    killpg(pgid, 0) はゾンビにも成功してしまう(コンテナの PID 1 が回収しない場合など)ので、
    Linux では /proc を見て判定する。
    """
    proc = Path("/proc")
    if proc.is_dir():
        for d in proc.iterdir():
            if not d.name.isdigit():
                continue
            try:
                stat = (d / "stat").read_text()
            except OSError:
                continue
            # "pid (comm) state ppid pgrp ..." の comm は空白や括弧を含みうるので最後の ')' で切る
            fields = stat[stat.rfind(")") + 2:].split()
            if len(fields) >= 3 and int(fields[2]) == pgid and fields[0] != "Z":
                return True
        return False
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


class SubprocessBackend:
    """開発用: asyncio の subprocess で直接起動する。本アプリ終了時に道連れで止める。"""

    def __init__(
        self,
        specs: tuple[ProcessSpec, ...] | list[ProcessSpec],
        *,
        base_dir: str | Path | None = None,
        stop_timeouts: tuple[float, float, float] = (8.0, 3.0, 3.0),
    ) -> None:
        # cwd 未指定・相対指定は base_dir(本アプリのディレクトリ)基準
        self._base_dir = Path(base_dir) if base_dir is not None else None
        self._specs = {s.id: s for s in specs}
        self._procs: dict[str, asyncio.subprocess.Process] = {}
        self._stopped_by_user: dict[str, bool] = {}
        self._locks = {s.id: asyncio.Lock() for s in specs}
        self._timeouts = stop_timeouts

    def _spec(self, id: str) -> ProcessSpec:
        try:
            return self._specs[id]
        except KeyError:
            raise UnknownProcess(id) from None

    def _state(self, id: str) -> str:
        proc = self._procs.get(id)
        if proc is None:
            return "inactive"
        rc = proc.returncode
        if rc is None:
            return "active"
        if self._stopped_by_user.get(id) or rc == 0:
            return "inactive"
        return f"failed(rc={rc})"

    def pid(self, id: str) -> int | None:
        """テスト・デバッグ用。動作中なら PID(= PGID)。"""
        self._spec(id)
        proc = self._procs.get(id)
        return proc.pid if proc is not None and proc.returncode is None else None

    async def list(self) -> list[dict[str, str]]:
        return [{"id": s.id, "label": s.label, "state": self._state(s.id)} for s in self._specs.values()]

    async def start(self, id: str) -> None:
        spec = self._spec(id)
        async with self._locks[id]:
            await self._start_locked(spec)

    async def stop(self, id: str) -> None:
        self._spec(id)
        async with self._locks[id]:
            await self._stop_locked(id)

    async def restart(self, id: str) -> None:
        spec = self._spec(id)
        async with self._locks[id]:
            await self._stop_locked(id)
            await self._start_locked(spec)

    async def shutdown(self) -> None:
        await asyncio.gather(*(self.stop(i) for i in self._specs), return_exceptions=False)

    async def _start_locked(self, spec: ProcessSpec) -> None:
        if self._state(spec.id) == "active":
            return
        # start_new_session=True で新しいプロセスグループにし、ros2 launch の子ごと止められるようにする
        cwd: Path | str | None = spec.cwd
        if self._base_dir is not None:
            cwd = self._base_dir / (spec.cwd or ".")
        proc = await asyncio.create_subprocess_exec(
            *spec.cmd,
            cwd=cwd,
            stdin=asyncio.subprocess.DEVNULL,
            start_new_session=True,
        )
        self._procs[spec.id] = proc
        self._stopped_by_user[spec.id] = False
        log.info("started %s pid=%d cmd=%s", spec.id, proc.pid, list(spec.cmd))

    async def _stop_locked(self, id: str) -> None:
        proc = self._procs.get(id)
        if proc is None:
            return
        pgid = proc.pid  # start_new_session なので PGID == PID
        if proc.returncode is not None and not _group_alive(pgid):
            return  # 既に止まっている(異常終了の表示は残す)
        self._stopped_by_user[id] = True
        # ROS 2 は SIGINT で正常終了するので、まず SIGINT
        for sig, timeout in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), self._timeouts):
            if not _group_alive(pgid):
                break
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            log.info("stop %s: sent %s to pgid=%d", id, sig.name, pgid)
            if await self._wait_group(proc, pgid, timeout):
                break
        else:
            if _group_alive(pgid):
                log.error("stop %s: pgid=%d が SIGKILL 後も残っている", id, pgid)
        if proc.returncode is None:
            await proc.wait()
        log.info("stopped %s rc=%s", id, proc.returncode)

    @staticmethod
    async def _wait_group(proc: asyncio.subprocess.Process, pgid: int, timeout: float) -> bool:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while True:
            # リーダーの回収は asyncio が行う。グループ全体が消えるまで待つ
            if proc.returncode is not None and not _group_alive(pgid):
                return True
            if loop.time() >= deadline:
                return False
            await asyncio.sleep(0.05)


def make_backend(config: ProcessConfig, base_dir: str | Path | None = None) -> ProcessBackend:
    if config.backend == "subprocess":
        return SubprocessBackend(config.processes, base_dir=base_dir)
    # SystemdBackend は今回は実装しない
    raise ConfigError(f"未対応の backend: {config.backend}")
