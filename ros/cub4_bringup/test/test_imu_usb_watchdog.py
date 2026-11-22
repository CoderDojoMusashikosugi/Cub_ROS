"""imu_usb_watchdog.py のテスト。

USBへの要求・sysfs・usbreset・時刻は差し替えるので、実機のIMUが無くても動く。
"""

import importlib.util
import os
import struct
from types import SimpleNamespace

import pytest
import rclpy

SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'imu_usb_watchdog.py')
_spec = importlib.util.spec_from_file_location('imu_usb_watchdog', SCRIPT)
wd = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(wd)


# ---------------------------------------------------------------- choose_action

@pytest.mark.parametrize('args, expected', [
    # (imu_present, imu_ok, hub_ok, siblings, port_connected, last_action, hub_reset_enabled)
    ((True, True, True, [], None, None, True), 'imu'),          # USBは応答、データだけ止まった
    ((True, None, True, [], None, None, True), 'imu'),          # 判定不能でもIMUから
    ((True, False, True, [True], None, None, True), 'imu'),     # IMU単体の無応答
    ((True, False, True, [], None, None, True), 'imu'),         # 兄弟なし
    ((True, False, True, [False, False], None, None, True), 'hub'),   # TTの詰まり
    ((True, False, True, [False], None, None, False), 'imu'),   # TTの詰まりだがハブ不可
    ((True, True, True, [], None, 'imu', True), 'hub'),         # IMUのリセットで直らなかった
    ((True, True, True, [], None, 'hub', True), 'hub'),         # ハブのリセットで直らなかった
    ((True, True, True, [], None, 'imu', False), 'imu'),        # 直らないがハブ不可
    ((False, False, True, [], True, None, True), 'hub'),        # 繋がっているのに未認識
    ((False, False, True, [], True, None, False), 'none'),      # 未認識だがハブ不可
    ((False, False, True, [], False, None, True), 'none'),      # 抜かれている
    ((False, False, True, [], None, None, True), 'none'),       # ポート状態が不明
    ((False, False, True, [], None, 'imu', True), 'hub'),       # リセット後に再認識されない
    ((True, True, False, [], None, None, True), 'none'),        # ハブが無応答
    ((False, False, False, [], True, 'imu', True), 'none'),     # ハブが無応答(未認識)
])
def test_choose_action(args, expected):
    action, reason = wd.choose_action(*args)
    assert action == expected
    assert reason


# ---------------------------------------------------------------- sysfs helpers

def make_usb_device(path, busnum, devnum, speed='12'):
    path.mkdir(parents=True)
    (path / 'busnum').write_text(f'{busnum}\n')
    (path / 'devnum').write_text(f'{devnum}\n')
    (path / 'speed').write_text(f'{speed}\n')
    return path


@pytest.fixture
def sysfs(tmp_path):
    """/sys/devices と /sys/class/tty を真似たツリー。"""
    hub = make_usb_device(tmp_path / 'devices' / 'usb1' / '1-2' / '1-2.3', 1, 7, '480')
    imu = make_usb_device(hub / '1-2.3.1', 1, 23)
    (imu / '1-2.3.1:1.0' / 'ttyUSB2').mkdir(parents=True)     # usb-serial
    acm = make_usb_device(hub / '1-2.3.2', 1, 24)
    (acm / '1-2.3.2:1.0').mkdir(parents=True)                 # cdc-acm
    make_usb_device(hub / '1-2.3.3', 1, 25, '480')            # ハイスピード機器(TTを通らない)
    tty = tmp_path / 'class' / 'tty'
    tty.mkdir(parents=True)
    (tty / 'ttyUSB2').mkdir()
    os.symlink(imu / '1-2.3.1:1.0' / 'ttyUSB2', tty / 'ttyUSB2' / 'device')
    (tty / 'ttyACM0').mkdir()
    os.symlink(acm / '1-2.3.2:1.0', tty / 'ttyACM0' / 'device')
    return SimpleNamespace(root=str(tmp_path), hub=str(hub), imu=str(imu), acm=str(acm))


def test_usb_device_of_tty(sysfs):
    assert wd.usb_device_of_tty('ttyUSB2', sysfs.root) == os.path.realpath(sysfs.imu)
    assert wd.usb_device_of_tty('ttyACM0', sysfs.root) == os.path.realpath(sysfs.acm)
    assert wd.usb_device_of_tty('ttyUSB9', sysfs.root) is None


def test_usb_bus_dev(sysfs):
    assert wd.usb_bus_dev(sysfs.hub) == '001/007'
    assert wd.usb_bus_dev(os.path.join(sysfs.hub, 'missing')) is None


def test_port_number():
    assert wd.port_number('/sys/devices/usb1/1-2/1-2.3/1-2.3.1') == 1
    assert wd.port_number('/sys/devices/usb1/1-2') == 2
    assert wd.port_number('/sys/devices/usb1') is None


def test_full_speed_siblings(sysfs):
    siblings = wd.full_speed_siblings(sysfs.hub, sysfs.imu)
    # 自分自身とハイスピード機器(1-2.3.3)は含めない
    assert siblings == [sysfs.acm]


# ---------------------------------------------------------------- USB requests

@pytest.mark.parametrize('reply, expected', [
    (struct.pack('<HH', 0x0103, 0), True),    # power enable connect
    (struct.pack('<HH', 0x0101, 0), True),    # power connect (認識失敗中)
    (struct.pack('<HH', 0x0100, 0), False),   # power のみ(何も挿さっていない)
    (False, None),                            # ハブが応答しない
    (None, None),                             # 判定不能
])
def test_hub_port_connected(monkeypatch, reply, expected):
    calls = []

    def fake_read(device, request_type, request, value, index, length, timeout_ms=1000):
        calls.append((device, request_type, request, value, index, length))
        return reply

    monkeypatch.setattr(wd, 'usb_control_read', fake_read)
    assert wd.hub_port_connected('/sys/hub', 1) is expected
    # ハブクラス・ポート宛てのGET_STATUS(読み取りのみ)
    assert calls == [('/sys/hub', 0xA3, 0x00, 0, 1, 4)]


def test_hub_port_connected_without_port():
    assert wd.hub_port_connected('/sys/hub', None) is None
    assert wd.hub_port_connected(None, 1) is None


@pytest.mark.parametrize('reply, expected', [(b'\x00\x00', True), (False, False), (None, None)])
def test_usb_probe(monkeypatch, reply, expected):
    monkeypatch.setattr(wd, 'usb_control_read', lambda *a, **k: reply)
    assert wd.usb_probe('/sys/dev') is expected


# ---------------------------------------------------------------- node behaviour

class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def env(tmp_path, monkeypatch):
    rclpy.init()
    hub = make_usb_device(tmp_path / '1-2.3', 1, 7, '480')
    imu = make_usb_device(hub / '1-2.3.1', 1, 23)
    node = wd.ImuUsbWatchdog()
    clock = FakeClock()
    node.now = clock
    node.baseline = node.last_check = clock.t
    state = {'present': True, 'publishers': 1, 'imu_ok': True, 'hub_ok': True,
             'port_connected': True}
    resets = []

    node.find_imu = lambda: str(imu) if state['present'] else None
    node.count_publishers = lambda topic: state['publishers']
    monkeypatch.setattr(wd, 'usb_probe',
                        lambda dev: state['hub_ok'] if dev == str(hub) else state['imu_ok'])
    monkeypatch.setattr(wd, 'full_speed_siblings', lambda h, e: [])
    monkeypatch.setattr(wd, 'hub_port_connected', lambda h, p: state['port_connected'])

    def fake_start_reset(device):
        # usbresetがすぐ終わったことにする
        resets.append(device)
        node.baseline = clock()
        node.last_reset_done = clock()

    node.start_reset = fake_start_reset
    yield SimpleNamespace(node=node, clock=clock, state=state, resets=resets,
                          imu=str(imu), hub=str(hub))
    node.destroy_node()
    rclpy.shutdown()


def advance(env, seconds, data=False):
    env.clock.t += seconds
    if data:
        env.node.on_data(None)
    env.node.check()


def test_no_reset_while_receiving(env):
    for _ in range(20):
        advance(env, 5, data=True)
    assert env.resets == []
    assert env.node.consecutive_resets == 0


def test_stall_resets_imu_then_hub(env):
    advance(env, 0)                         # IMUを見つけて計測開始
    advance(env, 25)
    assert env.resets == []                 # stall_timeout(30s)未満
    advance(env, 6)
    assert env.resets == [env.imu]          # まずIMUだけをリセット
    assert env.node.last_action == 'imu'
    advance(env, 25)
    assert env.resets == [env.imu]          # リセット後も30s待つ
    advance(env, 6)
    assert env.resets == [env.imu, env.hub]  # 直らなければハブへ
    assert env.node.last_action == 'hub'


def test_hub_reset_repeats_with_backoff(env):
    advance(env, 0)
    advance(env, 31)
    advance(env, 31)
    assert env.resets == [env.imu, env.hub]
    advance(env, 200)
    assert len(env.resets) == 2             # 2回目以降は 120s * 2 = 240s 待つ
    advance(env, 45)
    assert env.resets == [env.imu, env.hub, env.hub]


def test_recovery_clears_escalation(env):
    advance(env, 0)
    advance(env, 31)
    assert env.node.consecutive_resets == 1
    advance(env, 5, data=True)
    assert env.node.consecutive_resets == 0
    assert env.node.last_action is None
    advance(env, 31)
    assert env.resets == [env.imu]          # 前回のリセットから120s経つまでは待つ
    advance(env, 90)
    assert env.resets == [env.imu, env.imu]  # 復旧後の次の停止は再びIMUから


def test_no_reset_without_publisher(env):
    env.state['publishers'] = 0             # spresense_imu_nodeが止まっている
    advance(env, 0)
    advance(env, 60)
    assert env.resets == []


def test_missing_but_connected_resets_hub(env):
    advance(env, 1, data=True)              # 一度見つけてハブとポートを覚える
    env.state['present'] = False            # 認識が外れる
    advance(env, 1)
    advance(env, 25)
    assert env.resets == []
    advance(env, 6)
    assert env.resets == [env.hub]


def test_missing_and_unplugged_does_nothing(env):
    advance(env, 1, data=True)
    env.state['present'] = False
    env.state['port_connected'] = False     # ポートに何も繋がっていない
    advance(env, 1)
    advance(env, 31)
    assert env.resets == []
    assert env.node.consecutive_resets == 1  # 判断は記録して、間隔を延ばす


def test_never_found_does_nothing(env):
    env.state['present'] = False            # 起動時から見つからない(どのハブか不明)
    advance(env, 120)
    assert env.resets == []


def test_reenumeration_restarts_stall_timer(env):
    advance(env, 0)
    advance(env, 20)
    env.state['present'] = False
    advance(env, 5)
    env.state['present'] = True             # 抜き差し等で認識し直された
    advance(env, 5)
    advance(env, 20)
    assert env.resets == []                 # 再認識から30s経っていない
    advance(env, 11)
    assert env.resets == [env.imu]


def test_start_reset_dry_run(env, monkeypatch):
    def must_not_run(*a, **k):
        raise AssertionError('usbreset must not run in dry_run')

    monkeypatch.setattr(wd.subprocess, 'Popen', must_not_run)
    env.node.dry_run = True
    wd.ImuUsbWatchdog.start_reset(env.node, env.hub)
    assert env.node.reset_proc is None
    assert env.node.last_reset_done == env.clock.t
    assert env.node.total_resets == 0


def test_start_reset_runs_usbreset(env, monkeypatch):
    calls = []

    class FakePopen:
        def __init__(self, args, **kwargs):
            calls.append(args)

    monkeypatch.setattr(wd.subprocess, 'Popen', FakePopen)
    wd.ImuUsbWatchdog.start_reset(env.node, env.hub)
    assert calls == [['sudo', '-n', 'usbreset', '001/007']]
    assert env.node.total_resets == 1
