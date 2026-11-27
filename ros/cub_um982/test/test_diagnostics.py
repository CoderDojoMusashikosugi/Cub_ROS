"""Diagnostic levels: STALE / ERROR / WARN / OK rules shared by main and secondary antennas."""

from pathlib import Path
import sys
from unittest.mock import patch

from diagnostic_msgs.msg import DiagnosticStatus
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cub_um982.extended_client import AntennaGgaState, ExtendedUM982Client
from cub_um982.um982_node import classify_antenna_status


def state(time=100.0, has_position=True, rtk_state='rtk_fix'):
    return AntennaGgaState(time=time, has_position=has_position, rtk_state=rtk_state)


def classify(link_problem=None, gga_state=None, now=100.5):
    return classify_antenna_status(link_problem, gga_state, now, 2.0)


def test_no_communication_is_stale_even_with_old_fix():
    level, message = classify('not connected', state())
    assert level == DiagnosticStatus.STALE
    assert 'not connected' in message


def test_no_gga_while_communicating_is_error():
    assert classify(gga_state=None)[0] == DiagnosticStatus.ERROR
    level, message = classify_antenna_status(None, state(time=90.0), 100.0, 2.0, 'GGAH')
    assert level == DiagnosticStatus.ERROR
    assert 'GGAH' in message


def test_no_position_is_error():
    level, message = classify(gga_state=state(has_position=False, rtk_state='no_fix'))
    assert level == DiagnosticStatus.ERROR
    assert 'No position fix' in message


@pytest.mark.parametrize('rtk_state', ['standalone', 'dgps', 'rtk_float', 'unknown'])
def test_position_without_rtk_fix_is_warn(rtk_state):
    assert classify(gga_state=state(rtk_state=rtk_state))[0] == DiagnosticStatus.WARN


def test_rtk_fix_is_ok():
    assert classify(gga_state=state())[0] == DiagnosticStatus.OK


PRIMARY_FIX = '$GNGGA,085947.90,3530.0000,N,13930.0000,E,4,18,0.7,10.0,M,30.0,M,1,0*00'
SECONDARY_FLOAT = '$GNGGAH,085947.90,3530.0001,N,13930.0001,E,5,17,0.8,10.1,M,30.0,M,1,0*00'
# Satellites tracked but no solution yet
PRIMARY_TRACKING_NO_FIX = '$GNGGA,085947.90,,,,,0,12,9999.0,,,,,,*46'
SECONDARY_EMPTY = '$GNGGAH,,,,,,0,,,,,,,,*30'


def receive(client, lines):
    packets = iter(lines)

    def read():
        try:
            return next(packets)
        except StopIteration:
            client._stop.set()
            return None

    client._stop.clear()
    with patch.object(client, '_readline', side_effect=read):
        client._reader_loop()


def test_gga_state_tracks_each_antenna_independently():
    client = ExtendedUM982Client('/unused')
    receive(client, [PRIMARY_FIX, SECONDARY_FLOAT])
    assert client.get_gga_state(False).has_position
    assert client.get_gga_state(False).rtk_state == 'rtk_fix'
    assert client.get_gga_state(True).rtk_state == 'rtk_float'


def test_losing_fix_replaces_previous_state():
    client = ExtendedUM982Client('/unused')
    receive(client, [PRIMARY_FIX, SECONDARY_FLOAT])
    receive(client, [PRIMARY_TRACKING_NO_FIX, SECONDARY_EMPTY])
    primary = client.get_gga_state(False)
    secondary = client.get_gga_state(True)
    assert not primary.has_position and primary.rtk_state == 'no_fix'
    assert primary.gga.num_sats == 12  # tracked satellites are still reported
    assert not secondary.has_position and secondary.gga is None
