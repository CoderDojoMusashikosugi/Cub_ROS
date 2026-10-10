"""Regressions for dual antenna output, indoor reception and heading validity."""

from pathlib import Path
import sys
from unittest.mock import Mock, patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cub_um982.extended_client import ExtendedUM982Client, parse_receiver_heading
from cub_um982.configure_um982 import load_commands
from um982.nmea import gps_week_seconds_to_unix


PRIMARY = '$GNGGA,085947.90,3530.0000,N,13930.0000,E,4,18,0.7,10.0,M,30.0,M,1,0*00'
SECONDARY = '$GNGGAH,085947.90,3530.0001,N,13930.0001,E,4,17,0.8,10.1,M,30.0,M,1,0*00'
# Captured from the indoor UM982 (R4.10Build13495).
NO_FIX = '$GNGGA,085947.90,,,,,0,00,9999.0,,,,,,*46'
NO_FIX_SECONDARY = '$GNGGAH,085947.90,,,,,0,00,9999.0,,,,,,*0E'
HEADING = (
    '#HEADINGA,COM1,13495,92.0,FINE,2439,550805.900,76924,5,18;'
    'SOL_COMPUTED,NARROW_INT,0.2000,90.0000,1.0000,0.0000,0.5,0.6,"",18,17,17,17'
)
NATIVE_HEADING = (
    '#UNIHEADINGA,94,GPS,FINE,2439,550805900,0,0,18,7;'
    'SOL_COMPUTED,NARROW_INT,0.2000,90.0000,1.0000,0.0000,0.5,0.6,"",18,17,17,17'
)


def test_configuration_preset_enables_secondary_without_tty():
    with patch('sys.stdin') as stdin:
        stdin.isatty.return_value = False
        stdin.readlines.return_value = []
        commands = load_commands()
    assert 'GPGGAH COM1 0.1' in commands
    assert commands[-1] == 'SAVECONFIG'


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


def test_start_requests_secondary_using_accepted_command_syntax():
    client = ExtendedUM982Client('/unused', output_rate=10)
    port = Mock()
    client._ser = port
    with patch.object(client, '_reader_loop'), patch('cub_um982.extended_client.time.sleep'):
        client.start()
        client.stop()
    commands = [call.args[0].decode().strip() for call in port.write.call_args_list]
    assert 'GPGGAH 0.1' in commands
    assert 'GPGGA 0.1' in commands
    assert 'HEADINGA 0.1' in commands
    assert all(not command.startswith('LOG ') for command in commands)


@pytest.mark.parametrize('lines', [(PRIMARY, SECONDARY), (SECONDARY, PRIMARY)])
def test_antennas_stay_separate_and_ntrip_uses_primary(lines):
    client = ExtendedUM982Client('/unused')
    primary, secondary = [], []
    client.set_position_callback(primary.append)
    client.set_secondary_position_callback(secondary.append)
    receive(client, lines)
    assert len(primary) == len(secondary) == 1
    assert primary[0].lat != secondary[0].lat
    assert client.get_position().num_sats == 18
    assert client.get_secondary_position().num_sats == 17
    assert 'GGAH' not in client._get_latest_gga_raw()
    assert '3530.0000' in client._get_latest_gga_raw()


def test_indoor_packets_are_received_but_have_no_fix():
    client = ExtendedUM982Client('/unused')
    secondary = []
    client.set_secondary_position_callback(secondary.append)
    receive(client, [NO_FIX, NO_FIX_SECONDARY])
    assert len(secondary) == 1
    assert not secondary[0].is_valid
    assert secondary[0].num_sats == 0
    assert client.no_fix_count == client.secondary_no_fix_count == 1
    assert client.last_no_fix_time is not None
    assert client.last_secondary_no_fix_time is not None


def test_empty_utc_no_fix_is_counted_for_each_antenna():
    client = ExtendedUM982Client('/unused')
    receive(client, ['$GNGGA,,,,,,0,,,,,,,,*78', '$GNGGAH,,,,,,0,,,,,,,,*30'])
    assert client.no_fix_count == client.secondary_no_fix_count == 1


@pytest.mark.parametrize('packet', [HEADING, NATIVE_HEADING])
def test_heading_formats_have_same_gnss_time_and_angles(packet):
    heading = parse_receiver_heading(packet, 18)
    assert heading.timestamp == pytest.approx(gps_week_seconds_to_unix(2439, 550805.9, 18))
    assert heading.heading_deg == 90.0
    assert heading.heading_stddev == 0.5


@pytest.mark.parametrize('packet', [HEADING, NATIVE_HEADING])
def test_unsolved_heading_clears_previous_attitude(packet):
    client = ExtendedUM982Client('/unused')
    receive(client, [packet, PRIMARY])
    assert client.get_position().heading == 90.0
    unsolved = packet.replace('SOL_COMPUTED,NARROW_INT', 'INSUFFICIENT_OBS,NONE')
    receive(client, [unsolved, PRIMARY])
    assert client.get_position().heading is None
