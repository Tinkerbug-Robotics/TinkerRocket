#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Guards for Data_Analysis/bs_log.py — the base-station log reader (#850).

Two different things are checked here, and the second is the one that matters:

1. The reader round-trips a log it built itself (self-consistency).
2. The offsets it reads at agree with the C structs, taken from the
   `static_assert(offsetof(...) == N)` lines in RocketComputerTypes.h.

(2) is the real guard.  A pure round-trip test would happily agree with itself
while disagreeing with the firmware, which is exactly how a wire-format reader
goes silently wrong — the parser and the producer drift and nothing notices
until a flight log decodes to nonsense.
"""

import pathlib
import re
import struct
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'Data_Analysis'))

import bs_log  # noqa: E402

TYPES_H = (REPO / 'tinkerrocket-idf' / 'components' / 'TR_RocketComputerTypes'
           / 'RocketComputerTypes.h')


def c_offsets(struct_name):
    """Pull {field: offset} out of the header's offsetof static_asserts."""
    text = TYPES_H.read_text()
    pat = re.compile(
        r'static_assert\(offsetof\(' + struct_name + r',\s*(\w+)\)\s*==\s*(\d+)')
    return {m.group(1): int(m.group(2)) for m in pat.finditer(text)}


def c_sizeof(struct_name):
    text = TYPES_H.read_text()
    m = re.search(r'static_assert\(sizeof\(' + struct_name + r'\)\s*==\s*(\d+)', text)
    assert m, f'no sizeof static_assert for {struct_name}'
    return int(m.group(1))


# --------------------------------------------------------------------------
# (2) the reader's constants must match the firmware's structs
# --------------------------------------------------------------------------

def test_frame_sizes_match_the_header():
    assert bs_log.SIZE_OF_LORA_FAST == c_sizeof('LoRaFastData')
    assert bs_log.SIZE_OF_LORA_SLOW == c_sizeof('LoRaSlowData')
    assert bs_log.LORA_HDR_LEN == c_sizeof('LoRaFrameHeader')
    assert bs_log.BS_RX_HDR_LEN == c_sizeof('BsLoRaRxHeader')
    assert bs_log.BS_EVT_HDR_LEN == c_sizeof('BsEventHeader')


def test_struct_format_widths_match_the_header():
    assert struct.calcsize(bs_log.FMT_LORA_HDR) == c_sizeof('LoRaFrameHeader')
    assert struct.calcsize(bs_log.FMT_BS_RX_HDR) == c_sizeof('BsLoRaRxHeader')
    assert struct.calcsize(bs_log.FMT_BS_EVT_HDR) == c_sizeof('BsEventHeader')


def test_fast_frame_offsets_match_the_header():
    """Every offset the fast unpacker reads at, against the C struct."""
    off = c_offsets('LoRaFastData')
    o = bs_log.LORA_HDR_LEN
    expected = {
        'num_sats': o, 'pdop_u8': o + 1, 'hacc_u8': o + 2,
        'ecef_x_m': o + 3, 'ecef_y_m': o + 6, 'ecef_z_m': o + 9,
        'acc_x_x10': o + 12, 'acc_y_x10': o + 14, 'acc_z_x10': o + 16,
        'gyro_x_x10': o + 18, 'gyro_y_x10': o + 20, 'gyro_z_x10': o + 22,
        'q0': o + 24, 'q1': o + 26, 'q2': o + 28, 'q3': o + 30,
        'pressure_alt_m': o + 32, 'altitude_rate': o + 35,
        'vel_e_dms': o + 37, 'vel_n_dms': o + 39, 'vel_u_dms': o + 41,
        'sensor_health': o + 43, 'flags2': o + 47,
    }
    for field, want in expected.items():
        assert off[field] == want, (
            f'LoRaFastData.{field} is at {off[field]} in the header but the '
            f'reader parses it at {want}')


def test_slow_frame_offsets_match_the_header():
    off = c_offsets('LoRaSlowData')
    o = bs_log.LORA_HDR_LEN
    expected = {
        'max_alt_m': o, 'max_speed': o + 3, 'temp_x10': o + 5,
        'voltage_u8': o + 7, 'current_ma': o + 8, 'soc_i8': o + 10,
        'cam_ma': o + 11, 'servo_ma': o + 13,
    }
    for field, want in expected.items():
        assert off[field] == want, (
            f'LoRaSlowData.{field} is at {off[field]} in the header but the '
            f'reader parses it at {want}')


# --------------------------------------------------------------------------
# helpers that build a log the way the firmware would
# --------------------------------------------------------------------------

def _crc_stub():
    return b'\x00\x00'          # the reader is length-driven, not CRC-driven


def _framed(mtype, payload):
    return bs_log.PREAMBLE + bytes([mtype, len(payload)]) + payload + _crc_stub()


def _lora_hdr(seq, ftype, rocket_id=1, network_id=0, next_ch=0xFF, flags_state=0x10):
    ver_type = (bs_log.LORA_PROTO_VERSION << 4) | ftype
    return struct.pack(bs_log.FMT_LORA_HDR, network_id, rocket_id, next_ch,
                       seq, ver_type, flags_state)


def _fast_frame(seq, *, palt=1234, sats=11):
    b = bytearray(bs_log.SIZE_OF_LORA_FAST)
    b[0:7] = _lora_hdr(seq, bs_log.LORA_FRAME_FAST)
    o = 7
    b[o] = sats
    b[o + 1] = 2           # pdop
    b[o + 2] = 5           # hacc
    b[o + 3:o + 6] = (1000).to_bytes(3, 'little')     # ecef x
    b[o + 32:o + 35] = int(palt).to_bytes(3, 'little')
    struct.pack_into('<3h', b, o + 37, 10, -20, 30)   # vel e/n/u dm/s
    return bytes(b)


def _slow_frame(seq, *, cam_ma=1480, servo_ma=340, volt_u8=187):
    b = bytearray(bs_log.SIZE_OF_LORA_SLOW)
    b[0:7] = _lora_hdr(seq, bs_log.LORA_FRAME_SLOW)
    o = 7
    b[o:o + 3] = (2500).to_bytes(3, 'little')         # max_alt
    struct.pack_into('<h', b, o + 3, 120)             # max_speed
    struct.pack_into('<h', b, o + 5, 215)             # temp x10
    b[o + 7] = volt_u8
    struct.pack_into('<h', b, o + 8, -450)            # current mA
    struct.pack_into('<b', b, o + 10, 79)             # soc
    struct.pack_into('<2H', b, o + 11, cam_ma, servo_ma)
    return bytes(b)


def _rx_record(t_ms, frame, rssi_x10=-410, snr_x10=120, freq_hz=915_000_000):
    hdr = struct.pack(bs_log.FMT_BS_RX_HDR, t_ms, rssi_x10, snr_x10, freq_hz)
    return _framed(bs_log.BS_LORA_RX_MSG, hdr + frame)


def _write_log(path, records):
    blob = bs_log.BS_LOG_MAGIC + bytes([bs_log.BS_LOG_FORMAT_VERSION])
    path.write_bytes(blob + b''.join(records))


# --------------------------------------------------------------------------
# (1) behaviour
# --------------------------------------------------------------------------

def test_reads_both_frame_types(tmp_path):
    p = tmp_path / 'lora_001.bin'
    _write_log(p, [_rx_record(100, _fast_frame(0)),
                   _rx_record(600, _slow_frame(1))])

    rows, events, fmt = bs_log.read_bs_log(p)
    assert fmt == 'bin'
    assert [r['frame'] for r in rows] == ['fast', 'slow']
    assert rows[0]['pressure_alt'] == 1234
    assert rows[1]['cam_a'] == pytest.approx(1.48)
    assert rows[1]['servo_a'] == pytest.approx(0.34)


def test_forward_fill_across_frame_types(tmp_path):
    """The property the whole format rests on.

    A slow frame must not blank the position, and a following fast frame must
    not blank the battery — otherwise the rows alternate half-empty and the
    generated CSV is useless.
    """
    p = tmp_path / 'lora_002.bin'
    _write_log(p, [_rx_record(100, _fast_frame(0, palt=1234)),
                   _rx_record(600, _slow_frame(1)),
                   _rx_record(1100, _fast_frame(2, palt=1500))])

    rows, _, _ = bs_log.read_bs_log(p)
    assert len(rows) == 3

    # slow row keeps the position from the preceding fast frame
    assert rows[1]['pressure_alt'] == 1234
    assert rows[1]['num_sats'] == 11

    # the next fast row keeps the battery from the preceding slow frame
    assert rows[2]['pressure_alt'] == 1500
    assert rows[2]['cam_a'] == pytest.approx(1.48)
    assert rows[2]['soc'] == 79


def test_gap_is_computed_from_seq(tmp_path):
    p = tmp_path / 'lora_003.bin'
    _write_log(p, [_rx_record(100, _fast_frame(10)),
                   _rx_record(600, _fast_frame(11)),
                   _rx_record(1100, _fast_frame(15))])   # 3 missed
    rows, _, _ = bs_log.read_bs_log(p)
    assert [r['gap'] for r in rows] == [-1, 0, 3]


def test_unknown_rssi_is_nan_not_zero(tmp_path):
    """0 dBm is a legal reading; "no reading" must not masquerade as one."""
    p = tmp_path / 'lora_004.bin'
    _write_log(p, [_rx_record(100, _fast_frame(0),
                              rssi_x10=bs_log.BS_RSSI_UNKNOWN,
                              snr_x10=bs_log.BS_RSSI_UNKNOWN)])
    rows, _, _ = bs_log.read_bs_log(p)
    assert rows[0]['rssi'] != rows[0]['rssi']     # NaN
    assert rows[0]['snr'] != rows[0]['snr']


def test_events_are_read(tmp_path):
    p = tmp_path / 'lora_005.bin'
    text = b'hop start'
    evt = struct.pack(bs_log.FMT_BS_EVT_HDR, 4200, 915_500_000, len(text)) + text
    _write_log(p, [_rx_record(100, _fast_frame(0)),
                   _framed(bs_log.BS_EVENT_MSG, evt)])
    rows, events, _ = bs_log.read_bs_log(p)
    assert len(rows) == 1
    assert len(events) == 1
    assert events[0]['event'] == 'hop start'
    assert events[0]['rx_freq_mhz'] == pytest.approx(915.5)


def test_wrong_proto_version_is_skipped(tmp_path):
    p = tmp_path / 'lora_006.bin'
    bad = bytearray(_fast_frame(0))
    bad[5] = (4 << 4) | bs_log.LORA_FRAME_FAST      # pretend to be v4
    _write_log(p, [_rx_record(100, bytes(bad)),
                   _rx_record(600, _fast_frame(1))])
    rows, _, _ = bs_log.read_bs_log(p)
    assert len(rows) == 1
    assert rows[0]['seq'] == 1


def test_bad_magic_is_refused(tmp_path):
    p = tmp_path / 'not_a_log.bin'
    p.write_bytes(b'\x00' * 64)
    with pytest.raises(ValueError, match='magic'):
        bs_log.read_bs_bin(p)


def test_legacy_csv_still_reads():
    """The committed example flight must keep working — CSV cannot be
    regenerated from binary, so dropping this reader would strand it."""
    legacy = REPO / 'examples' / 'flights' / 'lora_20260705_173025.csv'
    if not legacy.exists():
        pytest.skip('example flight not present')
    rows, events, fmt = bs_log.read_bs_log(legacy)
    assert fmt == 'csv'
    assert len(rows) > 100
    assert all('seq' in r for r in rows[:10])


def test_format_detection_is_by_content_not_extension(tmp_path):
    """A renamed file must still read correctly."""
    src = REPO / 'examples' / 'flights' / 'lora_20260705_173025.csv'
    if not src.exists():
        pytest.skip('example flight not present')
    misnamed = tmp_path / 'lora_007.bin'      # CSV content, .bin name
    misnamed.write_bytes(src.read_bytes())
    _, _, fmt = bs_log.read_bs_log(misnamed)
    assert fmt == 'csv'
