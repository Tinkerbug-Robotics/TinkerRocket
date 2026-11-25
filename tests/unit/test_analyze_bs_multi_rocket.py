#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Guards for Data_Analysis/analyze_bs_multi_rocket.py (three-rocket bench).

Logs are synthesised with the same framing helpers the reader's own tests use,
so a slotted run, a colliding run and the edge cases (seq wrap, a reboot, files
on one time axis) each have a known answer.
"""

import pathlib
import struct
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'Data_Analysis'))

import analyze_bs_multi_rocket as amr  # noqa: E402
import bs_log  # noqa: E402

FAST_TOA = 102   # SF8/BW250/CR4/5, preamble 12
SLOW_TOA = 56


def _framed(mtype, payload):
    return bs_log.PREAMBLE + bytes([mtype, len(payload)]) + payload + b'\x00\x00'


def _frame(rid, seq, slow=False):
    size = bs_log.SIZE_OF_LORA_SLOW if slow else bs_log.SIZE_OF_LORA_FAST
    ftype = bs_log.LORA_FRAME_SLOW if slow else bs_log.LORA_FRAME_FAST
    b = bytearray(size)
    ver_type = (bs_log.LORA_PROTO_VERSION << 4) | ftype
    b[0:7] = struct.pack(bs_log.FMT_LORA_HDR, 240, rid, 0xFF, seq & 0xFFFF, ver_type, 0x10)
    return bytes(b)


def _rx(t_ms, frame):
    hdr = struct.pack(bs_log.FMT_BS_RX_HDR, t_ms, -410, 120, 915_000_000)
    return _framed(bs_log.BS_LORA_RX_MSG, hdr + frame)


def _event(t_ms, text):
    raw = text.encode()
    hdr = struct.pack(bs_log.FMT_BS_EVT_HDR, t_ms, 915_000_000, len(raw))
    return _framed(bs_log.BS_EVENT_MSG, hdr + raw)


def _write(path, records):
    path.write_bytes(bs_log.BS_LOG_MAGIC + bytes([bs_log.BS_LOG_FORMAT_VERSION])
                     + b''.join(records))
    return path


def _stream(rid, start_phase_ms, n, drop=lambda seq: False, seq0=0, t0=0):
    """Records for one rocket transmitting every 500 ms from start_phase_ms.

    The base station stamps a frame when it finishes arriving, so each record's
    time is start + time on air.  Every 6th frame (seq % 6 == 5) is SLOW, as in
    loraFrameTypeForSlot().
    """
    out = []
    for i in range(n):
        seq = seq0 + i
        if drop(seq):
            continue
        slow = seq % 6 == 5
        start = t0 + start_phase_ms + 500 * i
        out.append((start + (SLOW_TOA if slow else FAST_TOA), _frame(rid, seq, slow)))
    return out


def _log(tmp_path, name, *streams, events=()):
    """Write a log from rocket streams [(t, frame)] and events [(t, text)]."""
    recs = [(t, _rx(t, frame)) for s in streams for t, frame in s]
    recs += [(t, _event(t, text)) for t, text in events]
    recs.sort(key=lambda r: r[0])
    return _write(tmp_path / name, [rec for _t, rec in recs])


def test_time_on_air_matches_the_firmware_table():
    # The table in RocketComputerTypes.h: FAST 102 / SLOW 56 ms at SF8/BW250,
    # FAST 345 ms at SF10 (pinned by test_rocket_computer_types.cpp).
    assert amr.time_on_air_ms(55) == 102
    assert amr.time_on_air_ms(22) == 56
    assert amr.time_on_air_ms(55, sf=10) == 345


def test_three_slotted_rockets_have_no_loss_and_separate_phases(tmp_path):
    p = _log(tmp_path, 'lora_slots.bin',
             _stream(1, 10, 240), _stream(2, 10 + 167, 240), _stream(3, 10 + 333, 240))
    rows, shared = amr.load([p])
    stats = amr.rocket_stats(rows)
    assert shared
    assert set(stats) == {1, 2, 3}
    for rid, phase in ((1, 10), (2, 177), (3, 343)):
        assert stats[rid]['loss_pct'] == pytest.approx(0.0)
        assert stats[rid]['phase_ms'] == pytest.approx(phase, abs=1)
        assert stats[rid]['phase_spread_ms'] < 1
    pairs = amr.pair_separation(amr.window_phases(rows))
    assert pairs[(1, 2)]['min_ms'] == pytest.approx(167, abs=1)
    assert pairs[(1, 3)]['min_ms'] == pytest.approx(167, abs=1)   # 333 apart = 167 the other way
    assert all(p['overlap_windows'] == 0 for p in pairs.values())


def test_colliding_pair_reports_loss_streaks_and_overlap(tmp_path):
    # Rockets 1 and 2 start 30 ms apart; the base station keeps only every
    # other frame of each, and loses runs of 10 in the middle.
    lost = lambda seq: seq % 2 == 1 or 100 <= seq < 110
    p = _log(tmp_path, 'lora_collide.bin',
             _stream(1, 0, 200, drop=lost), _stream(2, 30, 200, drop=lost))
    rows, _ = amr.load([p])
    stats = amr.rocket_stats(rows)
    assert stats[1]['loss_pct'] == pytest.approx(100 * (1 - 95 / 199), abs=0.5)
    assert stats[1]['missed_1'] > 0
    assert stats[1]['missed_6_20'] == 1
    assert stats[1]['longest_gap_s'] == pytest.approx(6.0, abs=0.1)   # seq 98 -> 110
    pairs = amr.pair_separation(amr.window_phases(rows))
    assert pairs[(1, 2)]['overlap_windows'] == pairs[(1, 2)]['windows']


def test_seq_wrap_is_not_loss_and_a_reboot_is_a_new_segment(tmp_path):
    wrap = _stream(1, 0, 20, seq0=65530)
    reboot = _stream(1, 0, 10, seq0=0, t0=20 * 500 + 3000)
    p = _log(tmp_path, 'lora_wrap.bin', wrap, reboot)
    stats = amr.rocket_stats(amr.load([p])[0])
    # The wrap from 65535 to 0 is one step; the reboot back to seq 0 after
    # seq 13 is a new segment, not 65 000 missed frames.
    assert stats[1]['segments'] == 2
    assert stats[1]['received'] == 30
    assert stats[1]['loss_pct'] < 10


def test_files_share_a_time_axis_through_their_bs_ms_event(tmp_path):
    # Two files whose own clocks start at 0 but were opened 2 000 ms apart on
    # the base station: rocket 2 is really 167 ms after rocket 1.
    # Rocket 1 starts at BS 5010 ms (file a time 10); rocket 2 at BS 7177 ms
    # (file b time 177), which is 2167 ms later: 167 ms apart in the cycle.
    a = _log(tmp_path, 'lora_a.bin', _stream(1, 10, 40),
             events=[(0, 'log open bs_ms=5000')])
    b = _log(tmp_path, 'lora_b.bin', _stream(2, 177, 40),
             events=[(0, 'log open bs_ms=7000')])
    rows, shared = amr.load([a, b])
    assert shared
    stats = amr.rocket_stats(rows)
    sep = amr._phase_distance(stats[1]['phase_ms'], stats[2]['phase_ms'], 500)
    assert sep == pytest.approx(167, abs=1)


def test_files_without_a_time_base_are_flagged(tmp_path):
    a = _log(tmp_path, 'lora_a.bin', _stream(1, 0, 10))
    b = _log(tmp_path, 'lora_b.bin', _stream(2, 0, 10))
    _rows, shared = amr.load([a, b])
    assert not shared
    text, _stats, _pairs = amr.report(*amr.load([a, b]), 500, 30.0, {})
    assert 'no common time base' in text


def test_slot_adjust_events_are_counted_per_rocket():
    events = [dict(event='slot adjust rid=2 slot=1 err=+14 ms'),
              dict(event='slot adjust rid=2 slot=1 err=-3 ms'),
              dict(event='slot lock rid=3 slot=2 err=+120 ms'),
              dict(event='logging on')]
    assert amr.slot_adjusts(events) == {2: (2, -3), 3: (1, 120)}


def test_main_prints_a_report(tmp_path, capsys):
    p = _log(tmp_path, 'lora_main.bin', _stream(1, 0, 30), _stream(2, 167, 30),
             events=[(600, 'slot adjust rid=2 slot=1 err=+14 ms')])
    plot_path = tmp_path / 'bench.png'
    assert amr.main([str(p), '--plot', str(plot_path)]) == 0
    out = capsys.readouterr().out
    assert 'from 2 rocket(s)' in out
    assert '1-2' in out
    assert 'rocket 2: 1 adjust(s), last error +14 ms' in out
    assert plot_path.stat().st_size > 0
