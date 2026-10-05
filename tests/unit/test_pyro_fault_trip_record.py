"""Pyro fault-current trip record (PYRO_FAULT_TRIP_MSG, 0x94, #1553).

No flight log carries the record yet (it is new), so the .bin under test is
synthesised with the framing the out computer writes:
[AA 55 AA 55][type][len][payload][CRC16 BE].  The 13-byte wire layout is pinned
on the firmware side by a static_assert and the host gtest; this pins the
Python reading of it, that the parser registers the type, and that the health
section reports every trip as a PROBLEM.
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))

import plot_flight_data_mini as pfd  # noqa: E402

PREAMBLE = bytes([0xAA, 0x55, 0xAA, 0x55])
FMT = "<IhHHBBB"


def frame(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    crc = pfd.crc16(body)
    return PREAMBLE + body + bytes([crc >> 8, crc & 0xFF])


def trip(time_us, peak_counts, idx, held_ms, reason, source, phase) -> bytes:
    return struct.pack(FMT, time_us, peak_counts, idx, held_ms, reason, source, phase)


def test_registered_with_the_firmware_size():
    assert pfd.MSG_PYRO_FAULT_TRIP == 0x94
    assert pfd.MSG_NAMES[0x94] == "PyroFaultTrip"
    assert pfd.MSG_EXPECTED_LEN[0x94] == 13
    assert struct.calcsize(pfd.FMT_PYRO_FAULT_TRIP) == 13
    assert pfd.FMT_PYRO_FAULT_TRIP.replace(" ", "") == FMT


def test_decode_synthetic_payload():
    rec = pfd.decode_pyro_fault_trip(trip(812_345_678, 8000, 1, 0, 2, 1, 1))
    assert rec["time_us"] == 812_345_678
    assert rec["peak_counts"] == 8000
    assert rec["peak_a"] == 10.0          # 800 counts per amp
    assert rec["clipped"] is False
    assert rec["trip_index"] == 1
    assert rec["held_ms"] == 0
    assert (rec["reason_name"], rec["source_name"], rec["phase_name"]) == (
        "flight", "shunt poll", "trip")

    clipped = pfd.decode_pyro_fault_trip(trip(1, 32767, 2, 140, 1, 2, 2))
    assert clipped["clipped"] is True
    assert (clipped["reason_name"], clipped["source_name"], clipped["phase_name"]) == (
        "fire test", "INA_ALERT edge", "release")

    assert pfd.decode_pyro_fault_trip(b"\x00" * 12) is None
    assert pfd.decode_pyro_fault_trip(b"\x00" * 14) is None


def build_log(tmp_path: Path) -> Path:
    frames = [
        # Trip 1: flight, shunt poll, released after 85 ms with a higher peak.
        frame(0x94, trip(812_000_000, 6400, 1, 0, 2, 1, 1)),
        frame(0x94, trip(812_085_000, 9600, 1, 85, 2, 1, 2)),
        # Trip 2: clipped, INA_ALERT, and the log ends before the release.
        frame(0x94, trip(900_000_000, 32767, 2, 0, 2, 2, 1)),
        # A wrong-length 0x94 must be skipped by the length gate, not decoded.
        frame(0x94, b"\x00" * 12),
    ]
    p = tmp_path / "flight_pyro_fault.bin"
    p.write_bytes(b"".join(frames) + b"\x00" * 16)
    return p


def test_parser_collects_trips_into_config(tmp_path):
    records, stats, config = pfd.parse_binary_file(str(build_log(tmp_path)))
    trips = config["pyro_fault_trips"]
    assert [(t["trip_index"], t["phase"]) for t in trips] == [(1, 1), (1, 2), (2, 1)]
    assert trips[1]["held_ms"] == 85
    assert stats["type_counts"]["PyroFaultTrip"] == 3
    # Discrete OC-clock events: kept out of the flight-clock streams.
    assert "PyroFaultTrip" not in records


def test_health_reports_every_trip_as_a_problem(tmp_path):
    from flight_report.modules import health

    _records, _stats, config = pfd.parse_binary_file(str(build_log(tmp_path)))
    verdict = health._pyro_fault(config)
    assert verdict["status"] == health.PROBLEM
    d = verdict["detail"]
    assert d.startswith("2 pyro fault-current trips")
    assert "trip #1 at OC uptime 812.000 s, peak 12.00 A, during flight, detected by shunt poll, consent held low 85 ms" in d
    assert "trip #2 at OC uptime 900.000 s, peak >= 40.96 A (shunt clipped)" in d
    assert "INA_ALERT edge" in d
    assert "no release record" in d


def test_health_omits_the_check_without_records():
    from flight_report.modules import health

    assert health._pyro_fault({"pyro_fault_trips": []}) is None
    assert health._pyro_fault({}) is None


def test_catalog_says_it_is_decoded_elsewhere():
    from flight_report import catalog

    out = catalog._unreadable({"type_counts": {"PyroFaultTrip": 2}}, {})
    assert len(out) == 1 and out[0].decoded_elsewhere
