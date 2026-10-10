"""The magnetometers' chip frames, as the log reader must apply them.

IIS2MDC (#1589):

ST's LIS2MDL / IIS2MDC axes are a left-handed set ("frame is left-handed"),
so no sensor->board rotation maps them onto the right-handed board frame.
With the +90 deg Rz alone, every IIS2MDC board vector had board y reversed:
the field turned the wrong way under roll and any heading from it was the
mirror image of the true one.  `parse_binary_file` negates chip X first
(`mag_type_chip_signs`), as SensorConverter::convertIIS2MDCData does.

The physical facts pinned here do not come from the converter:
  - the #204 bench (config.h): board +X north reads the field on chip -Y,
    board +X east reads it on chip -X;
  - the chip is on the top side, its Z out of the top face (board +Z).

Logs hold raw chip counts, so the rule applies to every IIS2MDC log,
including the pre-2026-06-17 ones whose firmware applied Rz(0) and whose
status query predates the v4 rotation field (the reader falls back to 90).
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))

import plot_flight_data_mini as pfd  # noqa: E402


def frame(msg_type, payload):
    body = bytes([msg_type, len(payload)]) + payload
    crc = pfd.crc16(body)
    return pfd.SYNC + body + bytes([crc >> 8, crc & 0xFF])


def status_query_v6(iis_rot_cdeg, mag_type):
    p = struct.pack(
        "<BHHhhB hhh BB hhhh h",
        16, 256, 4000, -4500, 18000, 6,
        0, 0, 0,                 # hg bias
        0, 0, 10000, 0, 0, 0,    # b2r identity
        iis_rot_cdeg,
    )
    p += struct.pack("<ffhBBB", 0, 0, 0, 0, 0, 0)  # v5 guidance-target echo
    return frame(0xA0, p + bytes([mag_type]))


def status_query_v2():
    """A 16-byte v2 query: no b2r, no IIS2MDC rotation, no mag_type — what
    the 2026-05/06 V8 logs carry."""
    return frame(0xA0, struct.pack("<BHHhhB hhh", 16, 256, 4000, -4500, 18000, 2, 0, 0, 0))


def mag_frame(t_us, x, y, z):
    return frame(0xD1, struct.pack("<Ihhh", t_us, x, y, z))


# (chip counts, board field in µT) — 150 counts = 22.5 µT on the IIS2MDC.
BENCH = [
    ((0, -150, 0), (22.5, 0.0, 0.0)),   # board +X north: field on chip -Y
    ((-150, 0, 0), (0.0, 22.5, 0.0)),   # board +X east (north = board +Y): chip -X
    ((0, 0, 300), (0.0, 0.0, 45.0)),    # chip Z is board Z
]


def parse(tmp_path, query, counts):
    log = tmp_path / "log.bin"
    log.write_bytes(query + b"".join(mag_frame(1000 * (i + 1), *c) for i, c in enumerate(counts)))
    return pfd.parse_binary_file(str(log))


@pytest.mark.parametrize("query", [status_query_v6(9000, 0), status_query_v2()],
                         ids=["v6-stamped-90", "pre-v4-fallback"])
def test_bench_readings_land_on_the_board_axes(tmp_path, query):
    records, _, config = parse(tmp_path, query, [c for c, _ in BENCH])
    assert config["iis2mdc_rot_z_deg"] == 90.0
    assert pfd.mag_type_chip_signs(config["mag_type"]) == (-1.0, 1.0, 1.0)
    for rec, (_, want) in zip(records["IIS2MDC"], BENCH):
        got = (rec["mag_x"], rec["mag_y"], rec["mag_z"])
        assert got == pytest.approx(want, abs=1e-9)


def test_the_iis2mdc_conversion_is_a_reflection(tmp_path):
    """Unit chip counts give the conversion's columns: det must be -1."""
    records, _, _ = parse(tmp_path, status_query_v6(9000, 0),
                          [(1000, 0, 0), (0, 1000, 0), (0, 0, 1000)])
    cols = [[r["mag_x"] / 150.0, r["mag_y"] / 150.0, r["mag_z"] / 150.0] for r in records["IIS2MDC"]]
    m = [[cols[c][r] for c in range(3)] for r in range(3)]
    assert m == [[pytest.approx(v, abs=1e-9) for v in row]
                 for row in ([0, -1, 0], [-1, 0, 0], [0, 0, 1])]


# QMC5883P (#1590): as TR_QMC5883P configures it the chip's axes are
# right-handed with Z pointing INTO the board; its signs (+1, -1, -1) turn Z
# back out of the top, and each board's rotation is then its placement.
# Measured on a Beetle bench log (gyro kinematics and the dip at rest);
# V10 is the same chip turned back by the 90 deg its footprint is.
QMC_BEETLE = [[0, -1, 0], [-1, 0, 0], [0, 0, -1]]
QMC_V10 = [[-1, 0, 0], [0, 1, 0], [0, 0, -1]]


def qmc_matrix(tmp_path, rot_cdeg):
    records, _, config = parse(tmp_path, status_query_v6(rot_cdeg, 1),
                               [(3750, 0, 0), (0, 3750, 0), (0, 0, 3750)])
    cols = [[r["mag_x"] / 100.0, r["mag_y"] / 100.0, r["mag_z"] / 100.0] for r in records["IIS2MDC"]]
    return np.array(cols).T, config


def test_the_qmc5883p_is_not_reflected(tmp_path):
    """The QMC5883P's axes, as configured, are right-handed: its conversion
    stays a rotation (det +1), Z included."""
    m, config = qmc_matrix(tmp_path, -9000)
    assert pfd.mag_type_chip_signs(config["mag_type"]) == (1.0, -1.0, -1.0)
    assert np.linalg.det(m) == pytest.approx(1.0, abs=1e-9)


@pytest.mark.parametrize("stamp_cdeg", [-9000, 9000], ids=["post-1590-stamp", "pre-1590-beetle-stamp"])
def test_the_beetle_qmc5883p_maps_as_measured(tmp_path, stamp_cdeg):
    """Beetle logs map as measured, whether the firmware stamped the true -90
    or, before #1590, the IIS2MDC's +90 it applied to every board."""
    m, _ = qmc_matrix(tmp_path, stamp_cdeg)
    assert m.tolist() == [[pytest.approx(v, abs=1e-9) for v in row] for row in QMC_BEETLE]


def test_the_v10_qmc5883p_maps_as_inferred(tmp_path):
    m, _ = qmc_matrix(tmp_path, 18000)
    assert m.tolist() == [[pytest.approx(v, abs=1e-9) for v in row] for row in QMC_V10]


def test_a_flat_beetle_reads_the_field_pointing_down(tmp_path):
    """The dip a 2026-10-05 Beetle bench log failed: board flat, +Z up, field
    north and down — counts from the measured matrix, field back from the
    parser with a negative board z."""
    b = np.array([21.0, 0.0, -46.0])
    counts = np.array(QMC_BEETLE, float).T @ b / pfd.QMC5883P_UT_PER_LSB
    records, _, _ = parse(tmp_path, status_query_v6(9000, 1), [tuple(int(round(c)) for c in counts)])
    rec = records["IIS2MDC"][0]
    assert (rec["mag_x"], rec["mag_y"], rec["mag_z"]) == pytest.approx(tuple(b), abs=0.1)
