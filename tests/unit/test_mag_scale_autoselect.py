"""Per-log magnetometer count-scale auto-selection.

The IIS2MDC-named (0xD1) log stream carries raw int16 counts at a
sensitivity that depends on which board wrote the log: the big board's
IIS2MDC (0.15 µT/LSB) or the rocket-computer-mini's QMC5883P (100/3750
µT/LSB, #797).  OUT_STATUS_QUERY v6 stamps the chip in a mag_type byte;
`parse_binary_file` must key the scale off it and assume the IIS2MDC for
older logs (every pre-v6 log came from a big board).

Frames are synthesized byte-exact (same packing as the firmware structs,
CRC16 via the parser's own implementation) so these tests exercise the
real framing path, not a shortcut into the post-processing.
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))

import plot_flight_data_mini as pfd  # noqa: E402

QMC_SCALE = 100.0 / 3750.0


def frame(msg_type, payload):
    body = bytes([msg_type, len(payload)]) + payload
    crc = pfd.crc16(body)
    return pfd.SYNC + body + bytes([crc >> 8, crc & 0xFF])


def status_query(fmt_ver, mag_type=None):
    """OutStatusQueryData: v5 = 41 bytes; v6 appends the mag_type byte."""
    p = struct.pack(
        "<BHHhhB hhh BB hhhh h",
        16, 256, 4000, -4500, 0, fmt_ver,
        0, 0, 0,                 # hg bias
        0, 0, 10000, 0, 0, 0,    # b2r identity
        0,                       # iis2mdc rotation
    )
    p += struct.pack("<ffhBBB", 0, 0, 0, 0, 0, 0)  # v5 guidance-target echo
    if mag_type is not None:
        p += bytes([mag_type])
    return frame(0xA0, p)


def mag_frame(t_us, x, y, z):
    return frame(0xD1, struct.pack("<Ihhh", t_us, x, y, z))


def parse(tmp_path, fmt_ver, mag_type):
    log = tmp_path / "log.bin"
    log.write_bytes(status_query(fmt_ver, mag_type) + mag_frame(1000, 1000, 2000, -3000))
    return pfd.parse_binary_file(str(log))


def test_v6_qmc5883p_scale(tmp_path):
    records, _, config = parse(tmp_path, 6, 1)
    assert config["mag_type"] == 1
    assert config["mag_ut_per_lsb"] == pfd.QMC5883P_UT_PER_LSB == QMC_SCALE
    rec = records["IIS2MDC"][0]
    assert rec["mag_x"] == 1000 * QMC_SCALE
    assert rec["mag_z"] == -3000 * QMC_SCALE


def test_v6_iis2mdc_scale(tmp_path):
    records, _, config = parse(tmp_path, 6, 0)
    assert config["mag_type"] == 0
    assert config["mag_ut_per_lsb"] == pfd.IIS2MDC_UT_PER_LSB == 0.15
    assert records["IIS2MDC"][0]["mag_x"] == 150.0


def test_v6_unknown_mag_type_falls_back_to_iis2mdc(tmp_path):
    _, _, config = parse(tmp_path, 6, 7)
    assert config["mag_type"] == 7
    assert config["mag_ut_per_lsb"] == 0.15


def test_v5_log_assumes_iis2mdc(tmp_path):
    records, _, config = parse(tmp_path, 5, None)
    assert config["mag_type"] is None
    assert config["mag_ut_per_lsb"] == 0.15
    # The rest of the v5 config must decode exactly as before the v6 field.
    assert config["low_g_fs_g"] == 16
    assert config["ism6_rot_z_deg"] == -45.0
    assert records["IIS2MDC"][0]["mag_x"] == 150.0
