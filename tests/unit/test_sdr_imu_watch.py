"""The gnss-sdr-dev board's IMU watch in a flight log: the extractor's summary of the 4001 aiding block's version 2.

The board drops its IMU from the tracking loops only when the IMU's data stop
or cannot be read, and refuses it when its output at rest is not near zero
(the pad check); every epoch's aiding block says so (the board's
receiver/README.md, "The IMU's watch"). After a flight the extractor must make
any drop impossible to miss: a first line that says DROPPED with the count, each
drop with its time, the flight computer's phase, its reason and when the IMU
was back. The frames here come from a bit packer of their own (the layout
checked against tests_cpp/test_sdr_aid.cpp's E4, which the board's encoder
also makes byte for byte).
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))
if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import extract_gnss_rtcm as ext              # noqa: E402
from test_sdr_rtcm_record import build_log   # noqa: E402

# tests_cpp/test_sdr_aid.cpp's kE4: the board's epoch with a version 2 block (the IMU refused by the pad check in the
# boost phase after three drops, the last for its attitude; the loops' own share 1.4, -0.6, -19.2 m/s^2 from 9
# channels; the pad window 1.84 m/s^2).
E4 = bytes.fromhex("D30029FA1109579032 0FDA17B87FFFFFFFFE290CBAD6C48D159E26AF030AEF0188A4B38F00D2407FF7A05C008AF2DB"
                   .replace(" ", ""))


def crc24q(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def epoch(tow_ms: int, src: int, check: int, phase: int, why: int, drops: int, last: int,
          lo=(0.0, 0.0, 0.0), pad=0.1, ver: int = 2) -> bytes:
    """A 4001 sub-type 1 epoch with no marks and an aiding block (the board's layout)."""
    bits = []

    def u(n, v):
        v &= (1 << n) - 1
        bits.extend((v >> (n - 1 - k)) & 1 for k in range(n))

    u(12, 4001), u(4, 1), u(16, 2443), u(30, tow_ms), u(20, 0), u(16, 0), u(48, 0), u(2, 1), u(6, 20), u(48, 0)
    u(6, 0), u(6, 0)
    u(4, ver), u(2, src), u(2, check), u(1, phase == 2), u(1, 0), u(2, 1), u(1, 0), u(3, phase), u(1, 1), u(1, 1)
    u(1, 0), u(4, why if ver >= 2 else 0), u(1, 0), u(8, 0)
    if ver >= 2:
        u(8, drops), u(4, last), u(4, 8)
        for x in lo:
            u(10, round(x / 0.2))
        u(8, round(pad / 0.02)), u(2, 0)
    bits += [0] * (-len(bits) % 8)
    pay = bytes(int("".join(map(str, bits[i:i + 8])), 2) for i in range(0, len(bits), 8))
    f = bytes([0xD3, len(pay) >> 8, len(pay) & 0xFF]) + pay
    c = crc24q(f)
    return f + bytes([c >> 16, (c >> 8) & 0xFF, c & 0xFF])


def flight() -> list[bytes]:
    """10 s at 100 Hz: the pad check passing at 0.5 s, the boost from 3 s (the IMU's attitude lapsing at 3.5 s for
    0.5 s), the coast from 5 s (one epoch not a number at 6 s)."""
    out, drops, last = [], 0, 0
    for k in range(1000):
        t = k / 100
        phase = 1 if t < 3.0 else (2 if t < 5.0 else 3)
        src, why, check = 2, 0, 0 if t < 0.5 else 1
        if k == 350:
            drops, last = drops + 1, 4
        if 350 <= k < 400:
            src, why = 0, 4
        if k == 600:
            drops, last, src, why = drops + 1, 5, 0, 5
        lo = (0.2, -0.2, 0.4) if phase == 1 else ((1.0, 2.0, -18.0) if phase == 2 else (0.4, 0.4, 1.0))
        out.append(epoch(204_000_000 + 10 * k, src, check, phase, why, drops, last, lo, 0.12))
    return out


def test_the_v2_block_decodes():
    assert ext.frame_ok(E4)
    d = ext.decode_epoch_aid(E4)
    assert d["ver"] == 2 and d["src"] == 0 and d["check"] == 2 and d["phase"] == 2 and d["fc_frames"] == 0x3C
    assert d["why"] == 7 and d["drops"] == 3 and d["last"] == 4 and d["lo_n"] == 9
    assert [round(x, 3) for x in d["lo"]] == [1.4, -0.6, -19.2]
    assert abs(d["pad"] - 1.84) < 1e-9
    # the packer here makes the same frame
    assert ext.decode_epoch_aid(epoch(0, 0, 2, 2, 7, 3, 4, (1.4, -0.6, -19.2), 1.84))["drops"] == 3


def test_a_version_1_block_has_no_watch():
    d = ext.decode_epoch_aid(epoch(1000, 2, 0, 1, 0, 0, 0, ver=1))
    assert d["ver"] == 1 and d["why"] == 0 and d["drops"] == 0 and d["lo"] is None and d["pad"] is None
    assert ext.imu_watch([(0, epoch(1000, 2, 0, 1, 0, 0, 0, ver=1))])["rows"] == []


def test_each_drop_is_listed_with_its_phase_reason_and_return(tmp_path, capsys):
    log = build_log(tmp_path, flight())
    assert ext.main([str(log), "--aid-csv", "--aid-plot"]) == 0
    out = capsys.readouterr().out
    lines = out.splitlines()
    i = next(k for k, line in enumerate(lines) if "IMU AIDING" in line)
    assert lines[i].strip() == "IMU AIDING: DROPPED 2 TIMES (the board's IMU into its loops)"
    assert "drop 1:" in lines[i + 2] and "boost: its attitude lapsed" in lines[i + 2] and "back 0.50 s later" in lines[i + 2]
    assert "drop 2:" in lines[i + 3] and "coast: not a number" in lines[i + 3] and "back 0.01 s later" in lines[i + 3]
    assert "the pad check passing at" in out
    assert "aided " in lines[i + 1] and "(949 of 1000)" in lines[i + 1]
    csv_lines = (tmp_path / "flight.aid.csv").read_text().splitlines()
    assert len(csv_lines) == 1001 and csv_lines[0].startswith("time_us,gps_tow_ms,ver,src,check,why,drops")
    assert (tmp_path / "flight.aid.png").stat().st_size > 10_000


def test_a_flight_never_dropped_says_so(tmp_path, capsys):
    frames = [epoch(204_000_000 + 10 * k, 2, 1, 1 if k < 300 else 2, 0, 0, 0) for k in range(500)]
    assert ext.main([str(build_log(tmp_path, frames))]) == 0
    out = capsys.readouterr().out
    assert "IMU aiding: never dropped (the board's IMU into its loops)" in out
    assert "DROPPED" not in out
