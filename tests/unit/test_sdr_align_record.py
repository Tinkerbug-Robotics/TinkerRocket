"""The gnss-sdr-dev board IMU's mounting in a flight log: the extractor's line for the board's last 4001 sub-type 5.

The board reports how its IMU is mounted on the flight computer, and its
accelerometer's calibration, once a second and at each change; the flight
computer logs those frames among the board's. After a flight the extractor
says in one line what the board flew with. The frames are the alignment
protocol's goldens (align_codec.py), which tests_cpp/test_sdr_align.cpp pins
too.
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

# Six faces, whole, U10 turned 45 deg about z on the flight computer's IMU, every axis calibrated, not saved yet.
A2 = bytes.fromhex(
    "D3003BFA15C911B36DC96069532B40E185A82000000005A820961F5D802580BF5EAD400B200FE0000050200000600FE000401020002000"
    "0FE08000103FE02AAFF6")
# One pose on a saved rotation (from 3), against the body, the rail pose 1.234 deg off it.
A3 = bytes.fromhex(
    "D30027FA1505016E8210482019401FFFE7FE00596000000000000000000000000000000007D0201F20809FB490")
# A 4002 command (save, id 42), which the flight computer logs beside them.
C1 = bytes.fromhex("D3000EFA26098030A3ECEAC66D01D512A0CA9F10")


def test_the_goldens_are_good_frames():
    for f in (A2, A3, C1):
        assert ext.frame_ok(f)
    assert ext.message_type(A2) == 4001
    assert ext.message_type(C1) == 4002


def test_decodes_the_boards_report():
    a = ext.decode_align(A2)
    assert a is not None
    assert (a["have"], a["n_pose"], a["cal_axes"], a["saved"], a["from"]) == (2, 6, 7, 2, 1)
    assert a["spread_deg"] == 90.0
    assert abs(a["resid_deg"] - 0.14) < 1e-12
    assert abs(a["q"][0] - 0.7071) < 1e-3 and abs(a["q"][3] - 0.7071) < 1e-3
    assert abs(a["scale"][0] - 1.012) < 1e-9
    assert abs(a["bias"][1] + 0.271) < 1e-9
    assert a["poses"][5][1] == 255
    assert a["poses"][1][0][0] == -1.0
    assert ext.decode_align(C1) is None
    assert ext.decode_align(A2[:-8]) is None   # cut short of its poses (and its CRC)


def test_the_last_report_in_one_line(tmp_path, capsys):
    log = build_log(tmp_path, [A3, C1, A2])
    assert ext.main([str(log)]) == 0
    out = capsys.readouterr().out
    assert ("the board's IMU mounting (2 reports; the last): whole, 6 poses 90 deg apart, the worst 0.14 deg off; "
            "accelerometer axes calibrated: xyz; saved, but another in use, in use from this boot's rests") in out
    assert "1 command" in out
    a3 = ext.decode_align(A3)
    assert ext.align_line(a3) == ("tilt only (one pose); accelerometer axes calibrated: none; saved, "
                                  "in use from the saved one with this boot's rests; "
                                  "the rail pose 1.24 deg off the saved rotation")
