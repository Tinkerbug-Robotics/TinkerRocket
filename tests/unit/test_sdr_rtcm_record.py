"""The gnss-sdr-dev board's RTCM3 in a flight log (GNSS_RTCM_MSG, 0x95): the extractor.

A V9 or V10 with TinkerRocket's gnss-sdr-dev board on J1 logs every frame the
board sends, as the mini logs its LC86G's: MSM7 1077/1097/1127 at 100 Hz,
TinkerRocket's message 4001 after each epoch and each fix, and the board's
ephemerides. The .bin is synthesised with the out computer's framing and the
FC's split rule (test_gnss_rtcm_record.py has both) from the board's own
recorded stream, tests_cpp/fixtures/sdr_rtcm_stream.h: one fix, its
ephemerides and ten epochs. When RTKLIB's convbin is on this machine
(ModelRockets/bench-backups/rtklib-builds/v2.5.1/, or $TR_CONVBIN) the
extracted .rtcm3 also goes through it into RINEX.
"""

from __future__ import annotations

import datetime
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))
if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import extract_gnss_rtcm as ext              # noqa: E402
from test_gnss_rtcm_record import records_for   # noqa: E402  the FC's split, the OC's framing

FIXTURE = REPO_ROOT / "tests_cpp" / "fixtures" / "sdr_rtcm_stream.h"
WEEK, FIRST_TOW_MS = 2443, 156619330


def stream() -> bytes:
    text = FIXTURE.read_text()
    body = text[text.index("kSdrStream["):]
    return bytes(int(h, 16) for h in re.findall(r"0x([0-9A-F]{2})", body[body.index("{"):]))


def frames_of(s: bytes) -> list[bytes]:
    out, i = [], 0
    while i < len(s):
        n = ((s[i + 1] & 3) << 8 | s[i + 2]) + 6
        out.append(s[i:i + n])
        i += n
    return out


def build_log(tmp_path: Path, frames: list[bytes]) -> Path:
    # Each frame stamped as the FC stamps it: when its last byte arrived, at 2 Mbaud.
    out, t = [], 5_000_000
    for f in frames:
        t += len(f) * 5
        out += records_for(f, t)
    p = tmp_path / "flight.bin"
    p.write_bytes(b"".join(out))
    return p


def test_the_fixture_is_the_boards_stream():
    fr = frames_of(stream())
    assert len(fr) == 45
    assert all(ext.frame_ok(f) for f in fr)
    assert [ext.message_type(f) for f in fr[:5]] == [4001, 1019, 1046, 1046, 1046]
    assert [ext.message_type(f) for f in fr[5:9]] == [1077, 1097, 1127, 4001]


def test_extractor_gives_back_the_boards_stream(tmp_path, capsys):
    s = stream()
    log = build_log(tmp_path, frames_of(s))
    assert ext.main([str(log)]) == 0
    assert (tmp_path / "flight.rtcm3").read_bytes() == s
    out = capsys.readouterr().out
    assert "0 incomplete, 0 bad CRC" in out
    assert "4001 x11" in out and "1077 x10" in out and "1127 x10" in out
    assert "MSM7 epochs: 10 at a 10 ms step (100.0 Hz), 0 missing, 0 out of order" in out
    assert "TinkerRocket 4001 (gnss-sdr-dev): 10 epochs, 1 fixes (1 valid)" in out
    assert "Galileo" in out                       # its ephemerides named, as an LC86G's are


def test_a_lost_epoch_is_counted(tmp_path):
    fr = frames_of(stream())
    kept = fr[:9] + fr[13:]                      # the second epoch's four frames lost
    frames, _ = ext.extract(build_log(tmp_path, kept))
    es = ext.epoch_summary(frames)
    assert es["epochs"] == 9 and es["step_ms"] == 10 and es["missing"] == 1
    assert es["tr_epochs"] == 9


def test_beidou_is_on_gps_time():
    fr = frames_of(stream())
    assert [ext.msm_gps_tow(f) for f in fr[5:8]] == [FIRST_TOW_MS] * 3
    assert ext.msm_gps_tow(fr[0]) is None         # a 4001
    assert ext.msm_gps_tow(fr[1]) is None         # an ephemeris


def convbin() -> Path | None:
    env = os.environ.get("TR_CONVBIN")
    if env:
        return Path(env)
    p = REPO_ROOT.parent / "bench-backups" / "rtklib-builds" / "v2.5.1" / "convbin"
    return p if p.exists() else shutil.which("convbin") and Path(shutil.which("convbin"))


@pytest.mark.skipif(convbin() is None, reason="RTKLIB's convbin is not on this machine")
def test_rtklib_reads_the_extracted_stream(tmp_path):
    log = build_log(tmp_path, frames_of(stream()))
    assert ext.main([str(log)]) == 0
    t = datetime.datetime(1980, 1, 6) + datetime.timedelta(weeks=WEEK, milliseconds=FIRST_TOW_MS)
    obs, nav = tmp_path / "flight.obs", tmp_path / "flight.nav"
    subprocess.run([str(convbin()), "-r", "rtcm3", "-tr", t.strftime("%Y/%m/%d"), t.strftime("%H:%M:%S"),
                    "-v", "3.04", "-od", "-os", "-o", str(obs), "-n", str(nav), str(tmp_path / "flight.rtcm3")],
                   check=True, capture_output=True)
    epochs = [line for line in obs.read_text().splitlines() if line.startswith("> ")]
    assert len(epochs) == 10
    secs = [float(e.split()[6]) for e in epochs]
    assert all(abs((b - a) - 0.01) < 1e-6 for a, b in zip(secs, secs[1:]))
    assert all(int(e.split()[8]) == 18 for e in epochs)   # every observable of every epoch
    assert "G" in obs.read_text() and "C" in obs.read_text() and "E" in obs.read_text()
    assert nav.stat().st_size > 0
