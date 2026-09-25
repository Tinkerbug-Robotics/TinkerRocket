"""The deployment-detector replay builds, and its board pick changes nothing.

Data_Analysis/replay_deployment_detector.py compiles the REAL detector and the
flight computer's config.h into a ctypes shim.  #808 made config.h refuse to
compile without a board revision -- deliberately, since a wrong pyro map is
silent -- and from then on the tool exited 1 before replaying anything.
Nothing ran it, so nobody noticed for five weeks.

The tool now passes V9's flag.  That is only safe while every value the shim
reads is the same on every board: an M1 log replayed against a V9 threshold
that differs from M1's would disagree with the vehicle with no symptom, which
is exactly the drift the tool exists to prevent.  So the second test builds
the shim for every board config.h accepts and compares the shipped values bit
for bit.  If it fails, do not just change the pick -- replay each log with the
board that flew it.
"""
from __future__ import annotations

import ctypes
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))

import replay_deployment_detector as rdd  # noqa: E402

# One entry per board_<rev>.h under flight_computer/main/board/ -- legacy/
# included, since config.h still builds V7 and V8 from there -- with the defines
# the FC's CMake passes for it.  M1 needs its two driver seams as well, or
# config.h refuses the build (flight_computer/CMakeLists.txt sets both under
# TR_BOARD_M1).
BOARDS = {
    "v7": ("-DTR_BOARD_V7=1",),
    "v8": ("-DTR_BOARD_V8=1",),
    "v9": ("-DTR_BOARD_V9=1",),
    "m1": ("-DTR_BOARD_M1=1", "-DTR_MAG_DRIVER_QMC5883P=1", "-DTR_GNSS_DRIVER_LC86=1"),
}


def _shipped(so: Path) -> bytes:
    """tr_deploy_shipped_cfg's float and unsigned banks, raw, for a bit-exact compare."""
    lib = ctypes.CDLL(str(so))
    lib.tr_deploy_shipped_cfg.argtypes = [
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_uint)]
    f = (ctypes.c_float * 4)()
    u = (ctypes.c_uint * 5)()
    lib.tr_deploy_shipped_cfg(f, u)
    return bytes(f) + bytes(u)


def test_replay_tool_builds_and_loads_its_shim():
    det = rdd.Detector()
    # build_ticks divides by it.
    assert det.loop_hz > 0


def test_every_board_ships_the_values_the_replay_uses(tmp_path):
    board_dir = rdd._FC_MAIN_DIR / "board"
    headers = {p.stem.removeprefix("board_") for p in board_dir.rglob("board_*.h")}
    assert headers == set(BOARDS), (
        f"{board_dir} has {sorted(headers)} but this test covers {sorted(BOARDS)}: "
        "give every board header an entry in BOARDS")

    want = _shipped(rdd._build_shim())
    for board, defines in BOARDS.items():
        out = tmp_path / f"shim_{board}.so"
        proc = subprocess.run(rdd._shim_cmd(out, defines), capture_output=True, text=True)
        assert proc.returncode == 0, f"{board} shim failed to build:\n{proc.stderr}"
        assert _shipped(out) == want, (
            f"config.h gives {board} different DEPLOY_* / FLIGHT_LOOP_UPDATE_RATE "
            f"values from the replay tool's pick ({' '.join(rdd._BOARD_DEFINES)}); "
            "replay each log with the board that flew it rather than one fixed board")
