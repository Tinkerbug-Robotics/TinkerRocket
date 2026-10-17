"""tools/accel_cal_six_face.py's solve (#1629): a sphere fit -- the offset and a per-axis scale such that all six
faces' corrected readings are one g long -- so faces a few degrees off square, tilted unequally between the up and
down face of an axis, do not bias the offset. The apps' solvers (AccelSixFace.swift / AccelSixFace.kt) are the same
fit and pin the same cases."""

from __future__ import annotations

import importlib.util
import math
import random
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
_spec = importlib.util.spec_from_file_location("accel_cal_six_face", REPO_ROOT / "tools" / "accel_cal_six_face.py")
acs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(acs)

G = 9.80665
# the user's first run on the Beetle (2026-10-17): each face's tilt off its axis, deg
USER_TILTS = {(0, 1): 9.1, (0, -1): 5.0, (1, 1): 6.9, (1, -1): 4.5, (2, 1): 3.7, (2, -1): 9.0}


def faces(offset, scale, tilts, rng):
    out = {}
    for (ax, sg), t in tilts.items():
        f = [0.0, 0.0, 0.0]
        f[ax] = sg * G * math.cos(math.radians(t))
        other = (ax + rng.choice((1, 2))) % 3
        f[other] += rng.choice((1, -1)) * G * math.sin(math.radians(t))
        out[(ax, sg)] = {"mean": [scale[i] * f[i] + offset[i] for i in range(3)]}
    return out


def per_axis(fc):
    return [0.5 * (fc[(a, 1)]["mean"][a] + fc[(a, -1)]["mean"][a]) for a in range(3)]


def test_the_fit_recovers_the_offset_whatever_the_faces_tilt():
    rng = random.Random(11)
    worst_fit = worst_avg = 0.0
    for _ in range(200):
        b = [rng.gauss(0, 15e-3 * G) for _ in range(3)]
        s = [1 + rng.gauss(0, 0.004) for _ in range(3)]
        fc = faces(b, s, USER_TILTS, rng)
        off, scl, converged = acs.solve(fc, G)
        assert converged
        worst_fit = max(worst_fit, max(abs(off[i] - b[i]) for i in range(3)))
        worst_avg = max(worst_avg, max(abs(per_axis(fc)[i] - b[i]) for i in range(3)))
        assert all(abs(scl[i] - s[i]) < 1e-6 for i in range(3))
    assert worst_fit < 1.5e-3 * G            # under 1.5 mg (noise-free: essentially exact)
    assert worst_avg > 1.5e-3 * G            # the per-axis average alone is not: the reason for the fit


def test_square_faces_and_a_scale_out_of_range():
    rng = random.Random(3)
    b, s = [0.30, -0.45, 0.12], [1.0, 1.0, 1.0]
    off, scl, converged = acs.solve(faces(b, s, {k: 0.0 for k in USER_TILTS}, rng), G)
    assert converged and all(abs(off[i] - b[i]) < 1e-6 for i in range(3))
    off, scl, converged = acs.solve(faces(b, [1.06, 1.0, 1.0], USER_TILTS, rng), G)
    assert converged and not 0.97 <= scl[0] <= 1.03   # the script refuses to write it (outside 0.97-1.03)
