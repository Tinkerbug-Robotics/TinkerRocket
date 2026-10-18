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


# ---------- the high-g, aligned to the low-g (RocketComputerTypes.h, HG_CAL_*) ----------
# A board's two accelerometers as real ones differ: reading = S a + offset (board frame), the high-g with a full
# 3x3 S (scale and cross-axis), the low-g with a scale per axis. The six faces' fit must find l = M h - b with
# M = S_lg S_hg^-1 and b = M c_hg - o_lg, whatever the faces' tilts. The apps' fits (AccelSixFace.swift / .kt) pin
# the same cases.

S_LG = [1.0006, 0.9991, 1.0004]
O_LG = [-0.0155, -0.1324, -0.0306]
S_HG = [[0.9921, -0.0047, 0.0030], [0.0029, 1.0068, -0.0054], [-0.0016, 0.0051, 0.9880]]
C_HG = [0.98, -1.47, 0.62]           # the high-g's own zero-g, m/s^2 (100-150 mg, as on the V9)
SD_LG, SD_HG, N = 0.019, 0.56, 11340   # one 3 s window at 3780 Hz on the V9 (2026-10-17 bench log): 1.9 and 57 mg


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def inv3(a):
    m = acs.inverse(a)
    return m


def truth_alignment(s_lg=S_LG, s_hg=S_HG, o_lg=O_LG, c_hg=C_HG):
    M = matmul([[s_lg[i] if i == j else 0.0 for j in range(3)] for i in range(3)], inv3(s_hg))
    b = [sum(M[i][j] * c_hg[j] for j in range(3)) - o_lg[i] for i in range(3)]
    return [x for row in M for x in row], b


def board_faces(tilts, rng, s_lg=S_LG, s_hg=S_HG, o_lg=O_LG, c_hg=C_HG, noise=0.0, quad=(0.0, 0.0, 0.0)):
    """Six faces' windows: the true specific force a (one g, tilted), each channel's mean of it, with Gaussian
    noise of SD / sqrt(N) when `noise` (a multiplier), and the high-g's even-order nonlinearity quad_i * a_i^2."""
    out = {}
    for (ax, sg), t in tilts.items():
        a = [0.0, 0.0, 0.0]
        a[ax] = sg * G * math.cos(math.radians(t))
        other = (ax + rng.choice((1, 2))) % 3
        a[other] += rng.choice((1, -1)) * G * math.sin(math.radians(t))
        l = [s_lg[i] * a[i] + o_lg[i] for i in range(3)]
        h = [sum(s_hg[i][j] * a[j] for j in range(3)) + c_hg[i] + quad[i] * a[i] ** 2 for i in range(3)]
        if noise:
            l = [x + rng.gauss(0, noise * SD_LG / math.sqrt(N)) for x in l]
            h = [x + rng.gauss(0, noise * SD_HG / math.sqrt(N)) for x in h]
        out[(ax, sg)] = {"mean": l, "sd": [SD_LG] * 3, "n": N, "hg": {"mean": h, "sd": [SD_HG] * 3}}
    return out


def both_fits(fc):
    off, scl, conv = acs.solve(fc, G)
    assert conv
    return off, scl, acs.solve_high_g(fc, off, scl, G)


def test_the_alignment_recovers_scale_cross_axis_and_offset_whatever_the_tilts():
    rng = random.Random(5)
    M_true, b_true = truth_alignment()
    for tilts in (USER_TILTS, {k: 0.0 for k in USER_TILTS}):
        _, _, hg = both_fits(board_faces(tilts, rng))
        assert hg["problems"] == [] and hg["disagree"] == []
        assert max(abs(hg["matrix"][k] - M_true[k]) for k in range(9)) < 1e-9
        assert max(abs(hg["bias"][i] - b_true[i]) for i in range(3)) < 1e-9
        assert hg["max"] < 1e-9
        # and its own gravity fit finds the high-g's zero-g where the alignment says it is (to 0.1 mg: the
        # sphere fit has no cross-axis terms)
        assert max(abs(hg["own"]["offset"][i] - C_HG[i]) for i in range(3)) < 0.2e-3 * G
        assert max(abs(hg["implied_zero"][i] - C_HG[i]) for i in range(3)) < 1e-9


def test_with_bench_noise_the_errors_match_the_reported_sigma():
    rng = random.Random(1629)
    M_true, b_true = truth_alignment()
    z_m, z_b, false_alarms, trials = [], [], 0, 400
    for _ in range(trials):
        _, _, hg = both_fits(board_faces(USER_TILTS, rng, noise=1.0))
        assert hg["problems"] == []
        z_m += [(hg["matrix"][k] - M_true[k]) / hg["sigma_matrix"][k] for k in range(9)]
        z_b += [(hg["bias"][i] - b_true[i]) / hg["sigma_bias"][i] for i in range(3)]
        false_alarms += bool(hg["disagree"])
    rms = lambda z: math.sqrt(sum(x * x for x in z) / len(z))
    # the reported sigma carries the noise floor (drift, slow noise) on top of SD / sqrt(n): honest or cautious
    assert 0.3 < rms(z_m) < 1.1 and 0.3 < rms(z_b) < 1.1
    assert false_alarms / trials < 0.02
    # what a 3 s face buys: the scale to a few hundredths of a percent, the offset to a few tenths of a mg
    _, _, hg = both_fits(board_faces(USER_TILTS, rng, noise=1.0))
    assert max(hg["sigma_matrix"][k] for k in (0, 4, 8)) < 5e-4
    assert max(hg["sigma_bias"]) < 0.5e-3 * G


def test_a_disagreeing_high_g_is_flagged_and_the_low_g_alignment_still_saved():
    # an even-order nonlinearity of 4 mg at one g on the high-g's Z (its datasheet allows 2 % of full scale):
    # its own gravity fit takes it all as offset, the alignment a third of it, and the faces' residual stays
    # under the limit
    rng = random.Random(7)
    q = 4e-3 * G / G ** 2
    _, _, hg = both_fits(board_faces(USER_TILTS, rng, quad=(0.0, 0.0, q)))
    assert hg["disagree"] == [2]
    assert abs(hg["d_offset"][2]) > 2e-3 * G
    assert hg["problems"] == []                      # the alignment to the low-g is what is saved
    # a worse one leaves a face further from the low-g than the limit: not saved
    _, _, hg = both_fits(board_faces(USER_TILTS, rng, quad=(0.0, 0.0, 1.5 * q)))
    assert "residual" in hg["problems"]


def test_the_high_g_refusals():
    rng = random.Random(9)
    s = [[1.045, 0, 0], [0, 1, 0], [0, 0, 1]]        # a ratio 4.5 % off: no ISM6 is trimmed that badly
    _, _, hg = both_fits(board_faces(USER_TILTS, rng, s_hg=s))
    assert hg["problems"] == ["scale"]
    s = [[1, 0.05, 0], [0, 1, 0], [0, 0, 1]]
    _, _, hg = both_fits(board_faces(USER_TILTS, rng, s_hg=s))
    assert hg["problems"] == ["cross"]
    _, _, hg = both_fits(board_faces(USER_TILTS, rng, c_hg=[12.0, 0.0, 0.0]))
    assert hg["problems"] == ["bias"]
    fc = board_faces(USER_TILTS, rng)
    fc[(1, -1)]["hg"] = None                         # an older flight or out computer: no high-g in its frames
    off, scl, conv = acs.solve(fc, G)
    assert acs.solve_high_g(fc, off, scl, G)["problems"] == ["no_high_g"]


def test_the_status_frame_with_and_without_the_high_g():
    fixtures = REPO_ROOT / "tests_cpp" / "fixtures" / "wire" / "fileops"
    old = acs.parse((fixtures / "accelcal_0xD1_40.bin").read_bytes())
    assert old is not None and old["hg"] is None
    new = acs.parse((fixtures / "accelcal_0xD1_114.bin").read_bytes())
    assert new["n"] == old["n"] and new["offset"] == old["offset"]
    h = new["hg"]
    assert h["flags"] == acs.HGF_STORED | acs.HGF_APPLIED | acs.HGF_BIAS and h["bias_src"] == 2
    assert abs(h["matrix"][0] - 1.0081) < 1e-6 and abs(h["bias"][1] - 1.48) < 1e-6
    import struct
    payload = struct.pack(acs.FMT_HG_APPLY, acs.HG_OP_STORE, *([1.0] * 9), 0.1, 0.2, 0.3)
    assert len(payload) == 49
