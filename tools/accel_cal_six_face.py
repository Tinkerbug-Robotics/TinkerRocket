#!/usr/bin/env python3
"""The low-g accelerometer's six-face hand calibration (#1629), over BLE from a Mac.

On a still pad the navigation filter levels on gravity, and an accelerometer offset there is indistinguishable
from tilt: the pad calibration's one still window cannot separate them, and with GNSS correcting all three axes a
false tilt becomes a heading error in the boost. Turn each board axis straight up and then straight down: gravity
flips sign on that axis and the offset does not. The six faces are solved together as a sphere fit -- the offset and a
per-axis scale such that every corrected reading is one g long -- so a face a few degrees off square does not bias
the offset (the per-axis average (up + down) / 2 is the fit's starting point; with faces 4-9 deg off square and
unequal it alone is up to ~6 mg off, the fit ~1 mg). The flight computer
stores the offset in NVS ("acc_cal") and its converter takes it off the low-g and high-g readings, in the board
frame (+X forward, +Y left, +Z up -- before the board->rocket mounting, so a remount keeps it).

Each face: put the board (or the rocket) down on a new side, still, and press Enter. The flight computer averages
the low-g accelerometer for 3 s (BLE 78) and the script works out which face it was from the reading itself -- just
cover all six. Then it solves the offset, shows it, and on your yes writes it (BLE 76) and reads it back (BLE 77).

The same windows average the high-g accelerometer too (a flight and out computer that send it: the status frame's
AccelCalHighGStatus). Its correction is fitted to the low-g, the accurate channel on the bench: the matrix M (scale and
cross-axis) and offset b with l = M h - b on every face, least squares, l the raw low-g and h the raw high-g -- so the
high-g takes over from the low-g at its rail with no step (RocketComputerTypes.h, HG_CAL_*). The high-g's own gravity
fit (the low-g's sphere fit, on the high-g) is shown beside it as a check; where the two disagree beyond their noise,
the alignment to the low-g is what is saved. On a yes it writes the alignment too (BLE 80) and reads it back (77).

Disconnect the phone app first: one BLE central at a time, and the app's connect-time sync would queue its own
commands. Run from the Terminal panel (BLE).

    accel_cal_six_face.py                 # the six faces, then write on a yes
    accel_cal_six_face.py --read          # what the flight computer has stored
    accel_cal_six_face.py --clear         # remove the stored offset (on a yes)
    accel_cal_six_face.py --check         # after a save: any attitude, the corrected high-g against the low-g
    accel_cal_six_face.py --name "Mini V1" --lat 39.47 --dry-run
"""
import argparse
import asyncio
import math
import struct
import sys

try:
    from bleak import BleakClient, BleakScanner
except ImportError:          # the solve alone (tests) needs no BLE
    BleakClient = BleakScanner = None

SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
COMMAND_UUID = "cba1d466-344c-4be3-ab3f-189f80dd7518"
FILE_OPS_UUID = "8d53dc1d-1db7-4cd3-868b-8a527460aa84"
CMD_APPLY, CMD_READ, CMD_FACE, CMD_HG_APPLY = 76, 77, 78, 80
MARKER = 0xD1
FMT = "<BB3f3f3fH"                  # AccelCalStatusData, 40 bytes
FMT_HG = "<3f3fBB9f3f"              # AccelCalHighGStatus, 74 bytes, after it from a flight computer that sends it
FMT_HG_APPLY = "<B9f3f"             # HighGCalApplyData, 49 bytes
ACCF_STORED, ACCF_APPLIED = 0x01, 0x02
HGF_STORED, HGF_APPLIED, HGF_BIAS = 0x01, 0x02, 0x04
HG_OP_CLEAR, HG_OP_STORE = 0, 1
HG_BIAS_SOURCES = {0: "none recorded", 1: "the pad calibration", 2: "the six faces"}
IDENTITY = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
FACE_STATES = {0: "idle", 1: "measuring", 2: "done", 3: "FAILED"}
G0 = 9.80665
OFFSET_MAX = 1.0                    # m/s^2: the flight computer refuses larger (ACC_CAL_OFFSET_MAX_MPS2)
TILT_MAX_DEG = 10.0                 # a face whose axis is further off vertical is measured again
# The high-g's alignment: what the flight computer refuses (HG_CAL_SCALE_MIN..MAX, HG_CAL_CROSS_MAX,
# HG_CAL_BIAS_MAX_MPS2), and what this fit refuses on top: a face it fits worse than HG_RESIDUAL_MAX.
HG_SCALE_MIN, HG_SCALE_MAX, HG_CROSS_MAX, HG_BIAS_MAX = 0.97, 1.03, 0.03, 11.0
HG_RESIDUAL_MAX = 0.03              # m/s^2 (~3 mg): ~7x the residual of a still 3 s face (~0.3 mg rms)
# Each face mean's error beyond its samples' SD / sqrt(n): the offset's drift while the board is handled
# (~2 mg/degC) and the low-g's slow noise; and the floors under the two fits' agreement test.
HG_NOISE_FLOOR = 0.002              # m/s^2 (0.2 mg)
HG_AGREE_FLOOR_OFFSET = 0.005       # m/s^2 (0.5 mg)
HG_AGREE_FLOOR_SCALE = 2e-4         # 0.02 %
HG_AGREE_SIGMAS = 3.0
AXES = "XYZ"
WORDS = {0: "the board's +X (forward) end", 1: "the board's +Y (left) side", 2: "the board's +Z (component) side"}


def face_name(ax, sgn):
    return f"+{AXES[ax]} {'UP' if sgn > 0 else 'DOWN'}"


def local_g(lat_deg, h_m=0.0):
    """Normal gravity (WGS84, Somigliana) at a latitude and height, m/s^2."""
    s2 = math.sin(math.radians(lat_deg)) ** 2
    return 9.7803253359 * (1 + 0.00193185265241 * s2) / math.sqrt(1 - 0.00669437999013 * s2) - 3.086e-6 * h_m


def parse(data):
    """A 0xD1 frame -> its fields; "hg" is None when it carries no high-g (an older flight or out computer)."""
    if len(data) < 1 + struct.calcsize(FMT) or data[0] != MARKER:
        return None
    f = struct.unpack_from(FMT, data, 1)
    st = {"flags": f[0], "face_state": f[1], "offset": f[2:5], "mean": f[5:8], "sd": f[8:11], "n": f[11], "hg": None}
    at = 1 + struct.calcsize(FMT)
    if len(data) >= at + struct.calcsize(FMT_HG):
        h = struct.unpack_from(FMT_HG, data, at)
        st["hg"] = {"mean": h[0:3], "sd": h[3:6], "flags": h[6], "bias_src": h[7], "matrix": h[8:17],
                    "bias": h[17:20]}
    return st


def show_status(st):
    o = st["offset"]
    stored = bool(st["flags"] & ACCF_STORED)
    applied = bool(st["flags"] & ACCF_APPLIED)
    print(f"  stored offset: {'yes' if stored else 'none'}"
          + (f"  ({o[0]:+.4f}, {o[1]:+.4f}, {o[2]:+.4f}) m/s^2 = "
             f"({o[0] / G0 * 1e3:+.1f}, {o[1] / G0 * 1e3:+.1f}, {o[2] / G0 * 1e3:+.1f}) mg" if stored else "")
          + f";  applied by the converter: {'yes' if applied else 'no'}")
    h = st.get("hg")
    if h is None:
        print("  high-g: not in this frame -- the flight or out computer firmware predates its calibration")
        return
    m = h["matrix"]
    print(f"  high-g alignment: {'stored' if h['flags'] & HGF_STORED else 'none'}"
          f", applied: {'yes' if h['flags'] & HGF_APPLIED else 'no'}"
          + (f"  (diag {m[0]:.5f} {m[4]:.5f} {m[8]:.5f}, cross-axis max {max(abs(m[i]) for i in (1, 2, 3, 5, 6, 7)):.5f})"
             if h["flags"] & HGF_STORED else ""))
    b = h["bias"]
    print(f"  high-g bias in use: ({b[0]:+.4f}, {b[1]:+.4f}, {b[2]:+.4f}) m/s^2, from "
          f"{HG_BIAS_SOURCES.get(h['bias_src'], str(h['bias_src']))}" if h["flags"] & HGF_BIAS else
          "  high-g bias: none stored")


class Link:
    def __init__(self, client):
        self.c = client
        self.q = asyncio.Queue()

    def on_ops(self, _h, data):
        st = parse(bytes(data))
        if st is not None:
            self.q.put_nowait(st)

    async def drain(self):
        while not self.q.empty():
            self.q.get_nowait()

    async def send(self, cmd, payload=b""):
        await self.c.write_gatt_char(COMMAND_UUID, bytes([cmd]) + payload, response=True)

    async def wait(self, pred, timeout):
        loop = asyncio.get_running_loop()
        end = loop.time() + timeout
        while True:
            left = end - loop.time()
            if left <= 0:
                return None
            try:
                st = await asyncio.wait_for(self.q.get(), timeout=left)
            except asyncio.TimeoutError:
                return None
            if pred(st):
                return st


async def find(name):
    print(f"scanning for {name!r} (15 s) ...")
    found = await BleakScanner.discover(timeout=15.0, return_adv=True)
    for d, ad in found.values():
        n = ad.local_name or d.name or ""
        has_svc = SERVICE_UUID.lower() in [u.lower() for u in (ad.service_uuids or [])]
        if n == name or (has_svc and name.lower() in n.lower()):
            print(f"found {n!r} (rssi {ad.rssi})")
            return d
    return None


async def ask(prompt):
    return await asyncio.get_running_loop().run_in_executor(None, input, prompt)


async def read_status(link):
    await link.drain()
    await link.send(CMD_READ)
    return await link.wait(lambda s: True, 6.0)


async def measure_face(link):
    """One face window: the reading's mean, SD and count, or None (no answer)."""
    await link.drain()
    await link.send(CMD_FACE)
    st = await link.wait(lambda s: s["face_state"] in (2, 3), 10.0)
    return st


def which_face(mean):
    """The face a still reading is: the axis nearest vertical, and whether it points up (+g on it) or down."""
    ax = max(range(3), key=lambda i: abs(mean[i]))
    sgn = 1 if mean[ax] > 0 else -1
    n = math.sqrt(sum(m * m for m in mean))
    tilt = math.degrees(math.acos(min(1.0, abs(mean[ax]) / n))) if n > 0 else 90.0
    return ax, sgn, tilt


def solve_6x6(A, y):
    """A x = y by Gauss-Jordan with partial pivoting (6 x 6); None if singular."""
    n = len(y)
    M = [list(A[i]) + [y[i]] for i in range(n)]
    for c in range(n):
        piv = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[piv][c]) < 1e-15:
            return None
        M[c], M[piv] = M[piv], M[c]
        for r in range(n):
            if r != c:
                f = M[r][c] / M[c][c]
                M[r] = [M[r][j] - f * M[c][j] for j in range(n + 1)]
    return [M[i][n] / M[i][i] for i in range(n)]


def solve(faces, g):
    """The six faces' means -> offset (3), scale (3), converged. A sphere fit: Gauss-Newton on
    |k * (m - b)| = g over the six faces, b the offset and k = 1 / scale per axis, from the per-axis average."""
    b = [0.5 * (faces[(ax, 1)]["mean"][ax] + faces[(ax, -1)]["mean"][ax]) for ax in range(3)]
    k = [1.0, 1.0, 1.0]
    means = [list(f["mean"]) for f in faces.values()]
    converged = False
    for _ in range(20):
        J, r = [], []
        for m in means:
            v = [k[i] * (m[i] - b[i]) for i in range(3)]
            n = math.sqrt(sum(x * x for x in v))
            r.append(n - g)
            J.append([-k[i] * v[i] / n for i in range(3)] + [(m[i] - b[i]) * v[i] / n for i in range(3)])
        # normal equations (J is 6 x 6 here; this also serves a non-square J)
        JtJ = [[sum(J[a][i] * J[a][j] for a in range(len(J))) for j in range(6)] for i in range(6)]
        Jtr = [-sum(J[a][i] * r[a] for a in range(len(J))) for i in range(6)]
        dp = solve_6x6(JtJ, Jtr)
        if dp is None:
            break
        b = [b[i] + dp[i] for i in range(3)]
        k = [k[i] + dp[3 + i] for i in range(3)]
        if max(abs(x) for x in dp) < 1e-9:
            res = max(abs(math.sqrt(sum((k[i] * (m[i] - b[i])) ** 2 for i in range(3))) - g) for m in means)
            converged = res < 1e-6
            break
    scl = [1.0 / x if x else float("inf") for x in k]
    return b, scl, converged


def mat_vec(m, v):
    return [m[3 * i] * v[0] + m[3 * i + 1] * v[1] + m[3 * i + 2] * v[2] for i in range(3)]


def inverse(a):
    """The inverse of a small square matrix (rows), or None if singular."""
    n = len(a)
    cols = [solve_6x6(a, [1.0 if r == c else 0.0 for r in range(n)]) for c in range(n)]
    if any(c is None for c in cols):
        return None
    return [[cols[c][r] for c in range(n)] for r in range(n)]


def solve_high_g(faces, lg_offset, lg_scale, g):
    """The high-g's alignment to the low-g from the six faces, and its own gravity fit as a check.

    faces: {(axis, sign): {"mean": l, "sd": ..., "n": ..., "hg": {"mean": h, "sd": ...}}}, the raw board-frame
    readings (m/s^2) of each face's window; lg_offset / lg_scale: the low-g's sphere fit (solve()).

    The alignment: l = M h - b on every face, least squares, one row of M (and of b) at a time --
    [h_x h_y h_z -1] (M_i, b_i) = l_i over the six faces: four unknowns, six equations. Its 1-sigma comes from
    the faces' own noise (each mean's SD / sqrt(n), the high-g's through M, and HG_NOISE_FLOOR), not the
    residuals (two degrees of freedom a row are too few to judge it by).

    Returns a dict (see the keys below); "problems" lists why it may not be saved, empty when it may. The two
    fits disagreeing is not one: the alignment to the low-g is what is saved either way.
    """
    order = [(ax, sg) for ax in range(3) for sg in (1, -1)]
    if any(faces.get(k) is None or faces[k].get("hg") is None for k in order):
        return {"problems": ["no_high_g"]}
    L = [list(faces[k]["mean"]) for k in order]
    H = [list(faces[k]["hg"]["mean"]) for k in order]
    X = [h + [-1.0] for h in H]
    XtX = [[sum(X[f][i] * X[f][j] for f in range(6)) for j in range(4)] for i in range(4)]
    XtXi = inverse(XtX)
    if XtXi is None:
        return {"problems": ["fit"]}
    M, b = [0.0] * 9, [0.0] * 3
    for i in range(3):
        rhs = [sum(X[f][j] * L[f][i] for f in range(6)) for j in range(4)]
        th = [sum(XtXi[j][k] * rhs[k] for k in range(4)) for j in range(4)]
        M[3 * i:3 * i + 3] = th[0:3]
        b[i] = th[3]
    if not all(math.isfinite(v) for v in M + b):
        return {"problems": ["fit"]}
    res = [[mat_vec(M, H[f])[i] - b[i] - L[f][i] for i in range(3)] for f in range(6)]
    flat = [r for row in res for r in row]
    rms = math.sqrt(sum(r * r for r in flat) / len(flat))
    worst = max(abs(r) for r in flat)
    # each row's noise: the mean over the faces of every reading's variance in that row's equation
    sig_row = []
    for i in range(3):
        v = 0.0
        for k in order:
            n = max(int(faces[k].get("n", 0)), 1)
            sl = faces[k].get("sd", (0.0, 0.0, 0.0))
            sh = faces[k]["hg"].get("sd", (0.0, 0.0, 0.0))
            v += (sl[i] ** 2 + sum((M[3 * i + j] * sh[j]) ** 2 for j in range(3))) / n + HG_NOISE_FLOOR ** 2
        sig_row.append(math.sqrt(v / 6))
    sig_M = [sig_row[i] * math.sqrt(XtXi[j][j]) for i in range(3) for j in range(3)]
    sig_b = [sig_row[i] * math.sqrt(XtXi[3][3]) for i in range(3)]

    # the high-g's own gravity fit, and what the alignment says it should have found
    own = {k: {"mean": faces[k]["hg"]["mean"]} for k in order}
    own_off, own_scl, own_conv = solve(own, g)
    Minv = inverse([M[0:3], M[3:6], M[6:9]])
    if Minv is None:
        return {"problems": ["fit"]}
    zero = mat_vec([x for row in Minv for x in row], [b[i] + lg_offset[i] for i in range(3)])
    d_off = [zero[i] - own_off[i] for i in range(3)]
    d_scl = [M[4 * i] - lg_scale[i] / own_scl[i] for i in range(3)]
    sig_d_off, sig_d_scl, disagree = [], [], []
    for i in range(3):
        n_up, n_dn = (max(int(faces[(i, s)].get("n", 0)), 1) for s in (1, -1))
        s_up, s_dn = faces[(i, 1)]["hg"].get("sd", (0, 0, 0))[i], faces[(i, -1)]["hg"].get("sd", (0, 0, 0))[i]
        own_mean_var = s_up ** 2 / n_up + s_dn ** 2 / n_dn + 2 * HG_NOISE_FLOOR ** 2
        sig_d_off.append(math.sqrt(own_mean_var / 4 + sig_b[i] ** 2 + HG_AGREE_FLOOR_OFFSET ** 2))
        sig_d_scl.append(math.sqrt(own_mean_var / (4 * g * g) + sig_M[4 * i] ** 2 + HG_AGREE_FLOOR_SCALE ** 2))
        if not (abs(d_off[i]) <= HG_AGREE_SIGMAS * sig_d_off[i] and abs(d_scl[i]) <= HG_AGREE_SIGMAS * sig_d_scl[i]):
            disagree.append(i)

    problems = []
    if worst > HG_RESIDUAL_MAX:
        problems.append("residual")
    if any(not HG_SCALE_MIN <= M[4 * i] <= HG_SCALE_MAX for i in range(3)):
        problems.append("scale")
    if any(abs(M[k]) > HG_CROSS_MAX for k in (1, 2, 3, 5, 6, 7)):
        problems.append("cross")
    if any(abs(x) > HG_BIAS_MAX for x in b):
        problems.append("bias")
    return {"matrix": M, "bias": b, "residuals": res, "rms": rms, "max": worst,
            "sigma_matrix": sig_M, "sigma_bias": sig_b,
            "own": {"offset": own_off, "scale": own_scl, "converged": own_conv}, "implied_zero": zero,
            "d_offset": d_off, "d_scale": d_scl, "sigma_d_offset": sig_d_off, "sigma_d_scale": sig_d_scl,
            "disagree": disagree, "problems": problems}


HG_PROBLEM_TEXT = {
    "no_high_g": "the faces came without the high-g reading (update the flight and out computer firmware)",
    "fit": "the fit failed (are the six faces really six different sides?)",
    "residual": f"a face fits worse than {HG_RESIDUAL_MAX / G0 * 1e3:.0f} mg (did it move, or warm up?)",
    "scale": f"a scale is outside {HG_SCALE_MIN}-{HG_SCALE_MAX}",
    "cross": f"a cross-axis term is past {HG_CROSS_MAX}",
    "bias": f"an offset is past {HG_BIAS_MAX} m/s^2",
}


def show_high_g(hg):
    if hg["problems"] == ["no_high_g"] or "matrix" not in hg:
        print(f"\nHigh-g: {HG_PROBLEM_TEXT[hg['problems'][0]]} -- not saved")
        return
    M, b = hg["matrix"], hg["bias"]
    print("\nHigh-g, aligned to the low-g (l = M h - b, all six faces):")
    for i in range(3):
        print(f"  {AXES[i]}: scale {(M[4 * i] - 1) * 1e2:+.3f} % +- {hg['sigma_matrix'][4 * i] * 1e2:.3f}   "
              f"offset {b[i]:+.4f} m/s^2 ({b[i] / G0 * 1e3:+.1f} mg) +- {hg['sigma_bias'][i] / G0 * 1e3:.2f} mg")
    cross = max(abs(M[k]) for k in (1, 2, 3, 5, 6, 7))
    print(f"  cross-axis: up to {math.degrees(math.atan(cross)):.2f} deg "
          f"(+- {math.degrees(max(hg['sigma_matrix'][k] for k in (1, 2, 3, 5, 6, 7))):.3f})")
    print(f"  residual: {hg['rms'] / G0 * 1e3:.2f} mg rms, {hg['max'] / G0 * 1e3:.2f} mg max "
          f"(limit {HG_RESIDUAL_MAX / G0 * 1e3:.0f} mg)")
    own = hg["own"]
    print("  its own gravity fit, against the alignment:")
    for i in range(3):
        print(f"    {AXES[i]}: zero-g {own['offset'][i] / G0 * 1e3:+.1f} mg vs {hg['implied_zero'][i] / G0 * 1e3:+.1f} "
              f"(diff {hg['d_offset'][i] / G0 * 1e3:+.2f} mg, 3 sigma {3 * hg['sigma_d_offset'][i] / G0 * 1e3:.2f}); "
              f"scale diff {hg['d_scale'][i] * 1e2:+.3f} % (3 sigma {3 * hg['sigma_d_scale'][i] * 1e2:.3f})")
    if hg["disagree"]:
        print(f"  they DISAGREE on {', '.join(AXES[i] for i in hg['disagree'])}: the alignment to the low-g "
              "(the bench reference) is what is saved")
    else:
        print("  they agree within their noise")
    if hg["problems"]:
        print("  NOT saved: " + "; ".join(HG_PROBLEM_TEXT[p] for p in hg["problems"]))


async def write_high_g(link, op, matrix=IDENTITY, bias=(0.0, 0.0, 0.0)):
    await link.drain()
    await link.send(CMD_HG_APPLY, struct.pack(FMT_HG_APPLY, op, *matrix, *bias))
    st = await link.wait(lambda s: True, 8.0)
    back = await read_status(link)
    return back or st


def check_face(st, stored):
    """One window's corrected high-g (M h - b - o) less its calibrated low-g (l - o), m/s^2, by the stored
    calibration -- what the flight computer's converter makes of that attitude. None without a high-g."""
    h = st.get("hg")
    if h is None or stored.get("hg") is None:
        return None
    m, b = stored["hg"]["matrix"], stored["hg"]["bias"]
    mh = mat_vec(m, h["mean"])
    return [mh[i] - b[i] - st["mean"][i] for i in range(3)]


def high_g_read_back_ok(back, matrix, bias):
    h = back.get("hg") if back else None
    return (h is not None and bool(h["flags"] & HGF_STORED) and h["bias_src"] == 2
            and all(abs(h["matrix"][i] - matrix[i]) < 1e-6 for i in range(9))
            and all(abs(h["bias"][i] - bias[i]) < 1e-5 for i in range(3)))


async def write_offset(link, off):
    await link.drain()
    await link.send(CMD_APPLY, struct.pack("<3f", *off))
    # the flight computer answers the apply with its status; read it back as well
    st = await link.wait(lambda s: True, 8.0)
    back = await read_status(link)
    return back or st


async def run(args):
    d = await find(args.name)
    if d is None:
        print("DEVICE NOT FOUND -- is the phone app disconnected, and the rocket powered?")
        return 1
    async with BleakClient(d) as c:
        link = Link(c)
        await c.start_notify(FILE_OPS_UUID, link.on_ops)
        st = await read_status(link)
        if st is None:
            print("no answer to BLE 77 -- this flight/out computer firmware has no accelerometer calibration (#1629)")
            return 1
        print("now:")
        show_status(st)
        if args.read:
            return 0
        if args.check:
            if st.get("hg") is None or not st["hg"]["flags"] & HGF_STORED:
                print("no high-g alignment stored -- nothing to check")
                return 1
            print("\nAny attitude, still, then Enter: the stored calibration's corrected high-g against the "
                  "calibrated low-g (q + Enter to stop).")
            while (await ask("Enter to measure, q to stop ... ")).strip().lower() != "q":
                f = await measure_face(link)
                if f is None or f["face_state"] != 2:
                    print("  no answer, or it moved -- again")
                    continue
                d = check_face(f, st)
                if d is None:
                    print("  this frame has no high-g")
                    continue
                print(f"  low-g ({f['mean'][0]:+.3f}, {f['mean'][1]:+.3f}, {f['mean'][2]:+.3f}) m/s^2:  "
                      f"high-g - low-g ({d[0] / G0 * 1e3:+.2f}, {d[1] / G0 * 1e3:+.2f}, {d[2] / G0 * 1e3:+.2f}) mg")
            return 0
        if args.clear:
            if (await ask("Remove the stored offset (and the high-g alignment; its bias stays)? [y/N] ")).strip().lower() != "y":
                print("left as it is")
                return 0
            back = await write_offset(link, (0.0, 0.0, 0.0))
            # the high-g's alignment too when one is stored (an older flight computer would not answer 80)
            if st.get("hg") is not None and st["hg"]["flags"] & HGF_STORED:
                back = await write_high_g(link, HG_OP_CLEAR) or back
            print("after:")
            if back is None:
                print("  no answer -- read it back with --read")
                return 1
            show_status(back)
            return 0

        g = local_g(args.lat, args.height) if args.lat is not None else G0
        print(f"\nSix faces: each board axis straight up, then straight down, the board STILL for 3 s each.\n"
              f"Board axes: +X {WORDS[0]}, +Y {WORDS[1]}, +Z {WORDS[2]}. Order does not matter; the script "
              f"tells each face from its reading.\n")
        faces = {}
        while len(faces) < 6:
            need = [face_name(ax, sgn) for ax in range(3) for sgn in (1, -1) if (ax, sgn) not in faces]
            await ask(f"Still needed: {', '.join(need)}.  Set it on a new face, still, then Enter ... ")
            f = await measure_face(link)
            if f is None:
                print("  no answer -- try again")
                continue
            mean = f["mean"]
            n = math.sqrt(sum(m * m for m in mean))
            sdm = max(f["sd"])
            ax, sgn, tilt = which_face(mean)
            line = (f"  ({mean[0]:+.4f}, {mean[1]:+.4f}, {mean[2]:+.4f}) m/s^2  |f| {n:.4f}  SD max {sdm * 1e3 / G0:.1f} mg"
                    f"  {f['n']} samples")
            if f["face_state"] == 3:
                print(line + "  -- FAILED: it moved, or it is not one g; again")
                continue
            if tilt > TILT_MAX_DEG:
                print(line + f"  -- {face_name(ax, sgn)}, but {tilt:.0f} deg off vertical; set it squarer and again")
                continue
            if (ax, sgn) in faces:
                print(line + f"  -- {face_name(ax, sgn)} again (tilt {tilt:.1f} deg); kept the new one")
            else:
                print(line + f"  -- {face_name(ax, sgn)} (tilt {tilt:.1f} deg)")
            faces[(ax, sgn)] = f

        off, scl, converged = solve(faces, g)
        print(f"\nSolved (all six faces together, g = {g:.5f} m/s^2):")
        for ax in range(3):
            print(f"  {AXES[ax]}: offset {off[ax]:+.4f} m/s^2 ({off[ax] / G0 * 1e3:+.2f} mg)   "
                  f"scale {(scl[ax] - 1) * 1e2:+.3f} %")
        hg = solve_high_g(faces, off, scl, g)
        show_high_g(hg)
        bad = [AXES[i] for i in range(3) if abs(off[i]) > OFFSET_MAX]
        odd = [AXES[i] for i in range(3) if not 0.97 <= scl[i] <= 1.03]
        if not converged or bad or odd:
            why = ("the fit did not converge" if not converged else
                   f"offset past {OFFSET_MAX} m/s^2 on {', '.join(bad)}" if bad else
                   f"scale outside 0.97-1.03 on {', '.join(odd)}")
            print(f"\n{why} -- redo the faces (was one of them not still, or the wrong way?)")
            return 1
        if args.dry_run:
            print("\n--dry-run: nothing written")
            return 0
        with_hg = not hg["problems"]
        what = "this offset and the high-g alignment" if with_hg else "this offset"
        if (await ask(f"\nWrite {what} to the flight computer? [y/N] ")).strip().lower() != "y":
            print("nothing written")
            return 0
        back = await write_offset(link, off)
        ok = back is not None and (back["flags"] & ACCF_STORED) and all(
            abs(back["offset"][i] - off[i]) < 1e-5 for i in range(3))
        if with_hg and back is not None:
            hb = [float(x) for x in struct.unpack("<3f", struct.pack("<3f", *hg["bias"]))]
            hm = [float(x) for x in struct.unpack("<9f", struct.pack("<9f", *hg["matrix"]))]
            back = await write_high_g(link, HG_OP_STORE, hm, hb) or back
            hg_ok = high_g_read_back_ok(back, hm, hb)
        print("stored:")
        if back is None:
            print("  no answer -- read it back with --read")
            return 1
        show_status(back)
        print("OK: the offset stored and read back" if ok else "!! the offset's read-back does not match what was sent")
        if with_hg:
            print("OK: the high-g alignment stored and read back" if hg_ok else
                  "!! the high-g alignment's read-back does not match what was sent")
            ok = ok and hg_ok
        return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="Mini V1", help="the rocket's BLE name (default: Mini V1, the Beetle)")
    ap.add_argument("--read", action="store_true", help="only read what is stored")
    ap.add_argument("--clear", action="store_true", help="remove the stored offset")
    ap.add_argument("--check", action="store_true",
                    help="after a save: measure any attitude and show the corrected high-g less the low-g")
    ap.add_argument("--dry-run", action="store_true", help="measure and solve, write nothing")
    ap.add_argument("--lat", type=float, help="the site's latitude, deg (local gravity for the scale check)")
    ap.add_argument("--height", type=float, default=0.0, help="the site's height, m")
    sys.exit(asyncio.run(run(ap.parse_args())))


if __name__ == "__main__":
    main()
