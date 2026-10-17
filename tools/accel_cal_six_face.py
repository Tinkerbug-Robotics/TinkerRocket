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

Disconnect the phone app first: one BLE central at a time, and the app's connect-time sync would queue its own
commands. Run from the Terminal panel (BLE).

    accel_cal_six_face.py                 # the six faces, then write on a yes
    accel_cal_six_face.py --read          # what the flight computer has stored
    accel_cal_six_face.py --clear         # remove the stored offset (on a yes)
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
CMD_APPLY, CMD_READ, CMD_FACE = 76, 77, 78
MARKER = 0xD1
FMT = "<BB3f3f3fH"                  # AccelCalStatusData, 40 bytes
ACCF_STORED, ACCF_APPLIED = 0x01, 0x02
FACE_STATES = {0: "idle", 1: "measuring", 2: "done", 3: "FAILED"}
G0 = 9.80665
OFFSET_MAX = 1.0                    # m/s^2: the flight computer refuses larger (ACC_CAL_OFFSET_MAX_MPS2)
TILT_MAX_DEG = 10.0                 # a face whose axis is further off vertical is measured again
AXES = "XYZ"
WORDS = {0: "the board's +X (forward) end", 1: "the board's +Y (left) side", 2: "the board's +Z (component) side"}


def face_name(ax, sgn):
    return f"+{AXES[ax]} {'UP' if sgn > 0 else 'DOWN'}"


def local_g(lat_deg, h_m=0.0):
    """Normal gravity (WGS84, Somigliana) at a latitude and height, m/s^2."""
    s2 = math.sin(math.radians(lat_deg)) ** 2
    return 9.7803253359 * (1 + 0.00193185265241 * s2) / math.sqrt(1 - 0.00669437999013 * s2) - 3.086e-6 * h_m


def parse(data):
    if len(data) < 1 + struct.calcsize(FMT) or data[0] != MARKER:
        return None
    f = struct.unpack_from(FMT, data, 1)
    return {"flags": f[0], "face_state": f[1], "offset": f[2:5], "mean": f[5:8], "sd": f[8:11], "n": f[11]}


def show_status(st):
    o = st["offset"]
    stored = bool(st["flags"] & ACCF_STORED)
    applied = bool(st["flags"] & ACCF_APPLIED)
    print(f"  stored offset: {'yes' if stored else 'none'}"
          + (f"  ({o[0]:+.4f}, {o[1]:+.4f}, {o[2]:+.4f}) m/s^2 = "
             f"({o[0] / G0 * 1e3:+.1f}, {o[1] / G0 * 1e3:+.1f}, {o[2] / G0 * 1e3:+.1f}) mg" if stored else "")
          + f";  applied by the converter: {'yes' if applied else 'no'}")


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
        if args.clear:
            if (await ask("Remove the stored offset? [y/N] ")).strip().lower() != "y":
                print("left as it is")
                return 0
            back = await write_offset(link, (0.0, 0.0, 0.0))
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
        if (await ask("\nWrite this offset to the flight computer? [y/N] ")).strip().lower() != "y":
            print("nothing written")
            return 0
        back = await write_offset(link, off)
        print("stored:")
        if back is None:
            print("  no answer -- read it back with --read")
            return 1
        show_status(back)
        ok = (back["flags"] & ACCF_STORED) and all(abs(back["offset"][i] - off[i]) < 1e-5 for i in range(3))
        print("OK: stored and read back" if ok else "!! the read-back does not match what was sent")
        return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="Mini V1", help="the rocket's BLE name (default: Mini V1, the Beetle)")
    ap.add_argument("--read", action="store_true", help="only read what is stored")
    ap.add_argument("--clear", action="store_true", help="remove the stored offset")
    ap.add_argument("--dry-run", action="store_true", help="measure and solve, write nothing")
    ap.add_argument("--lat", type=float, help="the site's latitude, deg (local gravity for the scale check)")
    ap.add_argument("--height", type=float, default=0.0, help="the site's height, m")
    sys.exit(asyncio.run(run(ap.parse_args())))


if __name__ == "__main__":
    main()
