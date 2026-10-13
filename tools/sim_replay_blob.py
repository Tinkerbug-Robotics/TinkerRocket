#!/usr/bin/env python3
"""A flight's IMU, barometer and attitude as a TR_SIM_REPLAY flight computer plays them on the COCOM rig (#1600).

The COCOM rig plays a flight's GNSS signals from a SignalSim file. A TR_SIM_REPLAY build of the flight computer
carries the same flight's sensors and plays them from boot: the pad until the receiver has a clock, then each IMU,
barometer and magnetometer sample at its own GPS time, so the raw-GNSS filter, the physics checks and the flight
filter see one flight through boost, coast and descent, with the receiver's own output from the rig's signals
(SimReplay.h). This writes the file that build carries, from the sensors CSV a sim_flight_3d.py run writes beside
its scenario (scenarios/sim_<id>_sensors.csv.gz: 1 kHz, body FRD, NED at the pad) and the scenario's JSON (its GPS
time, prologue and pad).

    tools/sim_replay_blob.py SENSORS_CSV SCENARIO_JSON OUT.bin [--from -30] [--no-port-error]

then build the flight computer with it (any board; the Beetle's LC86G is the one whose fixes carry GPS time):

    idf.py -B build_replay -DTR_BOARD_M1=1 -DTR_SIM_REPLAY_BLOB=/abs/path/OUT.bin build

Samples are the CSV's averaged in pairs, 500 Hz: the mean over each 2 ms keeps the integral the filters take of
them. The barometer reads the static-port error the CSV models (baro_alt_port_m) unless --no-port-error.

Format (little-endian, no padding; SimReplay.cpp reads it), a 128-byte header:
    0  char[4]  magic "TRRP"          4  u16 version (1)       6  u16 header bytes (128)
    8  u16      sample bytes (24)     10 u16 rate, Hz          12 u32 samples
    16 f64      t0: the first sample's time, s from ignition
    24 f64      ignition's GPS time, s (week * 604800 + time of week)
    32 f64      pad latitude, deg     40 f64 longitude, deg    48 f64 height above the ellipsoid, m
    56 f64      the rig file's first instant, s from ignition (minus the prologue)
    64 f64      its last, s from ignition
    72 f32      accelerometer LSB, m/s^2     76 f32 gyro LSB, rad/s
    80 f32      pressure reference, Pa       84 f32 pressure LSB, Pa
    88 f32      pad loop, s: the first this many seconds are the pad, played in a loop before the clock is known
    92 f32      landed loop, s: the last this many seconds are landed, played in a loop after the end
    96 f32      decimal year, for the magnetic model
    100 char[28] the flight's name, NUL-padded
then per sample, 24 bytes:
    0  i16[3]  specific force, rocket frame FLU (x nose, y left, z up), accelerometer LSB
    6  i16[3]  angular rate, FLU, gyro LSB
    12 i16[4]  attitude, FLU body -> NED, scalar first, 1/32767
    20 i16     pressure minus the reference, pressure LSB
    22 u8      phase (0 pad, 1 rail, 2 boost, 3 coast, 4 drogue, 5 main, 6 landed)
    23 u8      flags: bit 0 the high-g channel was in use
"""

import argparse
import datetime as dt
import gzip
import json
import struct
import sys

import numpy as np

MAGIC = b"TRRP"
VERSION = 1
HEADER_BYTES = 128
SAMPLE_BYTES = 24
HEADER_FMT = "<4sHHHHIdddddddfffffff28s"
SAMPLE_FMT = "<3h3h4hhBB"
GPS_EPOCH = dt.datetime(1980, 1, 6)
LEAP_S = 18                                  # GPS - UTC since 2017
BARO_N = 5.255                               # the CSV's altitude from pressure: 44330 (1 - (p/p0)^(1/5.255))
PHASE_PAD, PHASE_LANDED = 0, 6

assert struct.calcsize(HEADER_FMT) == HEADER_BYTES
assert struct.calcsize(SAMPLE_FMT) == SAMPLE_BYTES


def read_csv(path):
    """Columns by name from a sensors CSV (# comment lines, then a header line)."""
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt") as f:
        lines = [ln for ln in f if not ln.startswith("#")]
    names = lines[0].strip().split(",")
    data = np.loadtxt(lines[1:], delimiter=",", ndmin=2)
    return {n: data[:, i] for i, n in enumerate(names)}


def frd_to_flu_quat(q):
    """q (N x 4, scalar first) turning body FRD into NED -> the same attitude for body FLU: q * (0, 1, 0, 0),
    the half turn about the nose that takes FLU onto FRD."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([-x, w, z, -y], axis=1)


def rotate(q, v):
    """R(q) v for each row (q scalar first)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    r = np.empty_like(v)
    r[:, 0] = (1 - 2 * (y * y + z * z)) * v[:, 0] + 2 * (x * y - w * z) * v[:, 1] + 2 * (x * z + w * y) * v[:, 2]
    r[:, 1] = 2 * (x * y + w * z) * v[:, 0] + (1 - 2 * (x * x + z * z)) * v[:, 1] + 2 * (y * z - w * x) * v[:, 2]
    r[:, 2] = 2 * (x * z - w * y) * v[:, 0] + 2 * (y * z + w * x) * v[:, 1] + (1 - 2 * (x * x + y * y)) * v[:, 2]
    return r


def gps_seconds(utc_iso):
    """Continuous GPS seconds (week * 604800 + time of week) of a UTC time."""
    t = dt.datetime.fromisoformat(utc_iso) + dt.timedelta(seconds=LEAP_S)
    return (t - GPS_EPOCH).total_seconds()


def decimal_year(utc_iso):
    """As the flight computer works it out for the magnetic model: year + (day of year - 1) / days in the year."""
    t = dt.datetime.fromisoformat(utc_iso)
    days = 366 if (t.year % 4 == 0 and (t.year % 100 != 0 or t.year % 400 == 0)) else 365
    return t.year + (t.timetuple().tm_yday - 1) / days


def lsb_for(peak, floor, step):
    """The finest LSB (a multiple of step, at least floor) that holds peak in an int16 with a margin."""
    need = peak / 32000.0
    return max(floor, step * np.ceil(need / step - 1e-9))


def build(cols, scen, t_from=-30.0, port_error=True, pad_loop_s=10.0, land_loop_s=5.0, name=None):
    """The blob's bytes and a summary."""
    t = cols["t_s"]
    keep = t >= t_from - 1e-9
    n = int(keep.sum()) // 2 * 2
    if n < 4:
        raise ValueError(f"fewer than 4 samples from t = {t_from} s")
    idx = np.flatnonzero(keep)[:n]
    dts = np.diff(t[idx])
    if not np.allclose(dts, dts[0], atol=1e-6):
        raise ValueError("the CSV's samples are not evenly spaced")
    rate = int(round(1.0 / (2 * dts[0])))

    def pairs(a):
        a = a[idx]
        return 0.5 * (a[0::2] + a[1::2])

    tt = pairs(t)
    f_frd = np.stack([pairs(cols[c]) for c in ("fx_mps2", "fy_mps2", "fz_mps2")], axis=1)
    w_frd = np.stack([pairs(cols[c]) for c in ("wx_rps", "wy_rps", "wz_rps")], axis=1)
    q_frd = np.stack([cols[c][idx][0::2] for c in ("qw", "qx", "qy", "qz")], axis=1)
    # Each pair's attitude is the mean of the two (a few mrad apart at the most): the normalised sum
    q2 = np.stack([cols[c][idx][1::2] for c in ("qw", "qx", "qy", "qz")], axis=1)
    q2 *= np.sign(np.sum(q_frd * q2, axis=1))[:, None]
    q_frd = q_frd + q2
    q_frd /= np.linalg.norm(q_frd, axis=1)[:, None]
    phase = np.maximum(cols["phase"][idx][0::2], cols["phase"][idx][1::2]).astype(int)
    hg = np.maximum(cols["acc_hg"][idx][0::2], cols["acc_hg"][idx][1::2]).astype(int)

    # the firmware's rocket frame is FLU: y and z flip
    flip = np.array([1.0, -1.0, -1.0])
    f = f_frd * flip
    w = w_frd * flip
    q = frd_to_flu_quat(q_frd)
    q *= np.where(q[:, :1] < 0, -1.0, 1.0)
    err = np.abs(rotate(q, f) - rotate(q_frd, f_frd)).max()
    if err > 1e-6 * max(1.0, np.abs(f).max()):
        raise AssertionError(f"FLU attitude does not reproduce the FRD one ({err:g})")

    p = pairs(cols["baro_p_pa"])
    if port_error:
        # The static port reads low by -k q (baro_alt_port_m - baro_alt_m): the pressure that altitude error is,
        # from the CSV's own altitude formula's slope, dp/dh = -n p / (44330 - h)
        h = pairs(cols["baro_alt_m"])
        dh = pairs(cols["baro_alt_port_m"]) - h
        p = p - BARO_N * p / (44330.0 - h) * dh

    acc_lsb = lsb_for(np.abs(f).max(), 0.01, 0.01)
    gyro_lsb = lsb_for(np.abs(w).max(), 0.001, 0.001)
    p_ref = float(np.round(p[0]))
    p_lsb = lsb_for(np.abs(p - p_ref).max(), 0.25, 0.25)

    t0 = float(tt[0])
    t_last = float(tt[-1])
    if t_last - t0 < pad_loop_s + land_loop_s:
        raise ValueError("too short for its pad and landed loops")
    if np.any(phase[tt < t0 + pad_loop_s + 1.0 / rate] != PHASE_PAD):
        raise ValueError(f"the first {pad_loop_s} s are not all pad: start earlier (--from)")
    if np.any(phase[tt > t_last - land_loop_s - 1.0 / rate] != PHASE_LANDED):
        raise ValueError(f"the last {land_loop_s} s are not all landed")

    prologue = float(scen["prologue_s"])
    ignition_gps = gps_seconds(scen["start_utc"]) + prologue
    tow0 = float(scen.get("tow0_file", -1.0))
    if tow0 >= 0 and abs((gps_seconds(scen["start_utc"]) % 604800.0) - tow0) > 1e-3:
        raise ValueError(f"start_utc {scen['start_utc']} is not the scenario's tow0_file {tow0}")
    origin = scen["origin"]
    flight = scen.get("flight") if isinstance(scen.get("flight"), dict) else {}
    nm = (name or flight.get("id") or scen["scenario"]).encode()[:27]

    header = struct.pack(HEADER_FMT, MAGIC, VERSION, HEADER_BYTES, SAMPLE_BYTES, rate, len(tt),
                         t0, ignition_gps, float(origin["lat_deg"]), float(origin["lon_deg"]),
                         float(origin["alt_m"]), -prologue, float(scen["duration_s"]) - prologue,
                         acc_lsb, gyro_lsb, p_ref, p_lsb, pad_loop_s, land_loop_s,
                         decimal_year(scen["start_utc"]), nm)

    def i16(a, lsb):
        v = np.round(a / lsb)
        if np.abs(v).max() > 32767:
            raise ValueError("out of int16 range")
        return v.astype(np.int16)

    fi = i16(f, acc_lsb)
    wi = i16(w, gyro_lsb)
    qi = i16(q, 1.0 / 32767.0)
    pi = i16(p - p_ref, p_lsb)
    rec = np.zeros(len(tt), dtype=np.dtype([("f", "<i2", 3), ("w", "<i2", 3), ("q", "<i2", 4), ("p", "<i2"),
                                             ("phase", "u1"), ("flags", "u1")]))
    assert rec.dtype.itemsize == SAMPLE_BYTES
    rec["f"], rec["w"], rec["q"], rec["p"] = fi, wi, qi, pi
    rec["phase"], rec["flags"] = phase, hg & 1
    summary = {
        "samples": len(tt), "rate_hz": rate, "t0_s": t0, "t_last_s": t_last,
        "ignition_gps_s": ignition_gps, "gps_week": int(ignition_gps // 604800),
        "ignition_tow_s": ignition_gps % 604800, "acc_lsb": acc_lsb, "gyro_lsb": gyro_lsb,
        "p_ref_pa": p_ref, "p_lsb_pa": p_lsb, "peak_acc_mps2": float(np.abs(f).max()),
        "peak_gyro_rps": float(np.abs(w).max()), "min_p_pa": float(p.min()),
        "bytes": HEADER_BYTES + len(tt) * SAMPLE_BYTES,
    }
    return header + rec.tobytes(), summary


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("sensors_csv")
    ap.add_argument("scenario_json")
    ap.add_argument("out")
    ap.add_argument("--from", dest="t_from", type=float, default=-30.0,
                    help="the first sample, s from ignition (default -30; the pad before it is looped)")
    ap.add_argument("--no-port-error", action="store_true", help="the barometer without the static-port error")
    ap.add_argument("--name", help="the flight's name in the header (default: the scenario's)")
    ap.add_argument("--pad-loop", type=float, default=10.0, help="s of pad looped before the clock (default 10)")
    ap.add_argument("--land-loop", type=float, default=5.0, help="s of ground looped after the end (default 5)")
    args = ap.parse_args(argv)
    with open(args.scenario_json) as f:
        scen = json.load(f)
    blob, s = build(read_csv(args.sensors_csv), scen, t_from=args.t_from, port_error=not args.no_port_error,
                    pad_loop_s=args.pad_loop, land_loop_s=args.land_loop, name=args.name)
    with open(args.out, "wb") as f:
        f.write(blob)
    print(f"{args.out}: {s['bytes'] / 1e6:.2f} MB, {s['samples']} samples at {s['rate_hz']} Hz, "
          f"t {s['t0_s']:.3f} .. {s['t_last_s']:.3f} s; ignition GPS week {s['gps_week']} TOW "
          f"{s['ignition_tow_s']:.3f}; LSBs {s['acc_lsb']:g} m/s^2, {s['gyro_lsb']:g} rad/s, {s['p_lsb_pa']:g} Pa "
          f"(peaks {s['peak_acc_mps2']:.1f} m/s^2, {s['peak_gyro_rps']:.2f} rad/s, lowest {s['min_p_pa']:.0f} Pa)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
