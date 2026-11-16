#!/usr/bin/env python3
"""A PX1105R rig capture beside a simulated flight's sensors, as raw_replay's events file (--receiver px1105r).

The RF rig played a simulated flight's sky (scenarios/real_sim_<flight>_sky3d_pad180) into a PX1105R and logged its
binary stream ('<host_t> B <hex body>' per SkyTraq frame); the same flight's sensors (scenarios/sim_<flight>_sensors
.csv.gz, 1 kHz, body FRD, NED at the pad, t_s from ignition) are what a TR_SIM_REPLAY flight computer plays beside
that sky. This puts both on one MCU clock -- MCU s = flight s + T_OFS, a perfect crystal -- by GPS time:
  - the sensors: flight time t_s is GPS time less the ignition's (the scenario's tow0_file + prologue_s);
  - each SkyTraq frame: its host arrival time mapped to GPS time by the lower envelope of the 0xE5s' arrivals
    against their epochs' GPS instants (each 0xE5's following 0xDF states it), so the earliest-arriving 0xE5s land
    on their own instant, plus --latency-ms (the receiver's output delay and the UART's, which the host's stamps
    cannot measure: a stand-in);
written as whole frames (A0 A1 .. 0D 0A), one R line each, at their arrival.

Events (sim_events.py's conventions, so the harness's cold start, quorum and scoring take it as a sim):
  I  every 2 ms: the mean of two 1 kHz rows (the replay's 500 Hz pairs), m/s^2 and rad/s, body FRD; flags 0x01 on
     the pad, 0x02 from ignition, 0x09 from a second after touchdown;
  B  each fresh 500 Hz barometer sample: pressure from the height with the static-port error (baro_alt_port_m) on
     META ground_pressure_pa (as sim_replay_blob.py's default);
  M  100 Hz: the model field (TR_GeoMag at the pad, given) turned into the body by the truth's attitude, 0.3 uT noise;
  K  one clock line, the ideal one (--px-tag line places the epochs on it);
  R  the PX1105R's frames.
META: t_base_s, state_machine none, ground_pressure_pa, pad_lla, seed_quat (the truth at the first tick, the sims'
seed_quat_truth 1), decimal_year, declination_rad, field_ned_ut, t_inflight_us, t_landed_flag_us, truth phase times,
the GPS time of ignition, the capture's mapping, and app_iono (--iono: the sky's own Klobuchar, which the rig's sky
never broadcasts; the V9 rig image takes it as the receiver's).

    px_events.py SENSORS_CSV SCENARIO_JSON CAPTURE OUT.events --field-ned N,E,D --declination-rad X
        [--decimal-year Y] [--latency-ms 20] [--iono "a0 a1 a2 a3 b0 b1 b2 b3 day"] [--from-s -170]

A static capture (--static): no sensors file; a level, still IMU (gravity and noise, no earth rate) for the span of
the capture, heading 0, the barometer constant, the mosaic seed none (the harness seeds from the receiver's fix).

"""
import argparse
import datetime as dt
import gzip
import json
import math
import struct
import sys

import numpy as np

T_OFS = 200.0                 # MCU s = flight s + T_OFS
G0 = 9.80665
GPS_EPOCH = dt.datetime(1980, 1, 6)
LEAP_S = 18


def frame(body):
    cs = 0
    for c in body:
        cs ^= c
    return b"\xa0\xa1" + struct.pack(">H", len(body)) + body + bytes([cs]) + b"\r\n"


def read_capture(path):
    op = gzip.open if path.endswith(".gz") else open
    out = []
    with op(path, "rt", errors="replace") as f:
        for ln in f:
            parts = ln.split(None, 2)
            if len(parts) < 3 or parts[1] != "B":
                continue
            try:
                out.append((float(parts[0]), bytes.fromhex(parts[2].strip())))
            except ValueError:
                continue
    return out


def envelope(frames):
    """Host time -> GPS time: each 0xE5 paired with the 0xDF after it (the same epoch: its GPS time), the lower
    envelope of host - GPS over 10 s bins, fitted by a line (robust)."""
    pts = []
    pend = None
    for t, b in frames:
        if b[0] == 0xE5 and len(b) >= 14:
            pend = t
        elif b[0] == 0xDF and len(b) >= 81 and pend is not None:
            st = b[2]
            if st >= 2:
                week = struct.unpack_from(">H", b, 3)[0]
                tow = struct.unpack_from(">d", b, 5)[0]
                pts.append((pend, week * 604800.0 + tow))
            pend = None
    pts = np.array(pts)
    off = pts[:, 0] - pts[:, 1]
    bins = np.floor((pts[:, 0] - pts[0, 0]) / 10.0)
    bx, by = [], []
    for k in np.unique(bins):
        m = bins == k
        j = np.argmin(off[m])
        bx.append(pts[m][j, 0])
        by.append(off[m][j])
    bx, by = np.array(bx), np.array(by)
    keep = np.ones(len(bx), bool)
    for _ in range(3):
        c = np.polyfit(bx[keep], by[keep], 1)
        r = by - np.polyval(c, bx)
        keep = r < np.percentile(r[keep], 75) + 0.002
    c = np.polyfit(bx[keep], by[keep], 1)
    lat = off - np.polyval(c, pts[:, 0])
    return c, pts, lat


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sensors", help="sim_<flight>_sensors.csv.gz (or 'none' with --static)")
    ap.add_argument("scenario", help="the scenario JSON (or 'none' with --static)")
    ap.add_argument("capture")
    ap.add_argument("out")
    ap.add_argument("--field-ned", required=True, help="N,E,D uT: TR_GeoMag's at the pad and date")
    ap.add_argument("--declination-rad", type=float, required=True)
    ap.add_argument("--decimal-year", type=float, default=2026.66)
    ap.add_argument("--latency-ms", type=float, default=20.0,
                    help="each frame's arrival after the envelope's (the receiver's output delay + the UART's)")
    ap.add_argument("--iono", help="'a0 a1 a2 a3 b0 b1 b2 b3 gps_day': an app ionosphere set (META app_iono)")
    ap.add_argument("--from-s", type=float, default=-170.0, help="the first sensor sample, s from ignition")
    ap.add_argument("--static", action="store_true", help="a static capture: a still, level IMU, no sensors file")
    ap.add_argument("--pad-lla", help="lat,lon,h: the static pad (the receiver's mean fix)")
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    field = np.array([float(x) for x in args.field_ned.split(",")])

    frames = read_capture(args.capture)
    c, pts, lat = envelope(frames)
    env = lambda t: t - np.polyval(c, t)   # host t -> GPS s
    print(f"capture: {len(frames)} frames; mapping host -> GPS: offset {c[1]:.6f} s, slope {c[0] * 1e6:+.3f} ppm; "
          f"0xE5 arrival over the envelope: median {np.median(lat) * 1e3:.1f} ms, p90 "
          f"{np.percentile(lat, 90) * 1e3:.1f} ms, max {lat.max() * 1e3:.1f} ms, min {lat.min() * 1e3:.1f} ms "
          f"({len(lat)} epochs)", file=sys.stderr)
    gps_first = float(pts[0, 1])

    lines = []
    ev = []
    if not args.static:
        sc = json.load(open(args.scenario))
        start = dt.datetime.fromisoformat(sc["start_utc"])
        week = int(((start - GPS_EPOCH).total_seconds() + LEAP_S) // 604800)
        gps_ign = week * 604800.0 + float(sc["tow0_file"]) + float(sc["prologue_s"])
        org = sc["origin"]
        pad = (float(org["lat_deg"]), float(org["lon_deg"]), float(org["alt_m"]))
        rows = [ln for ln in gzip.open(args.sensors, "rt") if not ln.startswith("#")]
        names = rows[0].strip().split(",")
        d = np.loadtxt(rows[1:], delimiter=",")
        col = {n: i for i, n in enumerate(names)}
        t = d[:, col["t_s"]]
        k0 = int(np.searchsorted(t, args.from_s))
        d = d[k0:]
        t = t[k0:]
        acc = d[:, [col["fx_mps2"], col["fy_mps2"], col["fz_mps2"]]]
        gyr = d[:, [col["wx_rps"], col["wy_rps"], col["wz_rps"]]]
        q = d[:, [col["qw"], col["qx"], col["qy"], col["qz"]]]
        phase = d[:, col["phase"]].astype(int)
        pd_ = d[:, col["pd_m"]]
        h_port = d[:, col["baro_alt_port_m"]]
        bnew = d[:, col["baro_new"]].astype(int)
        p_pad = d[0, col["baro_p_pa"]]
        burnout = float(t[np.argmax(phase >= 3)])
        apogee = float(t[int(np.argmin(pd_))])
        touch = float(t[np.argmax(phase >= 6)]) if (phase >= 6).any() else float(t[-1])
        P0 = float(np.mean(d[:200, col["baro_p_pa"]]))
        us = lambda x: int(round((x + T_OFS) * 1e6))
        for i in range(0, len(t) - 1, 2):
            a = 0.5 * (acc[i] + acc[i + 1])
            g = 0.5 * (gyr[i] + gyr[i + 1])
            ti = t[i + 1]
            flags = 0x01 if ti < 0.0 else 0x02
            if ti >= touch + 1.0:
                flags = 0x09
            ev.append((us(ti), 4, f"I {us(ti)} {a[0]:.5f} {a[1]:.5f} {a[2]:.5f} {g[0]:.7f} {g[1]:.7f} {g[2]:.7f} "
                                  f"{flags}"))
        for i in np.flatnonzero(bnew):
            pr = P0 * (1.0 - h_port[i] / 44330.0) ** 5.255
            ev.append((us(t[i]), 0, f"B {us(t[i])} {pr:.3f}"))
        for i in range(0, len(t), 10):
            w, x, y, z = q[i]
            R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                          [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                          [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])   # body -> NED
            mb = R.T @ field + rng.normal(0.0, 0.3, 3)
            ev.append((us(t[i]), 1, f"M {us(t[i])} {mb[0]:.4f} {mb[1]:.4f} {mb[2]:.4f}"))
        q0 = q[1]
        t_first_s = float(t[0])
        lines += [f"META t_inflight_us {us(0.0)}", f"META t_landed_flag_us {us(touch + 1.0)}",
                  f"META pad_lla {pad[0]!r} {pad[1]!r} {pad[2]!r}",
                  f"META seed_quat {q0[0]!r} {q0[1]!r} {q0[2]!r} {q0[3]!r}", "META seed_quat_truth 1",
                  f"META truth_thrust_start_s 0.0", f"META truth_burnout_s {burnout!r}",
                  f"META truth_apogee_s {apogee!r}", f"META truth_touchdown_s {touch!r}",
                  f"META flight {sc['scenario']}"]
    else:
        # a still, level board: heading 0 (north), so the body is NED
        gps_last = float(pts[-1, 1])
        gps_ign = gps_first + 60.0           # "flight time" 0 a minute into the receiver's fixes
        us = lambda x: int(round((x + T_OFS) * 1e6))
        t0 = gps_first - gps_ign - 5.0
        t1 = gps_last - gps_ign
        P0 = 101325.0
        n = int((t1 - t0) / 0.002)
        tt = t0 + 0.002 * np.arange(n)
        an = rng.normal(0.0, 0.022 / math.sqrt(2), (n, 3))
        gn = rng.normal(0.0, 0.0026 / math.sqrt(2), (n, 3))
        for i in range(n):
            a = an[i] + np.array([0.0, 0.0, -G0])
            g = gn[i]
            ev.append((us(tt[i]), 4, f"I {us(tt[i])} {a[0]:.5f} {a[1]:.5f} {a[2]:.5f} {g[0]:.7f} {g[1]:.7f} "
                                     f"{g[2]:.7f} 1"))
            if i % 1 == 0:
                ev.append((us(tt[i]), 0, f"B {us(tt[i])} {P0 + rng.normal(0.0, 0.7):.3f}"))
            if i % 5 == 0:
                mb = field + rng.normal(0.0, 0.3, 3)
                ev.append((us(tt[i]), 1, f"M {us(tt[i])} {mb[0]:.4f} {mb[1]:.4f} {mb[2]:.4f}"))
        t_first_s = float(tt[0])
        lines += ["META seed_quat 1.0 0.0 0.0 0.0", "META seed_quat_truth 1", "META flight static"]
        if args.pad_lla:
            la, lo, h = (float(x) for x in args.pad_lla.split(","))
            lines.append(f"META pad_lla {la!r} {lo!r} {h!r}")

    # the frames, each at its arrival on the MCU clock
    lat_s = args.latency_ms * 1e-3
    nfr = 0
    for th, b in frames:
        g = env(th) + lat_s
        tf = g - gps_ign
        if tf < t_first_s:
            continue
        ev.append((us(tf), 2, f"R {us(tf)} {frame(b).hex()}"))
        nfr += 1
    # the ideal clock line: MCU = GPS - gps_ign + T_OFS (whole second references)
    g0 = math.floor(gps_ign)
    mcu0 = us(g0 - gps_ign)
    k_t = us(t_first_s)
    ev.append((k_t, -1, f"K {k_t} {g0:.1f} {mcu0} 0.0 0.0 0.0 0.0 1"))
    ev.sort(key=lambda e: (e[0], e[1]))
    head = [f"# px_events.py: {args.capture}; flight time = MCU s + t_base_s",
            f"META t_base_s {-T_OFS}", "META state_machine none", f"META ground_pressure_pa {P0:.3f}",
            f"META decimal_year {args.decimal_year}", f"META declination_rad {args.declination_rad!r}",
            f"META field_ned_ut {float(field[0])!r} {float(field[1])!r} {float(field[2])!r}",
            f"META gps_ignition_s {gps_ign!r}", f"META px_latency_ms {args.latency_ms}",
            f"META px_map_offset_s {float(c[1])!r} px_map_slope {float(c[0])!r}"]
    if args.iono:
        head.append(f"META app_iono {args.iono}")
    with open(args.out, "w") as fh:
        fh.write("\n".join(head + lines) + "\n")
        for e in ev:
            fh.write(e[2] + "\n")
    print(f"wrote {args.out}: {len(ev)} events, {nfr} frames; ignition GPS {gps_ign:.3f}", file=sys.stderr)


if __name__ == "__main__":
    main()
