#!/usr/bin/env python3
"""An RF-rig run of the gnss-sdr-dev board as raw_replay's events file, with the flight's truth.

The rig plays a SignalSim flight (gnss repo cocom-rig/sdr/signalsim: GPS L1 C/A, Galileo E1, BeiDou B1C at
1575.42 MHz) into the board, which records its own RTCM3 and sends it to the V10. The board's stream is real; the
flight computer sat still on the bench, so the IMU, barometer and magnetometer the flight would have given are made
here from the same SignalSim config's trajectory, and the board's frames are put on their clock by GPS time:

  I  the EKF ticks at --ekf-rate: the trajectory's specific force and the earth's rate in the body, white noise and
     a constant bias on each axis (--clean: neither);
  B  the barometer at half that rate: the 1976 standard atmosphere at the height (so the pad reads
     META ground_pressure_pa), to any height;
  M  the magnetometer at 100 Hz: --field-ned-ut in the body (TR_GeoMag's WMM2025 at the files' pad and date by
     default), 0.3 uT of noise;
  R  every frame of the board's stream (the recorder's .rtcm3, or the GNSS_RTCM records of a V10 OC log), released
     --delay-ms after its epoch plus the UART time of the bytes before it in that epoch, and stamped at the flight
     computer's next poll (--poll-ms): the time sync then sees what the flight computer's arrivals give it;
  META the pad (pad_lla, the NED origin), its field and declination, ground_pressure_pa, t_base_s (flight time 0 =
     ignition), t_inflight_us, state_machine none, the truth's phase times. No seed: the cold start waits for the
     board's own fix, as on the flight computer (--seed-truth: seed_lla and seed_quat from the truth).

The trajectory: a SignalSim config's trajectoryList, from initPosition (LLA) at rest -- Const (the velocity held) and
VerticalAcc (a constant vertical acceleration to `speed` m/s over `time` s) -- so the hotshot and traveler files'
own motion, exactly as the files were made (make_b1c_flights.py from make_wide_p180.py). The flight is vertical: the
body is nose up all the way (body x up, --heading-deg the belly's (body z) azimuth from north), with no roll.
Ignition is the first VerticalAcc segment.

The truth goes to OUT.truth.nav (raw_replay --truth-nav's format, at 100 Hz: MCU us, NED velocity, the attitude
body FRD -> NED scalar first, the ellipsoidal height), with its phase times in META; score_sdr_rig.py scores a run
against it.

    sdr_rig_events.py CONFIG.json STREAM OUT.events [--clean] [--seed-truth] [...]
    raw_replay_mosaic --in OUT.events --out run.csv --receiver sdr [--cut-from-s 0 --cut-to-s 99999]
    score_sdr_rig.py OUT.events run.csv [label=run2.csv ...]

STREAM: a .rtcm3 file (sdrdev.py rec-get) or an OC log (.bin). Either way the arrivals are made here: a V10 log's
own stamps are on the bench clock, not the made IMU's.
"""
import argparse
import datetime as dt
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(WT, "Data_Analysis"))
import extract_gnss_rtcm as XR  # noqa: E402

GPS_EPOCH = dt.datetime(1980, 1, 6)
LEAP_S = 18                       # GPS - UTC since 2017
OMEGA_E = 7.2921151467e-5         # rad/s
A_WGS, E2_WGS = 6378137.0, 6.69437999014e-3
T_OFS = 10.0                      # MCU s = file s + T_OFS: room before the file's first sample
SDR_EPH = (1019, 1042, 1046)
# TR_GeoMag (WMM2025) at the B1C files' pad (0 N, 119 W, 1200 m) on 2026-08-18: N E D uT, declination rad
FILES_PAD = (0.0, -119.0, 1200.0)
FILES_FIELD = ((29.3209, 4.2492, 5.5972), 0.1439198)


# ------------------------------------------------------------------ the trajectory

class Trajectory:
    """A SignalSim trajectoryList's vertical motion: velocity piecewise linear between knots."""

    def __init__(self, cfg):
        t = cfg["trajectory"]
        p = t["initPosition"]
        if p.get("type") != "LLA" or p.get("format", "d") != "d":
            raise SystemExit(f"initPosition {p}: only LLA in degrees")
        self.pad = (float(p["latitude"]), float(p["longitude"]), float(p["altitude"]))
        v0 = float(t.get("initVelocity", {}).get("speed", 0.0))
        if v0 != 0.0:
            raise SystemExit("initVelocity: only a start at rest")
        tk, vk = [0.0], [0.0]
        self.ignition = None
        for s in t["trajectoryList"]:
            if s["type"] == "Const":
                v_end = vk[-1]
            elif s["type"] == "VerticalAcc":
                v_end = float(s["speed"])
                if self.ignition is None:
                    self.ignition = tk[-1]
            else:
                raise SystemExit(f"segment type {s['type']}: only Const and VerticalAcc")
            tk.append(tk[-1] + float(s["time"]))
            vk.append(v_end)
        self.tk, self.vk = np.array(tk), np.array(vk)
        # the height at each knot: the trapezoid of each segment (exact for a constant acceleration)
        self.hk = self.pad[2] + np.concatenate([[0.0], np.cumsum(np.diff(self.tk) * (self.vk[1:] + self.vk[:-1]) / 2)])
        self.duration = float(self.tk[-1])
        c = cfg["time"]
        if c.get("type") != "UTC":
            raise SystemExit(f"time {c}: only UTC")
        utc = dt.datetime(c["year"], c["month"], c["day"], c["hour"], c["minute"], int(c["second"]))
        self.gps_start_s = (utc - GPS_EPOCH).total_seconds() + LEAP_S
        self.decimal_year = c["year"] + (utc - dt.datetime(c["year"], 1, 1)).total_seconds() / (
            (dt.datetime(c["year"] + 1, 1, 1) - dt.datetime(c["year"], 1, 1)).total_seconds())

    def at(self, t):
        """Height (m), up velocity (m/s) and up acceleration (m/s^2) at file times t."""
        t = np.asarray(t, float)
        i = np.clip(np.searchsorted(self.tk, t, side="right") - 1, 0, len(self.tk) - 2)
        dtk = self.tk[i + 1] - self.tk[i]
        a = np.where(dtk > 0, (self.vk[i + 1] - self.vk[i]) / np.where(dtk > 0, dtk, 1.0), 0.0)
        tau = np.clip(t - self.tk[i], 0.0, None)
        v = self.vk[i] + a * tau
        h = self.hk[i] + self.vk[i] * tau + 0.5 * a * tau * tau
        a = np.where(t >= self.tk[-1], 0.0, a)
        return h, v, a

    def phases(self):
        """Ignition, burnout (the first deceleration after it) and apogee (the up velocity's sign change), file s;
        None for a static file."""
        if self.ignition is None:
            return None, None, None
        k = np.flatnonzero(self.tk[:-1] >= self.ignition)
        dv = np.diff(self.vk)
        burn = next((self.tk[j] for j in k if dv[j] < 0), None)
        apo = next((self.tk[j + 1] for j in k if self.vk[j] > 0 >= self.vk[j + 1]), None)
        if apo is not None:     # the zero crossing inside its segment
            j = int(np.flatnonzero(self.tk == apo)[0]) - 1
            apo = self.tk[j] + (self.tk[j + 1] - self.tk[j]) * self.vk[j] / (self.vk[j] - self.vk[j + 1])
        return self.ignition, burn, apo


def pressure_pa(h_msl):
    """The 1976 standard atmosphere's pressure (Pa) at a geopotential height, to 86 km (and its last layer on)."""
    layers = ((0.0, 101325.0, 288.15, -0.0065), (11000.0, 22632.06, 216.65, 0.0), (20000.0, 5474.889, 216.65, 0.001),
              (32000.0, 868.0187, 228.65, 0.0028), (47000.0, 110.9063, 270.65, 0.0), (51000.0, 66.93887, 270.65, -0.0028),
              (71000.0, 3.956420, 214.65, -0.002))
    g0, M, R = 9.80665, 0.0289644, 8.3144598
    hb, pb, tb, lb = next(L for L in reversed(layers) if h_msl >= L[0]) if h_msl >= 0 else layers[0]
    if lb == 0.0:
        return pb * math.exp(-g0 * M * (h_msl - hb) / (R * tb))
    return pb * (tb / (tb + lb * (h_msl - hb))) ** (g0 * M / (R * lb))


def gravity(lat_rad, h):
    """Normal gravity (Somigliana) with the free-air term, m/s^2, down."""
    s2 = math.sin(lat_rad) ** 2
    g0 = 9.7803253359 * (1 + 0.00193185265241 * s2) / math.sqrt(1 - E2_WGS * s2)
    return g0 * (1 - 2 * h / A_WGS * (1 + 1 / 298.257223563 + 0.00344978600308 - 2 / 298.257223563 * s2)
                 + 3 * h * h / A_WGS ** 2)


def nose_up_dcm(heading_deg):
    """C_b2n for the nose up (body x = -D), the belly (body z) toward heading_deg from north."""
    p = math.radians(heading_deg)
    x = np.array([0.0, 0.0, -1.0])
    z = np.array([math.cos(p), math.sin(p), 0.0])
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1)


def dcm_to_quat(c):
    """Scalar first, from a proper rotation matrix (body -> NED)."""
    w = math.sqrt(max(0.0, 1 + c[0, 0] + c[1, 1] + c[2, 2])) / 2
    x = math.sqrt(max(0.0, 1 + c[0, 0] - c[1, 1] - c[2, 2])) / 2
    y = math.sqrt(max(0.0, 1 - c[0, 0] + c[1, 1] - c[2, 2])) / 2
    z = math.sqrt(max(0.0, 1 - c[0, 0] - c[1, 1] + c[2, 2])) / 2
    x = math.copysign(x, c[2, 1] - c[1, 2])
    y = math.copysign(y, c[0, 2] - c[2, 0])
    z = math.copysign(z, c[1, 0] - c[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


# ------------------------------------------------------------------ the board's stream

FC_MESSAGE = 4002   # the V10's own frames to the board (aided-receiver plan, P1): not the board's output


def read_frames(path):
    """Every CRC-good RTCM3 frame of the board's output in a .rtcm3 file or a V10 OC log, in order. The V10's 4002
    frames, which the board's recorder and a bench V10's 0x95 records hold beside the board's since P1, are left out:
    they are neither the board's epochs nor its UART's bytes."""
    if path.endswith(".bin"):
        good, _stats = XR.extract(__import__("pathlib").Path(path))
        return [bytes(f) for _t, f in good if XR.message_type(bytes(f)) != FC_MESSAGE]
    data = open(path, "rb").read()
    out, i = [], 0
    while i + 6 <= len(data):
        if data[i] != 0xD3:
            i += 1
            continue
        n = ((data[i + 1] & 3) << 8) | data[i + 2]
        f = data[i:i + 6 + n]
        if len(f) == 6 + n and XR.frame_ok(f):
            if XR.message_type(f) != FC_MESSAGE:
                out.append(bytes(f))
            i += 6 + n
        else:
            i += 1
    return out


def tr_week_tow(f):
    """A 4001's sub-type, GPS week and time of week (ms): payload bits 12 type, 4 sub, 16 week, 30 tow."""
    b = int.from_bytes(f[3:11], "big")
    return (b >> 48) & 0xF, (b >> 32) & 0xFFFF, (b >> 2) & ((1 << 30) - 1)


def timed_4001(f):
    """A 4001 that carries the GPS time: the epoch (sub-type 1) and the fix (2). The status before the first fix (3),
    which the board's recorder keeps since P1, has none: its bits there are not a week and a time of week."""
    return XR.message_type(f) == XR.TR_MESSAGE and len(f) >= 14 and tr_week_tow(f)[0] in (1, 2)


def frame_epochs(frames):
    """Each frame's epoch as GPS seconds: an MSM's own time; a timed 4001's; anything else (an ephemeris, a status) the
    last epoch's before it (the first one's for those ahead of every epoch). None where no week is known yet."""
    week = next((tr_week_tow(f)[1] for f in frames if timed_4001(f)), None)
    if week is None:
        raise SystemExit("no 4001 in the stream: not the gnss-sdr-dev board's")
    out, last = [], None
    for f in frames:
        tow = XR.msm_gps_tow(f)
        if timed_4001(f):
            _sub, week, tow = tr_week_tow(f)
        if tow is not None:
            g = week * 604800.0 + tow * 1e-3
            if last is not None and g < last - 302400.0:     # the week turned before its 4001 said so
                g += 604800.0
            last = g
            out.append(g)
        else:
            out.append(last)
    first = next(g for g in out if g is not None)
    return [first if g is None else g for g in out]


# ------------------------------------------------------------------ the events

def build(a):
    cfg = json.load(open(a.config))
    tr = Trajectory(cfg)
    if a.pad:
        tr.pad = tuple(float(x) for x in a.pad.split(","))
        tr.hk = tr.hk - tr.hk[0] + tr.pad[2]
    if a.start_gpst is not None:
        tr.gps_start_s = a.start_gpst
    ign, burn, apo = tr.phases()
    lat_r = math.radians(tr.pad[0])
    if a.field_ned_ut:
        field = np.array([float(x) for x in a.field_ned_ut.split(",")])
        decl = math.radians(a.declination_deg) if a.declination_deg is not None else math.atan2(field[1], field[0])
    elif tuple(round(x, 6) for x in tr.pad) == FILES_PAD:
        field, decl = np.array(FILES_FIELD[0]), FILES_FIELD[1]
    else:
        raise SystemExit(f"pad {tr.pad}: give --field-ned-ut N,E,D (TR_GeoMag::fieldNED_uT there)")
    C_b2n = nose_up_dcm(a.heading_deg)
    C_n2b = C_b2n.T
    q_truth = dcm_to_quat(C_b2n)
    w_n = OMEGA_E * np.array([math.cos(lat_r), 0.0, -math.sin(lat_r)])
    rng = np.random.default_rng(a.seed)

    def mcu_us(file_s):
        return np.rint((np.asarray(file_s, float) + T_OFS) * 1e6).astype(np.int64)

    if a.stream == "none":                          # the IMU alone (a check of it against the truth)
        frames, gps = [], []
    else:
        frames = read_frames(a.stream)
        gps = frame_epochs(frames)
    f_s = np.array(gps) - tr.gps_start_s            # file time of each frame's epoch
    inside = (f_s >= -1.0) & (f_s <= tr.duration + 1.0)
    if frames and not inside.any():
        raise SystemExit(f"none of the stream's {len(frames)} frames falls in the file's time "
                         f"(GPS {tr.gps_start_s:.0f} .. +{tr.duration:.0f} s; the stream's {gps[0]:.0f} .. {gps[-1]:.0f})")
    t_end = min(tr.duration, float(f_s[inside].max()) + 1.0) if frames else tr.duration
    flight = ign is not None
    if not flight:                                  # a static file: flight time 0 is its start
        ign = 0.0
    ev = []

    # R: each frame after its epoch, the UART time of the bytes before it in that epoch, at the next poll
    byte_s = 10.0 / a.baud
    poll_us = int(a.poll_ms * 1000)
    poll_phase = int(rng.integers(0, poll_us)) if poll_us > 0 else 0
    cum, prev = 0, None
    n_r = 0
    for f, g, fs, ok in zip(frames, gps, f_s, inside):
        if not ok:
            continue
        cum = cum + len(f) if g == prev else len(f)
        prev = g
        ready = int(mcu_us(fs)) + int(round((a.delay_ms * 1e-3 + cum * byte_s) * 1e6))
        if poll_us > 0:
            ready = poll_phase + -(-(ready - poll_phase) // poll_us) * poll_us
        if XR.message_type(f) in SDR_EPH and fs < 0:
            ready = int(mcu_us(0.0))
        ev.append((ready, 5, f"R {ready} {f.hex()}"))
        n_r += 1

    # I, B: the EKF ticks and the barometer on alternate rows of twice the EKF rate
    rows = np.arange(0.0, t_end, 0.5 / a.ekf_rate)
    h, v, acc = tr.at(rows)
    if a.imu_tick_mean:
        # each tick's specific force the mean over the tick before it (the velocity's change over it), as the filter
        # integrates a sample; the instant's (the default) puts each of the thrust's steps one tick early
        _h0, v0, _a0 = tr.at(rows - 1.0 / a.ekf_rate)
        acc = (v - v0) * a.ekf_rate
    b_acc = np.zeros(3) if a.clean else rng.normal(0.0, a.acc_bias, 3)
    b_gyr = np.zeros(3) if a.clean else rng.normal(0.0, a.gyro_bias, 3)
    tu = mcu_us(rows)
    ign_us = int(mcu_us(ign))
    for k in range(0, len(rows), 2):
        v_n = np.array([0.0, 0.0, -v[k]])
        f_n = np.array([0.0, 0.0, -acc[k]]) + 2.0 * np.cross(w_n, v_n) - np.array([0.0, 0.0, gravity(lat_r, h[k])])
        fb = C_n2b @ f_n + b_acc
        wb = C_n2b @ w_n + b_gyr
        if not a.clean:
            fb = fb + rng.normal(0.0, a.acc_noise, 3)
            wb = wb + rng.normal(0.0, a.gyro_noise, 3)
        flags = 0x02 if flight and tu[k] >= ign_us else 0x01
        ev.append((int(tu[k]), 4, f"I {tu[k]} {fb[0]:.5f} {fb[1]:.5f} {fb[2]:.5f} {wb[0]:.7f} {wb[1]:.7f} "
                                  f"{wb[2]:.7f} {flags}"))
    # the barometer: the standard atmosphere at the height (the pad's ellipsoidal height taken as MSL), to any height
    P0 = pressure_pa(tr.pad[2])
    for k in range(1, len(rows), 2):
        hb = h[k] + (0.0 if a.clean else rng.normal(0.0, a.baro_noise))
        ev.append((int(tu[k]), 0, f"B {tu[k]} {pressure_pa(hb):.3f}"))

    # M: the field in the body at 100 Hz
    for ts in np.arange(0.0, t_end, 0.01):
        mb = C_n2b @ field + (0.0 if a.clean else rng.normal(0.0, 0.3, 3))
        u = int(mcu_us(ts))
        ev.append((u, 1, f"M {u} {mb[0]:.4f} {mb[1]:.4f} {mb[2]:.4f}"))
    ev.sort(key=lambda e: (e[0], e[1]))

    # the truth, at 100 Hz
    nav = os.path.splitext(a.out)[0] + ".truth.nav"
    tt = np.arange(0.0, t_end, 0.01)
    ht, vt, _ = tr.at(tt)
    with open(nav, "w") as fh:
        fh.write(f"# sdr_rig_events.py truth of {os.path.basename(a.config)}: t_us vn ve vd qw qx qy qz h "
                 "(ellipsoidal; MCU us, flight time = t_us/1e6 + META t_base_s)\n")
        q = q_truth
        fh.write("".join(f"{u} 0.0000 0.0000 {-vv:.4f} {q[0]:.7f} {q[1]:.7f} {q[2]:.7f} {q[3]:.7f} {hh:.3f}\n"
                         for u, vv, hh in zip(mcu_us(tt), vt, ht)))

    ign_mcu = ign + T_OFS
    meta = [f"# sdr_rig_events.py {os.path.basename(a.config)} {os.path.basename(a.stream)}",
            f"META t_base_s {-float(ign_mcu)!r}",
            "META state_machine none",
            f"META ekf_rate {a.ekf_rate:.3f}",
            f"META ground_pressure_pa {P0:.3f}",
            f"META pad_lla {tr.pad[0]!r} {tr.pad[1]!r} {tr.pad[2]!r}",
            f"META decimal_year {tr.decimal_year:.4f}",
            f"META declination_rad {float(decl)!r}",
            f"META field_ned_ut {float(field[0])!r} {float(field[1])!r} {float(field[2])!r}",
            f"META t_inflight_us {ign_us if flight else -1}",
            f"META sdr_rig_config {os.path.basename(a.config)}",
            f"META sdr_rig_truth {os.path.basename(nav)}",
            f"META sdr_rig_gps_start_s {tr.gps_start_s!r}",
            f"META sdr_rig_arrival delay_ms {a.delay_ms} poll_ms {a.poll_ms} baud {a.baud}",
            f"META sdr_rig_imu {'clean' if a.clean else 'noisy'} acc_bias {' '.join(f'{x:.4f}' for x in b_acc)} "
            f"gyro_bias {' '.join(f'{x:.6f}' for x in b_gyr)} heading_deg {a.heading_deg}",
            f"META truth_ignition_s 0.0",
            f"META truth_burnout_s {float(burn - ign) if burn is not None else -1.0!r}",
            f"META truth_apogee_s {float(apo - ign) if apo is not None else -1.0!r}",
            f"META truth_end_s {float(t_end - ign)!r}",
            f"META sdr_frames {n_r} of {len(frames)} (the rest outside the file's time)"]
    if a.seed_truth:
        meta += [f"META seed_lla {tr.pad[0]!r} {tr.pad[1]!r} {tr.pad[2]!r}",
                 "META seed_quat " + " ".join(repr(float(x)) for x in q_truth),
                 "META seed_quat_truth 1"]
    with open(a.out, "w") as fh:
        fh.write("\n".join(meta) + "\n")
        fh.write("\n".join(e[2] for e in ev) + "\n")
    n_i = sum(1 for e in ev if e[1] == 4)
    print(f"{os.path.basename(a.out)}: {n_i} IMU ticks at {a.ekf_rate:g}/s over {t_end:.1f} s of the file, "
          f"{n_r} of the stream's {len(frames)} frames, ignition at file {ign:.1f} s, burnout T+"
          f"{(burn - ign) if burn is not None else float('nan'):.2f}, apogee T+"
          f"{(apo - ign) if apo is not None else float('nan'):.2f}; truth -> {os.path.basename(nav)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config", help="the SignalSim config the rig played (cocom-rig/sdr/signalsim/configs/*.json)")
    ap.add_argument("stream", help="the board's RTCM3: a .rtcm3 (sdrdev.py rec-get) or a V10 OC log (.bin); "
                    "none: the IMU alone")
    ap.add_argument("out", help="the events file (its truth beside it: OUT.truth.nav)")
    ap.add_argument("--ekf-rate", type=float, default=500.0, help="I lines a second (the flight loop's EKF ticks)")
    ap.add_argument("--delay-ms", type=float, default=3.0, help="the board's epoch to its first byte out")
    ap.add_argument("--poll-ms", type=float, default=25.0, help="the flight computer's UART reads (0: none)")
    ap.add_argument("--baud", type=float, default=2.0e6)
    ap.add_argument("--heading-deg", type=float, default=0.0, help="the belly's (body z) azimuth from north")
    ap.add_argument("--clean", action="store_true", help="no IMU, barometer or magnetometer noise or bias")
    ap.add_argument("--imu-tick-mean", action="store_true",
                    help="each IMU tick's specific force the mean over the tick before it, not the instant's")
    ap.add_argument("--acc-noise", type=float, default=0.03, help="m/s^2 an EKF tick")
    ap.add_argument("--gyro-noise", type=float, default=0.0015, help="rad/s an EKF tick")
    ap.add_argument("--acc-bias", type=float, default=0.03, help="m/s^2, 1-sigma, drawn once an axis")
    ap.add_argument("--gyro-bias", type=float, default=0.002, help="rad/s, 1-sigma, drawn once an axis")
    ap.add_argument("--baro-noise", type=float, default=0.1, help="m")
    ap.add_argument("--seed", type=int, default=1, help="the noise's")
    ap.add_argument("--seed-truth", action="store_true", help="the cold start from the truth (seed_lla, seed_quat)")
    ap.add_argument("--field-ned-ut", help="N,E,D uT (default: the B1C files' pad's)")
    ap.add_argument("--declination-deg", type=float)
    ap.add_argument("--pad", help="lat,lon,h: another pad than the config's (a bench check)")
    ap.add_argument("--start-gpst", type=float, help="the file's start as GPS seconds instead of the config's")
    build(ap.parse_args())


if __name__ == "__main__":
    main()
