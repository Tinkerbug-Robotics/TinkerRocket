#!/usr/bin/env python3
"""Turn one OC flight log (.bin) into the event stream raw_replay (the C++ harness) reads.

    extract.py OC_LOG OUT.txt [--ref-log OC_LOG_WITH_PAD_CONTEXT]
    extract.py --sbf OC_LOG OUT.txt          # a V8/V9 log with the mosaic-G5's SBF (raw_replay --receiver mosaic)

What the flight computer's raw task was handed, rebuilt from the log:
  I  the IMU ticks: the flight loop's drain-window means (integer-rounded raw counts, every
     EKF_DECIMATION-th 1 kHz window at the log's own EKF tick rate), converted as SensorConverter
     does, FLU -> FRD, with the tick's flags (level, in flight, landed)
  B  barometer samples (time, pressure) -- the harness applies the ground pressure and BaroGatePolicy
  M  magnetometer samples, body FRD, uT
  R  the receiver's RTCM3 frames as they arrived (0x95), MSM7 and ephemerides
  K  the GPS -> MCU clock lines (0x9D), each from the time it was set (then the record's seq and restarts)
  F  the receiver's fixes (GPS week, time of week), for an MSM epoch's week
  V  the same fixes as the flight loop's GPS voter gets them: fix mode, satellites, height, velocity N E U
     (raw_replay's apogee quorum, --quorum; ignored otherwise)
  S  the start: the raw filter's own state at the log's first record (0x96 + 0x9B)
  P  the pad context (0x9E): its state at the launch tick, covariance, ephemerides, ionosphere

With --sbf (a V8/V9 log with the mosaic-G5, whose raw filter left no records of its own yet): no S, A or P
lines -- raw_replay starts the filter on the pad itself (--cold-start) -- and R carries the receiver's SBF blocks
as they arrived (0x95), its own fix (PVTGeodetic) among them (so no V lines). The blocks go over byte for byte: a
Detailed GNSS log's MeasEpoch carries every band the receiver tracks, which raw_replay --rr-multiband combines; META
sbf_signals counts them by SBF signal number (a line raw_replay does not read, so a file with it replays exactly as
one without). Every navigation block in the log
(GPSNav, GALNav, BDSNav, GPSIon) is put at the start as well, marked "preload", unless --nav-as-logged: the logs
begin long after the receiver did, and it sends a satellite's ephemeris only when it changes. raw_replay files a
preloaded issue only once a receiver could have had it (the RINEX rule: a GPS set 2 h before its toe, a Galileo batch
665 s after, a BeiDou set 18 s after), so an issue decoded later in the log is not handed over early. Use
--nav-as-logged where the flight computer booted inside the log (its store then held only what the log shows) and
for a comparison with the on-board filter's own records (0x96) when that is so. The flight's time is the log's
in-flight edge (T+0), or the log's own clock where it has none. A day's broadcast RINEX file goes to raw_replay
itself (--rinex).

Times are the FC's microseconds, unwrapped to 64 bits.
"""
import argparse
import datetime as dt
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
def _repo():
    """The repository: the one this script sits in (tools/raw_replay), else $RAW_REPLAY_WT."""
    up = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    if os.path.isdir(os.path.join(up, "Data_Analysis")):
        return up
    return os.environ.get("RAW_REPLAY_WT", up)


WT = _repo()
sys.path.insert(0, os.path.join(WT, "Data_Analysis"))
import plot_flight_data_mini as P  # noqa: E402

INFLIGHT, LANDED = 3, 4
GPS_EPOCH = dt.datetime(1980, 1, 6)
LEAP_S = 18


def unwrap(t, ref=None):
    """u32 microsecond stamps -> int64, unwrapped near ref (the first IMU sample)."""
    a = np.asarray(t, dtype=np.int64)
    if ref is None:
        ref = int(a[0])
    d = (a - ref + 2 ** 31) % 2 ** 32 - 2 ** 31
    return ref + d


def imu_ticks(recs, config, ekf_rate, t_ref, imu_phase_us=0.0):
    """The flight loop's EKF ticks: 1 kHz logic windows (at the loop's achieved pace), every second one handed to
    the filter, its integer mean (imu_drain::roundedMean) converted as SensorConverter does. imu_phase_us moves the
    windows' grid: the loop's own grid is not in the log, and which half of the samples the filter got is the
    grid's (a shock's attitude through it; --imu-phase-us)."""
    imu = recs["ISM6HG256"]
    t = unwrap([r["time_us"] for r in imu], t_ref)
    lo = np.array([[r["low_acc_x"], r["low_acc_y"], r["low_acc_z"]] for r in imu])
    hi = np.array([[r["high_acc_x"], r["high_acc_y"], r["high_acc_z"]] for r in imu])
    gy = np.array([[r["gyro_x"], r["gyro_y"], r["gyro_z"]] for r in imu])
    near = np.array([r["low_g_near_rail"] for r in imu])
    if config["b2r_quat"] and any(abs(a - b) > 1e-9 for a, b in zip(config["b2r_quat"], (1, 0, 0, 0))):
        raise SystemExit("board->rocket rotation is not identity: not handled")
    s_lo, s_hi, s_gy = P.ism6_scales(config["low_g_fs_g"], config["high_g_fs_g"], config["gyro_fs_dps"])
    rot = math.radians(config["ism6_rot_z_deg"])
    c, s = math.cos(rot), math.sin(rot)
    hb = np.array(config["hg_bias"], float)

    def unrot(v):   # board -> sensor axes (inverse of the converter's +Z rotation)
        return np.stack([v[:, 0] * c + v[:, 1] * s, -v[:, 0] * s + v[:, 1] * c, v[:, 2]], axis=1)

    raw_lo = np.rint(unrot(lo) / s_lo).astype(np.int64)
    raw_hi = np.rint(unrot(hi + hb) / s_hi).astype(np.int64)
    raw_gy = np.rint(unrot(gy) / s_gy).astype(np.int64)

    # logic windows on a regular grid at twice the EKF rate (EKF_DECIMATION = 2); the EKF takes every second one
    period_us = 1e6 / (2.0 * ekf_rate)
    w = np.floor((t - t[0] + imu_phase_us) / period_us).astype(np.int64)
    take = (w % 2) == 1
    idx = np.flatnonzero(take)
    if idx.size == 0:
        raise SystemExit("no IMU windows")
    ww = w[idx]
    starts = np.flatnonzero(np.r_[True, ww[1:] != ww[:-1]])
    ends = np.r_[starts[1:], idx.size]

    def rmean(sum_, n):
        half = n // 2
        q = np.where(sum_ >= 0, (sum_ + half) // n, -((-sum_ + half) // n))
        return q

    out = []
    for a, b in zip(starts, ends):
        sel = idx[a:b]
        n = len(sel)
        tf, tl = int(t[sel[0]]), int(t[sel[-1]])
        tc = tf + (tl - tf) // 2
        m_lo = rmean(raw_lo[sel].sum(axis=0), n)
        m_hi = rmean(raw_hi[sel].sum(axis=0), n)
        m_gy = rmean(raw_gy[sel].sum(axis=0), n)
        nr = bool(near[sel].any())
        if nr:
            v = m_hi * s_hi
            acc = np.array([v[0] * c - v[1] * s - hb[0], v[0] * s + v[1] * c - hb[1], v[2] - hb[2]])
        else:
            v = m_lo * s_lo
            acc = np.array([v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2]])
        g = m_gy * s_gy
        gyr = np.array([g[0] * c - g[1] * s, g[0] * s + g[1] * c, g[2]])
        acc = acc.astype(np.float32)
        out.append((tc, float(acc[0]), -float(acc[1]), -float(acc[2]),
                    float(np.float32(gyr[0]) * np.float32(math.pi / 180.0)),
                    float(np.float32(-gyr[1]) * np.float32(math.pi / 180.0)),
                    float(np.float32(-gyr[2]) * np.float32(math.pi / 180.0))))
    return out


def gps_of_fix(r):
    """A GNSS record's GPS week and time of week (ms), from its UTC date and time."""
    utc = dt.datetime(r["year"], r["month"], r["day"], r["hour"], r["minute"], r["second"]) \
        + dt.timedelta(milliseconds=r["milli_sec"])
    g = (utc - GPS_EPOCH).total_seconds() + LEAP_S
    week = int(g // 604800)
    return week, int(round((g - week * 604800) * 1000.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("out")
    ap.add_argument("--ekf-rate", type=float, help="the flight loop's EKF ticks a second (default: the log's)")
    ap.add_argument("--imu-phase-us", type=float, default=0.0,
                    help="the flight loop's IMU window grid moved by this much (us): which half of the samples the "
                         "filter took depends on it, and the log does not say (an ensemble over it)")
    ap.add_argument("--blob", help="the replay file the image carried: its ignition edge puts the log on the "
                                   "flight's time (default tools/replay/sim_replay.bin; with --sbf, only with --rig); "
                                   "none: a real flight, T+0 at the logged accelerometer's own ignition edge")
    ap.add_argument("--borrow", help="for a log without a pad context (an OC image that dropped 0x9B-0x9F): "
                                     "another run's event file (the same replay, the same bench heading error) to "
                                     "take the pad's covariance, ephemerides, ionosphere and IMU biases from")
    ap.add_argument("--clock-offset-ms", type=float, default=6.0,
                    help="without clock lines (0x9D): the fixes' stamps sit this far above the driver's line "
                         "(their lower envelope, measured on r20-r25)")
    ap.add_argument("--sbf", action="store_true",
                    help="a V8/V9 log with the mosaic-G5's SBF: no raw-filter records needed, the receiver's blocks "
                         "as R lines (raw_replay --receiver mosaic --cold-start)")
    ap.add_argument("--nav-as-logged", action="store_true",
                    help="with --sbf: the navigation blocks only where the log has them, not also at the start (a "
                         "log the flight computer booted in)")
    ap.add_argument("--rig", action="store_true",
                    help="with --sbf: a rig replay of the mosaic-only image (tools/rig_replay/run_v9.sh): the flight's "
                         "time from the replay's ignition edge (--blob) against the log's accelerometer, and the start "
                         "from the on-board filter's own records (S, A, the pad context) -- the OC log begins at "
                         "launch, so a cold start would begin in the boost")
    ap.add_argument("--seed-quat-from-log", action="store_true",
                    help="with --sbf: the cold start's attitude from the log's own flight filter (NonSensor q, the "
                         "auto image's GpsInsEKF), not the accelerometer's tilt and the pad heading guess the "
                         "mosaic-only image starts from (META seed_quat). Its heading is then that filter's: never "
                         "score heading against that filter's quaternion from such a run")
    args = ap.parse_args()
    if args.rig and not args.sbf:
        raise SystemExit("--rig is for a mosaic log (--sbf); an LC86G rig log takes --blob alone")
    if args.sbf:
        return extract_sbf(args)
    if args.blob is None:
        args.blob = DEFAULT_BLOB

    recs, _stats, config = P.parse_binary_file(args.log)
    imu = recs["ISM6HG256"]
    t_ref = int(imu[0]["time_us"])
    ns = recs["NonSensor"]
    nt = unwrap([r["time_us"] for r in ns], t_ref)
    if args.ekf_rate:
        ekf_rate = args.ekf_rate
    else:
        ek = np.array([r["ekf_ticks"] for r in ns], dtype=np.int64)
        ekf_rate = float((np.diff(ek) % 65536).sum() / ((nt[-1] - nt[0]) * 1e-6))
    lines = []
    lines.append(f"# extract.py {os.path.basename(args.log)}")
    lines.append(f"META ekf_rate {ekf_rate:.3f}")
    # the flight's time: the replay's ignition edge (the blob) against the log's accelerometer (truth.py)
    os.environ.setdefault("COCOM_SDR", os.path.join(WT, "gnss", "cocom-rig", "sdr"))
    sys.path.insert(0, os.path.join(WT, "tools", "rig_replay"))
    import truth as TR  # noqa: E402
    if args.blob == "none":
        edge = 0.0   # a real flight: T+0 at the logged accelerometer's own ignition edge
    else:
        with open(args.blob, "rb") as f:
            edge = TR.blob_edge_t(f.read())
    base_s = TR.log_time_base(recs, edge)   # flight s = log's (u32) MCU s + base
    # the unwrapped stamps here are the u32 ones while the log does not wrap
    lines.append(f"META t_base_s {float(base_s)!r}")
    lines.append(f"META edge_s {float(edge)!r}")

    # ---- the flight computer's state, each loop pass (NonSensor): in flight, apogee, landed
    state = np.array([r["rocket_state"] for r in ns])
    apo = np.array([bool(r.get("apogee_flag", False)) for r in ns])
    landed = np.array([bool(r.get("alt_landed", False)) for r in ns]) | (state == LANDED)
    pcx = P.join_pad_context(recs.get("PadContext") or [])
    k3 = np.flatnonzero(state == INFLIGHT)
    t_inflight = int(nt[k3[0]]) if k3.size else None
    if pcx and pcx["nav"]:
        t_inflight = int(unwrap([pcx["nav"]["filter_time_us"]], t_ref)[0])   # the shadow's own launch tick
    ka = np.flatnonzero(apo)
    t_apogee = int(nt[ka[0]]) if ka.size else None
    kl = np.flatnonzero(landed)
    t_landed = int(nt[kl[0]]) if kl.size else None
    kl4 = np.flatnonzero(state == LANDED)
    t_state_landed = int(nt[kl4[0]]) if kl4.size else None
    lines.append(f"META t_inflight_us {t_inflight if t_inflight is not None else -1}")
    lines.append(f"META t_apogee_flag_us {t_apogee if t_apogee is not None else -1}")
    lines.append(f"META t_landed_flag_us {t_landed if t_landed is not None else -1}")
    lines.append(f"META t_state_landed_us {t_state_landed if t_state_landed is not None else -1}")

    # ---- the ground pressure the flight loop froze (Snapshot)
    snap = recs.get("Snapshot") or []
    gp = [r["ground_pressure_pa"] for r in snap if r.get("ground_pressure_pa", 0) > 0]
    lines.append(f"META ground_pressure_pa {repr(float(gp[0])) if gp else -1}")

    lines.extend(start_lines(recs, args, t_ref, pcx, gp))
    lines.extend(antenna_lines(pcx))

    events = []
    # ---- IMU ticks with the loop's flags
    ticks = imu_ticks(recs, config, ekf_rate, t_ref, args.imu_phase_us)
    ki = np.searchsorted(nt, [tk[0] for tk in ticks], side="right") - 1
    for tk, k in zip(ticks, ki):
        k = max(int(k), 0)
        if pcx and pcx["nav"]:
            inf = tk[0] >= t_inflight and state[k] != LANDED     # the shadow's own launch tick
        else:
            inf = state[k] == INFLIGHT
        lvl = (not inf) or bool(apo[k])
        lnd = bool(landed[k])
        flags = (1 if lvl else 0) | (2 if inf else 0) | (8 if lnd else 0)
        events.append((tk[0], 0, "I %d %r %r %r %r %r %r %d" % (tk[0], *tk[1:], flags)))
    # ---- barometer
    for r in recs.get("BMP585") or []:
        tu = int(unwrap([r["time_us"]], t_ref)[0])
        events.append((tu, 1, f"B {tu} {r['pressure_pa']!r}"))
    # ---- magnetometer (rocket FLU -> FRD)
    for key in ("IIS2MDC", "MMC5983MA"):
        for r in recs.get(key) or []:
            tu = int(unwrap([r["time_us"]], t_ref)[0])
            events.append((tu, 2, f"M {tu} {r['mag_x']!r} {-r['mag_y']!r} {-r['mag_z']!r}"))
    # ---- the receiver's fixes: week and time of week (F), and as the flight loop's GPS voter gets them (V)
    for r in recs.get("GNSS") or []:
        if r["year"] < 2020:
            continue
        week, tow_ms = gps_of_fix(r)
        tu = int(unwrap([r["time_us"]], t_ref)[0])
        events.append((tu, 3, f"F {tu} {week} {tow_ms} {r['fix_mode']} {r['num_sats']}"))
        events.append((tu, 3, f"V {tu} {r['fix_mode']} {r['num_sats']} {r['alt_m']!r} {r['vel_n']!r} "
                              f"{r['vel_e']!r} {r['vel_u']!r}"))
    # ---- the clock lines (or, without them, one fitted to the fixes' lower envelope)
    if not recs.get("GnssClock"):
        fx = [r for r in recs.get("GNSS") or [] if r["year"] >= 2020 and r["fix_mode"] >= 2]
        if len(fx) >= 20:
            g = np.array([gps_of_fix(r)[0] * 604800 + gps_of_fix(r)[1] * 1e-3 for r in fx])
            st = unwrap([r["time_us"] for r in fx], t_ref) * 1e-6
            g0 = g[0]
            x = g - g0
            y = st - x
            # the lowest stamp of each 5 s, a line through them, then down by the measured offset
            b = np.floor(x / 5.0)
            xs, ys = [], []
            for k in np.unique(b):
                m = b == k
                j = np.argmin(y[m])
                xs.append(x[m][j]); ys.append(y[m][j])
            drift, a_s = np.polyfit(xs, ys, 1)
            a_s -= args.clock_offset_ms * 1e-3
            tu = int(unwrap([fx[0]["time_us"]], t_ref)[0])
            events.append((tu - 1, 4, f"K {tu - 1} {float(g0)!r} 0 {float(a_s * 1.0)!r} 0.0 {float(drift)!r} 0.0 1"))
            lines.append(f"META clock_fitted drift_ppm {drift * 1e6:.2f} from {len(fx)} fixes")
    for r in recs.get("GnssClock") or []:
        tu = int(unwrap([r["time_us"]], t_ref)[0])
        events.append((tu, 4, f"K {tu} {r['gps0_s']!r} {int(r['mcu0_us'])} {r['a_s']!r} {r['t_ref_s']!r} "
                              f"{r['drift']!r} {r['latency_s']!r} {1 if r['valid'] else 0} {r['seq']} {r['restarts']}"))
    # ---- the receiver's RTCM3 frames
    frames, dropped = P.join_gnss_rtcm(recs.get("GNSS_RTCM") or [])
    lines.append(f"META rtcm_frames {len(frames)} dropped {dropped}")
    for tu32, fr in frames:
        tu = int(unwrap([tu32], t_ref)[0])
        events.append((tu, 5, f"R {tu} {bytes(fr).hex()}"))

    events.sort(key=lambda e: (e[0], e[1]))
    lines.extend(e[2] for e in events)
    with open(args.out, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"{os.path.basename(args.log)}: {len(ticks)} IMU ticks at {ekf_rate:.1f}/s, {len(frames)} RTCM frames "
          f"({dropped} dropped), inflight {t_inflight}, apogee flag {t_apogee}, landed {t_landed}, "
          f"pad context {'missing ' + ','.join(pcx['missing']) if pcx and pcx['missing'] else ('yes' if pcx else 'none')}")


def antenna_lines(pcx):
    """The GNSS antenna's offset on the flight (the pad context's PCX_ANT, BLE cmd 81), in the filter's body frame,
    for raw_replay. The one the navigation filter flew with (PCX_ANT's applied) is META antenna_m, which it takes
    where --antenna-m gives none; a setting the filter did not fly with (it was still on an earlier offset, or had
    none on) is META antenna_m_set, which it only names. META antenna_used is 1 when the filter flew with the
    setting. A log without the record, or with neither known, gives none."""
    ant = pcx.get("ant") if pcx else None
    if not ant or not (ant["known"] or ant["applied_known"]):
        return []
    xyz = lambda v: " ".join(repr(float(x)) for x in v)  # noqa: E731
    lines = []
    if ant["applied_known"]:
        lines.append(f"META antenna_m {xyz(ant['applied_body_m'])}")
    if ant["known"] and not ant["used"]:
        lines.append(f"META antenna_m_set {xyz(ant['body_m'])}")
    lines.append(f"META antenna_used {1 if ant['used'] else 0}")
    return lines


def start_lines(recs, args, t_ref, pcx, gp):
    """The S, A and pad-context lines: the raw filter's own state at its first running pad record (0x96 and its 0x9B
    companion) as the start, each epoch's age when it closed it, and the pad context (0x9E) or a borrowed one.
    gp: the ground pressures the Snapshot records hold (for the barometer's offset at the start)."""
    lines = []
    # ---- the start: the raw filter's first record on the pad (0x96 and its 0x9B companion)
    sh = recs["RawGnssEkf"]
    att = {r["time_us"]: r for r in recs.get("RawGnssAtt") or []}
    s0 = next((r for r in sh if r["running"] and not r["in_flight"]), None)
    if s0 is None:
        raise SystemExit("no running pad record (0x96)")
    a0 = att.get(s0["time_us"])
    t0 = int(unwrap([s0["time_us"]], t_ref)[0])
    d2r = math.pi / 180.0
    ab = (a0["acc_bias_x"], a0["acc_bias_y"], a0["acc_bias_z"]) if a0 else (0.0, 0.0, 0.0)
    wb = tuple(x * d2r for x in ((a0["gyro_bias_x_dps"], a0["gyro_bias_y_dps"], a0["gyro_bias_z_dps"])
                                 if a0 else (0.0, 0.0, 0.0)))
    sig = [s0["pos_sigma_n"], s0["pos_sigma_e"], s0["pos_sigma_d"], s0["vel_sigma_n"], s0["vel_sigma_e"],
           s0["vel_sigma_d"]]
    asig = [a0["att_sigma_n_deg"], a0["att_sigma_e_deg"], a0["heading_sigma_deg"]] if a0 else [-1, -1, -1]
    bsig = ([a0["acc_bias_sigma_x"], a0["acc_bias_sigma_y"], a0["acc_bias_sigma_z"], a0["gyro_bias_sigma_x_dps"],
             a0.get("gyro_bias_sigma_y_dps", a0["gyro_bias_sigma_x_dps"]),
             a0.get("gyro_bias_sigma_z_dps", a0["gyro_bias_sigma_x_dps"])] if a0 else [-1] * 6)
    lines.append("S " + " ".join(repr(float(x)) for x in (
        [t0, s0["lat_deg"] * d2r, s0["lon_deg"] * d2r, s0["alt_m"], s0["vel_n"], s0["vel_e"], s0["vel_d"],
         s0["q0"], s0["q1"], s0["q2"], s0["q3"], *ab, *wb, s0["clk_bias_m"], s0["clk_drift_mps"], s0["isb_gal_m"],
         s0["isb_bds_m"], 1 if s0["clock_set"] else 0, 1 if a0 else 0] + sig + asig + bsig)))

    # ---- the barometer's offset at the start (the filter's height less the barometer's), for a borrowed context
    bmp = recs.get("BMP585") or []
    if gp and bmp:
        bt = unwrap([r["time_us"] for r in bmp], t_ref)
        sel = (bt >= t0) & (bt < t0 + 500000)
        if sel.any():
            pa = np.array([r["pressure_pa"] for r in bmp])[sel]
            alt = 44330.0 * (1.0 - (pa / gp[0]) ** (1.0 / 5.255))
            lines.append(f"META baro_offset_start_m {float(s0['alt_m'] - alt.mean())!r}")

    # ---- each epoch's age when the on-board filter closed it (0x96): the harness hands its epochs over to match
    last = None
    for r in sh:
        if r["running"] and r["epoch_tow_s"] != last and r["epoch_tow_s"] > 0:
            last = r["epoch_tow_s"]
            lines.append(f"A {int(round(r['epoch_tow_s'] * 1000))} {r['epoch_age_s'] * 1000.0:.1f}")

    # ---- the pad context
    donor = None
    if args.borrow and not (pcx and pcx["nav"]):
        donor = [ln.rstrip("\n") for ln in open(args.borrow)]
        took = [ln for ln in donor if ln.split(" ", 1)[0] in ("PNAV", "PCOV", "PSUR", "PEPH")]
        # the barometer's offset is this flight's own (the donor's receiver put its pad elsewhere)
        bo = [ln for ln in lines if ln.startswith("META baro_offset_start_m ")]
        for i, ln in enumerate(took):
            if ln.startswith("PNAV ") and bo:
                f = ln.split()
                f[1 + 21] = bo[0].split()[2]
                took[i] = " ".join(f)
        lines.extend(took)
        lines.append(f"META pcx_borrowed {os.path.basename(args.borrow)}")
        if a0 is None:
            ds = next(ln for ln in donor if ln.startswith("S "))
            dv = ds.split()[1:]
            sv = lines[[i for i, ln in enumerate(lines) if ln.startswith("S ")][0]].split()[1:]
            for k in list(range(11, 17)) + list(range(29, 38)):
                sv[k] = dv[k]                                 # the donor's IMU biases and their sigmas
            sv[22] = "1.0"                                     # biases present now
            i = [i for i, ln in enumerate(lines) if ln.startswith("S ")][0]
            lines[i] = "S " + " ".join(sv)
            lines.append("META biases_borrowed 1")
    if pcx and pcx["nav"]:
        n = pcx["nav"]
        lines.append("PNAV " + " ".join(repr(float(x)) for x in (
            [int(unwrap([n["filter_time_us"]], t_ref)[0]), n["lat_rad"], n["lon_rad"], n["height_m"], *n["vel_ned"],
             *n["quat"], *n["acc_bias_mps2"], *n["gyro_bias_rps"], n["clk_bias_m"], n["clk_drift_mps"],
             n["isb_gal_m"], n["isb_bds_m"], n["baro_offset_m"], n["declination_rad"], n["field_ut"],
             *n["iono"], n["iono_gps_day"], {"none": 0, "broadcast": 1, "pad survey": 2}.get(n["iono_source"], 0),
             n["flags"]])))
        if pcx["cov"] is not None:
            C = pcx["cov"]
            lines.append("PCOV " + " ".join(repr(float(C[i][j])) for i in range(19) for j in range(i, 19)))
        if pcx["survey"] is not None:
            sv = pcx["survey"]
            lines.append(f"PSUR {sv['tracked']} {sv['settling']} {sv['corrected']} {int(sv['pad_ok'])} "
                         f"{sv['rms_m']!r} {sv['pad_offset_m']!r} " + " ".join(f"{a} {b} {c!r}" for a, b, c in
                                                                                  sv["corrections"]))
        for fr in pcx["eph_frames"]:
            lines.append("PEPH " + bytes(fr).hex())
        lines.append("META pcx_missing " + (",".join(pcx["missing"]) or "none"))
    else:
        lines.append("META pcx_missing all")
    return lines

SBF_NAV_BLOCKS = (5891, 4002, 4081, 5893)   # GPSNav, GALNav, BDSNav, GPSIon
SBF_MEAS_EPOCH = 4027


def sbf_signal_counts(blocks):
    """Each SBF signal number's count over the MeasEpochs among blocks (the main antenna; the reference guide's
    4.2.1 layout, as SbfMeas.cpp reads it: Type1 sub-blocks with their Type2s, the signal from SigIdxLo or, at 31,
    ObsInfo's bits 3-7 plus 32), and the MeasEpochs. What the R lines carry for raw_replay --rr-multiband: a Detailed
    log's MeasEpoch holds every band the receiver tracks; a Reduced log's, once a second, cannot feed the 10 Hz grid."""
    counts = {}
    n_meas = 0
    for b in blocks:
        if len(b) < 20 or b[0] != 0x24 or b[1] != 0x40 or (int.from_bytes(b[4:6], "little") & 0x1FFF) != SBF_MEAS_EPOCH:
            continue
        length = min(int.from_bytes(b[6:8], "little"), len(b))
        n1, sb1, sb2 = b[14], b[15], b[16]
        n_meas += 1
        at = 20
        for _ in range(n1):
            if sb1 < 20 or at + sb1 > length:
                break
            typ, obs, n2 = b[at + 1], b[at + 18], b[at + 19]
            lo = typ & 0x1F
            sig = (obs >> 3) + 32 if lo == 31 else lo
            if typ >> 5 == 0:
                counts[sig] = counts.get(sig, 0) + 1
            at += sb1
            for _ in range(n2):
                if sb2 < 12 or at + sb2 > length:
                    break
                t2, o2 = b[at], b[at + 5]
                lo2 = t2 & 0x1F
                sig2 = (o2 >> 3) + 32 if lo2 == 31 else lo2
                if t2 >> 5 == 0:
                    counts[sig2] = counts.get(sig2, 0) + 1
                at += sb2
    return n_meas, counts
DEFAULT_BLOB = os.path.join(HERE, "..", "replay", "sim_replay.bin")


def common_events(recs, config, ekf_rate, t_ref, inflight_of):
    """The I, B, M, F and K events every log gives the harness. inflight_of(tick_us, k) says whether the loop was in
    flight at an IMU tick (k: its NonSensor record)."""
    ns = recs["NonSensor"]
    state = np.array([r["rocket_state"] for r in ns])
    apo = np.array([bool(r.get("apogee_flag", False)) for r in ns])
    landed = np.array([bool(r.get("alt_landed", False)) for r in ns]) | (state == LANDED)
    nt = unwrap([r["time_us"] for r in ns], t_ref)
    events = []
    ticks = imu_ticks(recs, config, ekf_rate, t_ref, args.imu_phase_us)
    ki = np.searchsorted(nt, [tk[0] for tk in ticks], side="right") - 1
    for tk, k in zip(ticks, ki):
        k = max(int(k), 0)
        inf = inflight_of(tk[0], k)
        lvl = (not inf) or bool(apo[k])
        lnd = bool(landed[k])
        flags = (1 if lvl else 0) | (2 if inf else 0) | (8 if lnd else 0)
        events.append((tk[0], 0, "I %d %r %r %r %r %r %r %d" % (tk[0], *tk[1:], flags)))
    for r in recs.get("BMP585") or []:
        tu = int(unwrap([r["time_us"]], t_ref)[0])
        events.append((tu, 1, f"B {tu} {r['pressure_pa']!r}"))
    for key in ("IIS2MDC", "MMC5983MA"):
        for r in recs.get(key) or []:
            tu = int(unwrap([r["time_us"]], t_ref)[0])
            events.append((tu, 2, f"M {tu} {r['mag_x']!r} {-r['mag_y']!r} {-r['mag_z']!r}"))
    for r in recs.get("GNSS") or []:
        if r["year"] < 2020:
            continue
        week, tow_ms = gps_of_fix(r)
        tu = int(unwrap([r["time_us"]], t_ref)[0])
        events.append((tu, 3, f"F {tu} {week} {tow_ms} {r['fix_mode']} {r['num_sats']}"))
    for r in recs.get("GnssClock") or []:
        tu = int(unwrap([r["time_us"]], t_ref)[0])
        events.append((tu, 4, f"K {tu} {r['gps0_s']!r} {int(r['mcu0_us'])} {r['a_s']!r} {r['t_ref_s']!r} "
                              f"{r['drift']!r} {r['latency_s']!r} {1 if r['valid'] else 0} {r['seq']} {r['restarts']}"))
    return events, len(ticks)


def extract_sbf(args):
    """--sbf: a V8/V9 log with the mosaic-G5's SBF, for raw_replay --receiver mosaic (a cold start on the pad)."""
    recs, _stats, config = P.parse_binary_file(args.log)
    imu = recs["ISM6HG256"]
    t_ref = int(imu[0]["time_us"])
    ns = recs["NonSensor"]
    nt = unwrap([r["time_us"] for r in ns], t_ref)
    if args.ekf_rate:
        ekf_rate = args.ekf_rate
    else:
        ek = np.array([r["ekf_ticks"] for r in ns], dtype=np.int64)
        ekf_rate = float((np.diff(ek) % 65536).sum() / ((nt[-1] - nt[0]) * 1e-6))
    state = np.array([r["rocket_state"] for r in ns])
    apo = np.array([bool(r.get("apogee_flag", False)) for r in ns])
    landed = np.array([bool(r.get("alt_landed", False)) for r in ns]) | (state == LANDED)
    k3 = np.flatnonzero(state == INFLIGHT)
    t_inflight = int(nt[k3[0]]) if k3.size else None
    ka = np.flatnonzero(apo)
    t_apogee = int(nt[ka[0]]) if ka.size else None
    kl = np.flatnonzero(landed)
    t_landed = int(nt[kl[0]]) if kl.size else None
    if args.rig:
        # a rig replay: the flight's time from the replay's ignition edge, as an LC86G rig log's (truth.py)
        os.environ.setdefault("COCOM_SDR", os.path.join(WT, "gnss", "cocom-rig", "sdr"))
        sys.path.insert(0, os.path.join(WT, "tools", "rig_replay"))
        import truth as TR  # noqa: E402
        with open(args.blob or DEFAULT_BLOB, "rb") as f:
            edge = TR.blob_edge_t(f.read())
        base = [f"META t_base_s {float(TR.log_time_base(recs, edge))!r}", f"META edge_s {float(edge)!r}"]
    else:
        # flight time = MCU s + this: T+0 at the in-flight edge, the log's own clock without one
        base = [f"META t_base_s {(-t_inflight * 1e-6 if t_inflight is not None else 0.0)!r}"]
    lines = [f"# extract.py --sbf {os.path.basename(args.log)}",
             f"META ekf_rate {ekf_rate:.3f}",
             *base,
             f"META t_inflight_us {t_inflight if t_inflight is not None else -1}",
             f"META t_apogee_flag_us {t_apogee if t_apogee is not None else -1}",
             f"META t_landed_flag_us {t_landed if t_landed is not None else -1}"]
    snap = recs.get("Snapshot") or []
    gp = [r["ground_pressure_pa"] for r in snap if r.get("ground_pressure_pa", 0) > 0]
    lines.append(f"META ground_pressure_pa {repr(float(gp[0])) if gp else -1}")
    if args.seed_quat_from_log:
        # the log's flight filter's attitude over its first 50 records with a unit quaternion, one sign
        qs = [np.array([r["q0"], r["q1"], r["q2"], r["q3"]]) for r in ns]
        qs = [q for q in qs if abs(np.linalg.norm(q) - 1.0) < 0.01][:50]
        if not qs:
            raise SystemExit("--seed-quat-from-log: no unit quaternion in the log's NonSensor records")
        qm = sum(q if np.dot(q, qs[0]) >= 0 else -q for q in qs)
        qm = qm / np.linalg.norm(qm)
        lines.append("META seed_quat " + " ".join(repr(float(x)) for x in qm))

    inflight_of = lambda tu, k: bool(state[k] == INFLIGHT)  # noqa: E731
    pcx = P.join_pad_context(recs.get("PadContext") or [])
    lines.extend(antenna_lines(pcx))
    if args.rig:
        # the on-board filter's start (S, A) and pad context, as an LC86G log's; in flight from the filter's own
        # launch tick where the pad context gives it, as there
        lines.extend(start_lines(recs, args, t_ref, pcx, gp))
        if pcx and pcx["nav"]:
            t_launch = int(unwrap([pcx["nav"]["filter_time_us"]], t_ref)[0])
            inflight_of = lambda tu, k: tu >= t_launch and state[k] != LANDED  # noqa: E731
    events, n_ticks = common_events(recs, config, ekf_rate, t_ref, inflight_of)
    frames, dropped = P.join_gnss_rtcm(recs.get("GNSS_RTCM") or [])
    raw = P.split_gnss_raw_frames(frames)
    sbf = raw["sbf"]
    if not sbf:
        raise SystemExit("no SBF blocks in this log (an LC86G log: extract.py without --sbf)")
    lines.append(f"META sbf_blocks {len(sbf)} dropped {dropped} bad {raw['bad']}")
    # what the MeasEpochs carry, signal by signal (raw_replay keeps a META key it does not read and replays as before)
    n_meas, sig_counts = sbf_signal_counts([b for _t, b in sbf])
    lines.append("META sbf_signals " + (" ".join(f"{k}:{v}" for k, v in sorted(sig_counts.items())) or "none")
                 + f" meas_epochs {n_meas}")
    t0 = min(e[0] for e in events) if events else int(unwrap([sbf[0][0]], t_ref)[0])
    n_nav = 0
    if not args.nav_as_logged:
        # every navigation block of the log, just before its first event (in log order), marked: raw_replay files
        # each issue when a receiver could first have had it
        per = {n: 0 for n in SBF_NAV_BLOCKS}
        for _tu32, b in sbf:
            if P.sbf_block_number(b) in SBF_NAV_BLOCKS:
                events.append((t0 - 1, -1, f"R {t0 - 1} {bytes(b).hex()} preload"))
                n_nav += 1
                per[P.sbf_block_number(b)] += 1
        lines.append(f"META nav_preloaded {n_nav} gps {per[5891]} gal {per[4002]} bds {per[4081]} ion {per[5893]}")
    else:
        lines.append("META nav_preloaded 0")
    for tu32, b in sbf:
        tu = int(unwrap([tu32], t_ref)[0])
        events.append((tu, 5, f"R {tu} {bytes(b).hex()}"))
    events.sort(key=lambda e: (e[0], e[1]))
    lines.extend(e[2] for e in events)
    with open(args.out, "w") as f:
        f.write("\n".join(lines) + "\n")
    names = {}
    for _t, b in sbf:
        nm = P.sbf_block_name(b)
        names[nm] = names.get(nm, 0) + 1
    top = ", ".join(f"{k} {v}" for k, v in sorted(names.items(), key=lambda kv: -kv[1])[:6])
    print(f"{os.path.basename(args.log)}: {n_ticks} IMU ticks at {ekf_rate:.1f}/s, {len(sbf)} SBF blocks "
          f"({dropped} frames dropped, {raw['bad']} bad; {top}), {n_nav} nav blocks preloaded, inflight "
          f"{t_inflight}, apogee flag {t_apogee}, landed {t_landed}; MeasEpoch signals (main antenna) "
          + (", ".join(f"{k} {v}" for k, v in sorted(sig_counts.items())) or "none"))


if __name__ == "__main__":
    main()
