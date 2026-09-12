#!/usr/bin/env python3
"""Replay real flight sensor data through the EKF.

Parses a binary flight log, feeds IMU/GNSS/baro/mag packets to the
EKF at native timestamps, and compares EKF output against GNSS truth.

Usage:
    python replay_flight_ekf.py <binary_file> [--plot-dir DIR]
"""

import sys, math, argparse, json, subprocess
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from plot_flight_data_mini import (parse_binary_file, get_array,
                                   pressure_to_altitude)

from tinkerrocket_sim._ekf import (GpsInsEKF, IMUData, GNSSDataLLA,
                                    MagData, BaroData, declination_rad)

G_MS2 = 9.80665
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

# Low-g saturation threshold: ±(16 - 0.5)g = ±15.5g
LOW_G_SAT_THRESH = 15.5 * G_MS2

# #1190 shock gate — flight_computer/main/config.h EKF_SHOCK_*; keep in step.
# Saturation is the ONLY criterion, judged per SENSOR axis on raw counts by the
# FC drain loop over EVERY sample (imu_drain_window.h) and passed to the filter
# as EkfIMUData.gyro_railed / accel_railed — the filter tests no magnitude of
# its own.  This replay does the same over the logged samples between two EKF
# ticks, on the raw counts recovered from the parser's SI values.  The bars in
# LSB are FS-independent (the ST gyro's nominal FS is 28571 LSB at every
# setting; the accelerometers map FS onto 32768).
SHOCK_GYRO_RAIL_FRAC = 0.95     # of the nominal gyro full scale
SHOCK_ACCEL_RAIL_FRAC = 0.95    # of the high-g accelerometer full scale
SHOCK_SETTLE_MS = 50            # hold after the last trip
GYRO_RAIL_LSB = int(SHOCK_GYRO_RAIL_FRAC / 0.035e-3)          # 27142 = 3800 dps at ±4000
ACCEL_RAIL_LSB = int(SHOCK_ACCEL_RAIL_FRAC * 32768)           # 31129 = 243 g at ±256
LOW_G_NEAR_RAIL_LSB = int((16.0 - 0.5) / 16.0 * 32768)        # 31744, the #1191 switch bar

# #1214 (squash commit on main) changed what the flight loop hands the EKF:
# the drain-window MEAN of every sample since the last tick, stamped at the
# window centre, instead of the newest drained sample.  A log written before
# it is only reproduced by --imu-feed newest; the version guard says which.
MEAN_FEED_COMMIT = "60efb324"


# Firmware pad heading (flight_computer/main/config.h: PAD_HEADING_DEG).
PAD_HEADING_DEG = 0.0

# Firmware EKF rate, in Hz. THE REPLAY MUST MATCH THIS OR IT IS NOT A REPLAY.
#
# This is a RATE, not a sample count, and that distinction is the whole point.
# The flight loop drains the IMU queue every iteration and LOGS every sample, but
# it hands the EKF only `ism6_latest_si` — the newest one — and only every
# EKF_DECIMATION-th loop. So the EKF integrates a small fraction of what is in the
# log, and which fraction depends on the loop rate, not on the IMU ODR:
#
#   loop ≈ 980 Hz, config::EKF_DECIMATION = 2  ->  EKF ≈ 490 Hz
#   (firmware confirms this directly: "[TIMING] ekf: cnt=489")
#
# Counting logged IMU samples instead gets this wrong, and the error MOVES as the
# IMU ODR changes:
#   * 2026-06 logs: IMU 907 Hz -> every 2nd sample ≈ 454 Hz. Right by luck.
#   * post-#474  : IMU 3840 Hz -> every 2nd sample ≈ 1920 Hz. 4x too fast: the
#     EKF sees 8x the samples the vehicle's did, fires every AHRS/mag/GNSS/baro
#     correction 4x too often, and converges a bias the firmware never had
#     (measured: a phantom 0.77 m/s² accel bias on the nose axis = a 4.5° tilt of
#     the gravity reference, showing up as a rock-steady 4.7° attitude offset).
#
# Driving the EKF on a clock instead of a sample counter is right for both, and
# stays right the next time the IMU ODR moves.
#
# #529 retired the hand-maintained rate: the firmware logs a free-running EKF
# update-tick counter (NonSensorData.ekf_ticks, uint16 wrap), and
# derive_ekf_rate_hz() below recovers the ACHIEVED rate from it — per log, no
# constant to track (a real collect measured 474 Hz against the 490 target).
# This value remains only as the fallback for logs that predate the field.
DEFAULT_EKF_RATE_HZ = 490.0


def derive_ekf_rate_hz(records):
    """Achieved EKF rate from the logged ekf_ticks counter (#529), or None.

    NonSensorData.ekf_ticks is a free-running uint16 count of actual EKF update
    ticks. Summing wrap-aware deltas over consecutive NonSensor records — and
    trimming the frozen head/tail (the counter sits at 0 until the EKF
    initializes, before the first good GNSS fix) — gives ticks/second as the
    vehicle actually ran, robust to loop-rate and EKF_DECIMATION changes.

    Returns None when the log predates the field (records carry ekf_ticks=None)
    or carries too little tick motion to trust.
    """
    ns = records.get("NonSensor") or []
    pts = [(r["time_us"], r["ekf_ticks"]) for r in ns
           if r.get("ekf_ticks") is not None]
    if len(pts) < 2:
        return None

    # Per consecutive pair: wrap-aware tick delta + wall time. A pair is
    # discarded (contributing neither ticks nor time) when time runs backwards,
    # spans a logging hole > 10 s, or the delta is implausibly large — a huge
    # "delta" is really the counter resetting across an in-flight reboot.
    pairs = []
    for (t0, k0), (t1, k1) in zip(pts, pts[1:]):
        dt_us = t1 - t0
        d_ticks = (k1 - k0) & 0xFFFF
        if dt_us <= 0 or dt_us > 10_000_000 or d_ticks > 4096:
            continue
        pairs.append((dt_us, d_ticks))

    # Trim to the window where the counter is actually moving, so the pre-init
    # frozen-at-0 stretch does not dilute the average. d == 0 pairs INSIDE the
    # window stay — they are the legitimate beat between the ~500 Hz NonSensor
    # cadence and the ~490 Hz EKF cadence, and dropping them would skew high.
    active = [i for i, (_, d) in enumerate(pairs) if d > 0]
    if not active:
        return None
    window = pairs[active[0]:active[-1] + 1]
    ticks = sum(d for _, d in window)
    dur_s = sum(dt for dt, _ in window) / 1e6
    if ticks < 100 or dur_s <= 0:
        return None                       # too little motion to trust
    rate = ticks / dur_s
    if not (50.0 <= rate <= 5000.0):
        return None                       # implausible — refuse, use fallback
    return rate

# RocketState::INFLIGHT (TR_RocketComputerTypes/RocketComputerTypes.h).
# INITIALIZATION=0, READY=1, PRELAUNCH=2, INFLIGHT=3, LANDED=4, MAG_CALIBRATION=5
ROCKET_STATE_INFLIGHT = 3


def _decimal_year(gnss_rec):
    """Decimal year from a GNSS fix, matching the firmware's day-of-year math
    (flight_computer/main.cpp, at EKF init).  Falls back to the WMM2025 epoch
    when the log carries no plausible date — same guard the firmware applies."""
    yr = int(gnss_rec.get("year", 0) or 0)
    if not (2020 <= yr <= 2035):
        return 2025.0                       # firmware falls back to a constant
    mo = max(1, min(12, int(gnss_rec.get("month", 1) or 1)))
    day = int(gnss_rec.get("day", 1) or 1)
    mdays = (31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31)
    doy = day + sum(mdays[:mo - 1])
    leap = (yr % 4 == 0 and (yr % 100 != 0 or yr % 400 == 0))
    if leap and mo > 2:
        doy += 1
    return yr + (doy - 1) / (366.0 if leap else 365.0)


def _nearest_nonsensor(records, t_us):
    """The NonSensor record closest in time to t_us, skipping absent (zero)
    quaternions.  Returns None if the log carries no usable NonSensor state."""
    ns = records.get("NonSensor") or []
    best, best_dt = None, None
    for r in ns:
        if (r["q0"]**2 + r["q1"]**2 + r["q2"]**2 + r["q3"]**2) < 0.25:
            continue                        # zeroed quaternion = "not populated"
        dt = abs(r["time_us"] - t_us)
        if best_dt is None or dt < best_dt:
            best, best_dt = r, dt
    return best


def _first_logged_quat(records, t_us):
    """The firmware's OWN attitude at log start — the correct seed for a replay
    (see the long note at the init site).  Normalised; None on a legacy log."""
    r = _nearest_nonsensor(records, t_us)
    if r is None:
        return None
    q = [r["q0"], r["q1"], r["q2"], r["q3"]]
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def _first_logged_vel(records, t_us):
    """The firmware's OWN NED velocity at log start.  The log stores ENU
    (e/n/u); the EKF wants NED.  None on a legacy log."""
    r = _nearest_nonsensor(records, t_us)
    if r is None:
        return None
    return (r["n_vel"], r["e_vel"], -r["u_vel"])


def _quat_from_accel_heading(acc_x_frd, acc_y_frd, acc_z_frd, heading_rad):
    """Body→NED quaternion from measured gravity + a known pad heading.

    A line-for-line port of the firmware's quatFromAccelHeading()
    (components/TR_Orientation/TR_Orientation.cpp). The FC calls this immediately
    after ekf.init() to coarse-align the pad attitude, because GpsInsEKF::initCore
    hard-codes a nose-up-vertical quaternion rather than deriving one from the
    sensors. A replay that omits this does not start where the vehicle started.

    Keep in step with the C++ — including the 80° roll gate, which exists because
    roll is ill-conditioned near vertical (the Y/Z accel components vanish there).
    """
    g_mag = math.sqrt(acc_x_frd**2 + acc_y_frd**2 + acc_z_frd**2)
    if g_mag < 0.1:
        g_mag = 9.807

    sx = max(-1.0, min(1.0, acc_x_frd / g_mag))
    pitch_rad = math.asin(sx)

    roll_rad = 0.0
    if abs(pitch_rad) < math.radians(80.0):
        roll_rad = math.atan2(-acc_y_frd, -acc_z_frd)

    cy, sy = math.cos(heading_rad * 0.5), math.sin(heading_rad * 0.5)
    cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
    cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def _first_logged_biases(records, t_us):
    """The firmware's own IMU biases and their covariance at the replay's start.

    #1412: the same #514 argument that seeds the quaternion and the velocity.
    The firmware initialises its filter on the first good fix, before logging
    starts, so by the first logged sample the biases have converged over a
    stretch that cannot be re-run — and unlike the attitude, a fresh filter's
    bias COVARIANCE is ~6200x the converged one, which is what lets GNSS and
    baro innovations drive the gyro bias to its clamp inside two seconds.

    FlightSnapshotData carries both (10 Hz).  Returns
    (gyro_rps, gyro_var, accel, accel_var) or None.  Note the parser hands back
    ekf_gyro_bias_* in DEG/S (deliberately — it is compared against a dps gyro)
    while p_gbias_* is the raw variance in (rad/s)^2, so only the state is
    converted here.
    """
    snaps = records.get("Snapshot") or []
    best = None
    for s in snaps:
        if s.get("time_us") is None or not s.get("ekf_initialized", True):
            continue
        if "ekf_gyro_bias_x" not in s or "p_gbias_x" not in s:
            continue
        if best is None or abs(s["time_us"] - t_us) < abs(best["time_us"] - t_us):
            best = s
    if best is None:
        return None
    g = tuple(math.radians(best[f"ekf_gyro_bias_{a}"]) for a in "xyz")
    a = tuple(best[f"ekf_accel_bias_{x}"] for x in "xyz")
    gv = max(float(best["p_gbias_x"]), float(best["p_gbias_y"]), float(best["p_gbias_z"]))
    av = max(float(best["p_abias_x"]), float(best["p_abias_y"]), float(best["p_abias_z"]))
    return g, gv, a, av


def build_event_list(records):
    """Merge all sensor records into a single time-sorted event list.

    Each event is (time_us, type_str, record_dict).
    """
    events = []
    for r in records["ISM6HG256"]:
        events.append((r["time_us"], "imu", r))
    for r in records["GNSS"]:
        events.append((r["time_us"], "gnss", r))
    for r in records["BMP585"]:
        events.append((r["time_us"], "baro", r))
    # Magnetometer: old PCB logs MMC5983MA, new PCB logs IIS2MDC. Both carry
    # mag_x/y/z (µT) post-parse, so the "mag" handler is identical.
    for r in records.get("MMC5983MA", []):
        events.append((r["time_us"], "mag", r))
    for r in records.get("IIS2MDC", []):
        events.append((r["time_us"], "mag", r))
    # #514: NonSensor carries the firmware's OWN rocket_state and apogee voters.
    # The AHRS accel gate keys off those, so the replay must follow the logged
    # state rather than re-deriving it from an accel heuristic (see the gate).
    for r in records.get("NonSensor", []):
        events.append((r["time_us"], "nonsensor", r))
    events.sort(key=lambda e: e[0])
    return events


def detect_flight_phases(records, t0_us):
    """Detect boost start/end from accel magnitude.

    Returns (boost_start_us, apogee_us) in absolute time_us.
    """
    imu = records["ISM6HG256"]
    imu_times = get_array(imu, "time_us")

    # Downsample to ~100 Hz for phase detection
    step = max(1, len(imu) // (len(imu) // 20))
    accel_mag = []
    for i in range(0, len(imu), step):
        r = imu[i]
        a = math.sqrt(r["low_acc_x"]**2 + r["low_acc_y"]**2 + r["low_acc_z"]**2)
        accel_mag.append((imu_times[i], a))

    # Boost: sustained accel > 3g
    boost_start = None
    boost_end = None
    for t_us, a in accel_mag:
        if a > 3.0 * G_MS2 and boost_start is None:
            boost_start = t_us
        if boost_start is not None and a < 2.0 * G_MS2 and boost_end is None:
            boost_end = t_us

    # Apogee: find GNSS peak altitude after boost
    gnss = records["GNSS"]
    if gnss and boost_start:
        gnss_times = get_array(gnss, "time_us")
        gnss_alt = np.array([g["alt_m"] for g in gnss])
        flight_mask = gnss_times > boost_start
        if flight_mask.any():
            peak_idx = np.argmax(gnss_alt[flight_mask])
            apogee_us = gnss_times[flight_mask][peak_idx]
        else:
            apogee_us = boost_end + 5_000_000 if boost_end else None
    else:
        apogee_us = None

    return boost_start, boost_end, apogee_us


# Repo root — this script lives in <repo>/Data_Analysis/.
_REPO_ROOT = Path(__file__).resolve().parent.parent

# The source this replay's EKF is compiled from. If any of these differ between
# the firmware's build commit and the current checkout, the replay is running a
# DIFFERENT filter than the one that wrote the log — the #515 trap, the thing
# that manufactured a phantom 50° "EKF error" on 2026-07-14. Kept explicit so the
# check stays honest if the component set grows.
_EKF_SRC = [
    "tinkerrocket-idf/components/TR_GpsInsEKF/TR_GpsInsEKF.cpp",
    "tinkerrocket-idf/components/TR_GpsInsEKF/TR_GpsInsEKF.h",
    "tinkerrocket-idf/components/TR_GeoMag/TR_GeoMag.cpp",
    "tinkerrocket-idf/components/TR_GeoMag/TR_GeoMag.h",
]


def _git(*args):
    """Run git in the repo. Returns (returncode, stdout) or (None, '') if git is
    unavailable. Never raises — a version check must not be able to break a replay."""
    try:
        p = subprocess.run(["git", "-C", str(_REPO_ROOT), *args],
                           capture_output=True, text=True, timeout=10)
        return p.returncode, p.stdout.strip()
    except Exception:
        return None, ""


def check_ekf_version(binary_file):
    """Compare the firmware that WROTE the log against the EKF this replay is built
    from, and print a verdict up front.

    The app writes a sidecar <name>.json next to <name>.bin carrying
    settings.fw_git_sha + fw_dirty (since #178). The .bin itself has no SHA, so
    before this the skew was undetectable — a stale replay looked authoritative.
    Now it checks whether the EKF/GeoMag source differs between the firmware's
    commit and the current checkout, and says so. Closes the detection half of #515
    with no firmware change. Returns a dict, or None if it can't be determined."""
    print("\n  ── Firmware / EKF version (#515 skew guard) ──")
    sidecar = Path(binary_file).with_suffix(".json")
    if not sidecar.exists():
        print(f"     no sidecar {sidecar.name} — firmware version UNKNOWN.")
        print("     Cannot check EKF skew; treat any attitude output with suspicion.")
        return None
    try:
        settings = json.loads(sidecar.read_text()).get("settings", {})
        fw_sha = settings.get("fw_git_sha")
        fw_dirty = bool(settings.get("fw_dirty", False))
    except Exception as e:
        print(f"     sidecar {sidecar.name} unreadable ({e}) — version UNKNOWN.")
        return None
    if not fw_sha:
        print(f"     sidecar {sidecar.name} has no fw_git_sha — version UNKNOWN.")
        return None

    print(f"     firmware built from {fw_sha}{'+dirty' if fw_dirty else ''}")
    dirty_note = ("  (build was DIRTY: uncommitted changes at flash time we can't "
                  "see — match not guaranteed)") if fw_dirty else ""

    rc, _ = _git("cat-file", "-e", f"{fw_sha}^{{commit}}")
    if rc is None:
        print("     git unavailable — cannot verify EKF skew.")
        return {"fw_sha": fw_sha, "fw_dirty": fw_dirty, "matched": None}
    if rc != 0:
        print(f"     commit {fw_sha} not in this repo (shallow clone / unmerged "
              "branch?) — cannot verify EKF skew.")
        return {"fw_sha": fw_sha, "fw_dirty": fw_dirty, "matched": None}

    # Which IMU feed wrote this log (see MEAN_FEED_COMMIT).
    rc_feed, _ = _git("merge-base", "--is-ancestor", MEAN_FEED_COMMIT, fw_sha)
    mean_feed = None if rc_feed is None else (rc_feed == 0)
    if mean_feed is True:
        print("     IMU feed of that firmware: drain-window MEAN (#1214) — "
              "replay with --imu-feed mean (the default)")
    elif mean_feed is False:
        print("     IMU feed of that firmware: NEWEST drained sample (pre-#1214) — "
              "replay with --imu-feed newest to reproduce it")

    # git diff <sha> -- <files>: lists EKF sources that differ between the
    # firmware's commit and the working tree the extension was built from.
    _, out = _git("diff", "--name-only", fw_sha, "--", *_EKF_SRC)
    differ = [ln for ln in out.splitlines() if ln.strip()]
    if not differ:
        print(f"     EKF source matches that commit — VERSION-MATCHED.{dirty_note}")
        return {"fw_sha": fw_sha, "fw_dirty": fw_dirty, "matched": True,
                "mean_feed": mean_feed}
    print("     *** EKF SOURCE DIFFERS from the firmware's commit — SKEW ***")
    for ln in differ:
        print(f"        changed: {ln}")
    print("     This replay runs a DIFFERENT filter than wrote the log. A fidelity")
    print("     FAIL below is EXPECTED, and its attitude output is not comparable.")
    print("     Skew is not cosmetic: an EKF change can rewrite the attitude wholesale.")
    print("     Measured on the 2026-08-29 logs (#1412) — #1304 made the mag fuse")
    print("     whenever it is valid, where the firmware that flew fused it only")
    print("     alongside a valid gravity reference. The flown filter therefore ran")
    print("     on gyro alone from launch to apogee; this one does not, and the")
    print("     replayed attitude is yanked 44 deg within 0.2 s of launch.")
    print("")
    print("     To match the firmware, build the WHOLE tool at its commit:")
    print(f"       git worktree add /tmp/replay-{fw_sha} {fw_sha}")
    print(f"       cd /tmp/replay-{fw_sha}/tinkerrocket-sim && "
          "TR_SKIP_GUIDANCE=1 python3 setup.py build_ext --inplace")
    print(f"       cd /tmp/replay-{fw_sha} && PYTHONPATH=\"$PWD/tinkerrocket-sim/src\" \\")
    print("         python3 Data_Analysis/replay_flight_ekf.py <log> --plot-dir /tmp/replay-plots")
    print("     Checking out only the EKF sources into THIS tree does not work: the")
    print("     pybind bindings and this script call EKF APIs that postdate an older")
    print("     log (17 of them at b0e4aeb — setShockGateSettle, setMagReference,")
    print("     setNoseFirstFlight, EkfIMUData::gyro_railed, ...), so the extension")
    print("     fails to compile. The worktree carries bindings and EKF together.")
    return {"fw_sha": fw_sha, "fw_dirty": fw_dirty, "matched": False,
            "mean_feed": mean_feed}


def _sensor_xy(x, y, rot_z_deg):
    """Board-frame X/Y back to the chip's own axes — the inverse of the
    converter's Z rotation (ISM6HG256_ROT_Z_DEG).  The firmware judges every
    rail per SENSOR axis on raw counts; with the -45° mount a single-axis rail
    reads 0.71× on two board axes and a body-frame test never sees it."""
    c = math.cos(math.radians(rot_z_deg))
    s = math.sin(math.radians(rot_z_deg))
    return x * c + y * s, -x * s + y * c


def _attach_raw_counts(records, config):
    """Recover each IMU record's logged int16 counts from the parser's SI values
    (the same inverse #1191's replay_imu_drain uses, round-trip checked) and hang
    them on the record as `_raw` = (lx, ly, lz, hx, hy, hz, gx, gy, gz).  The
    shock-gate verdicts are then judged exactly as the drain window judges them:
    per sensor axis, in LSB, against the firmware's bars.  Returns False (and
    attaches nothing) when the inverse cannot be trusted, in which case the
    verdicts fall back to the SI per-sensor-axis test."""
    imu = records.get("ISM6HG256") or []
    if not imu:
        return False
    try:
        from replay_imu_drain import conversion_matrices, recover_raw
        cm = conversion_matrices(config)
        raw = recover_raw(imu, cm)          # sys.exit()s if the round trip fails
    except (ImportError, KeyError, TypeError, ValueError, SystemExit) as e:
        print(f"  raw-count recovery unavailable ({e}); shock-gate verdicts from SI values")
        return False
    for r, row in zip(imu, raw):
        r["_raw"] = row
    return True


def _ekf_imu_from_window(win, rec, rot_z_deg, imu_feed, shock_gate, si_bars):
    """The EKF's IMU input for one tick, from every logged sample since the
    previous tick (`win`, ending in `rec`).

    imu_feed="mean"   — firmware since #1214: the window mean of every channel,
                        stamped at the window centre; the low-g/high-g switch
                        on the window's WORST sample per sensor axis.
    imu_feed="newest" — firmware before #1214: this sample alone, the switch
                        on its body-frame values at 15.5 g.
    The #1190 shock-gate verdicts — a gyro or high-g accelerometer SENSOR axis
    at its rail in any sample of the window — are judged in both modes, on the
    recovered raw counts when `_raw` is attached (the firmware's own LSB bars)
    and otherwise on SI values un-rotated to the sensor axes against
    `si_bars` = (gyro_rail_dps, accel_rail_mps2).  Not raised when the gate
    is off.
    """
    lg_rail = gy_rail = hg_rail = False
    gyro_rail_dps, accel_rail_mps2 = si_bars
    for r in win:
        raw = r.get("_raw")
        if raw is not None:
            a = [abs(int(v)) for v in raw]
            if max(a[0:3]) > LOW_G_NEAR_RAIL_LSB:
                lg_rail = True
            if max(a[6:9]) >= GYRO_RAIL_LSB:
                gy_rail = True
            if max(a[3:6]) >= ACCEL_RAIL_LSB:
                hg_rail = True
            continue
        sx, sy = _sensor_xy(r["low_acc_x"], r["low_acc_y"], rot_z_deg)
        if max(abs(sx), abs(sy), abs(r["low_acc_z"])) > LOW_G_SAT_THRESH:
            lg_rail = True
        gx, gy = _sensor_xy(r["gyro_x"], r["gyro_y"], rot_z_deg)
        if max(abs(gx), abs(gy), abs(r["gyro_z"])) >= gyro_rail_dps:
            gy_rail = True
        hx, hy = _sensor_xy(r["high_acc_x"], r["high_acc_y"], rot_z_deg)
        if max(abs(hx), abs(hy), abs(r["high_acc_z"])) >= accel_rail_mps2:
            hg_rail = True

    if imu_feed == "mean":
        n = float(len(win))
        def mean(k):
            return sum(r[k] for r in win) / n
        lx, ly, lz = mean("low_acc_x"), mean("low_acc_y"), mean("low_acc_z")
        hx, hy, hz = mean("high_acc_x"), mean("high_acc_y"), mean("high_acc_z")
        gx, gy, gz = mean("gyro_x"), mean("gyro_y"), mean("gyro_z")
        t0, t1 = win[0]["time_us"], win[-1]["time_us"]
        stamp = t0 + (t1 - t0) // 2
        near_rail = lg_rail
    else:
        lx, ly, lz = rec["low_acc_x"], rec["low_acc_y"], rec["low_acc_z"]
        hx, hy, hz = rec["high_acc_x"], rec["high_acc_y"], rec["high_acc_z"]
        gx, gy, gz = rec["gyro_x"], rec["gyro_y"], rec["gyro_z"]
        stamp = rec["time_us"]
        near_rail = (abs(lx) > LOW_G_SAT_THRESH or abs(ly) > LOW_G_SAT_THRESH
                     or abs(lz) > LOW_G_SAT_THRESH)

    ax, ay, az = (hx, hy, hz) if near_rail else (lx, ly, lz)

    # Board frame (FLU) → EKF body frame (FRD)
    imu_d = IMUData()
    imu_d.time_us = stamp
    imu_d.acc_x = ax               # X same
    imu_d.acc_y = -ay              # FLU Y=Left → FRD Y=Right
    imu_d.acc_z = -az              # FLU Z=Up → FRD Z=Down
    imu_d.gyro_x = gx              # deg/s — the EKF converts internally
    imu_d.gyro_y = -gy             # FLU→FRD
    imu_d.gyro_z = -gz             # FLU→FRD
    imu_d.gyro_railed = bool(shock_gate and gy_rail)
    imu_d.accel_railed = bool(shock_gate and hg_rail)
    return imu_d


def _spans(t_us, flag):
    """Contiguous runs where `flag` is true, as (start_us, end_us) pairs."""
    spans = []
    start = None
    for t, f in zip(t_us, flag):
        if f and start is None:
            start = t
        elif not f and start is not None:
            spans.append((start, t))
            start = None
    if start is not None:
        spans.append((start, t_us[-1]))
    return spans


def replay(binary_file, plot_dir=None, align_baro=True, imu_feed="mean",
           shock_gate=True):
    mode = ("aligned baro+GNSS frame (the FIX)" if align_baro
            else "pad-relative baro (firmware behaviour, the BUG)")
    if imu_feed not in ("auto", "mean", "newest"):
        raise ValueError(f"imu_feed must be 'auto', 'mean' or 'newest', not {imu_feed!r}")
    print(f"Parsing: {binary_file}")
    print(f"  Baro frame mode: {mode}")
    print(f"  Shock gate (#1190): {'ON' if shock_gate else 'OFF (pre-fix filter)'}")
    records, stats, config = parse_binary_file(str(binary_file))
    print(f"  Frames: {stats['good_crc']:,} good, {stats['bad_crc']} bad CRC")
    # Mag: old PCB logs MMC5983MA, new PCB logs IIS2MDC — only one is populated.
    # (The old line counted MMC only, so a new-PCB log read "Mag: 0" even while
    # the EKF was fusing thousands of IIS2MDC samples — misleading on every new
    # board.) Report the total and name the chip that actually logged.
    n_iis = len(records.get('IIS2MDC', []))
    n_mmc = len(records.get('MMC5983MA', []))
    mag_chip = "IIS2MDC" if n_iis >= n_mmc else "MMC5983MA"
    print(f"  IMU: {len(records['ISM6HG256']):,}  GNSS: {len(records['GNSS']):,}  "
          f"Baro: {len(records['BMP585']):,}  Mag: {n_iis + n_mmc:,} ({mag_chip})")

    ver = check_ekf_version(binary_file)

    # The guard already worked out which feed the firmware used; follow it.
    # The old default was a fixed "mean", so replaying any pre-#1214 log took
    # the guard's own advice line and then ignored it.
    if imu_feed == "auto":
        mean_feed = (ver or {}).get("mean_feed")
        if mean_feed is None:
            imu_feed = "mean"
            feed_why = "guard could not tell — assuming the current firmware"
        else:
            imu_feed = "mean" if mean_feed else "newest"
            feed_why = "chosen from the firmware's commit"
    else:
        feed_why = "forced on the command line"
    print(f"  IMU feed: {'drain-window MEAN (firmware since #1214)' if imu_feed == 'mean' else 'NEWEST drained sample (firmware before #1214)'}"
          f" — {feed_why}")

    # #529: EKF cadence — from the log itself when the firmware recorded its
    # tick counter; the hand-maintained constant only as a legacy fallback.
    ekf_rate_hz = derive_ekf_rate_hz(records)
    if ekf_rate_hz is not None:
        print(f"  EKF rate: {ekf_rate_hz:.1f} Hz (derived from the logged "
              f"ekf_ticks counter)")
    else:
        ekf_rate_hz = DEFAULT_EKF_RATE_HZ
        print(f"  EKF rate: {ekf_rate_hz:.0f} Hz (constant fallback — log "
              f"predates the #529 ekf_ticks field; verify it matches the "
              f"firmware that flew)")
    ekf_period_us = 1e6 / ekf_rate_hz

    # Detect flight phases
    imu_times = get_array(records["ISM6HG256"], "time_us")
    t0_us = imu_times[0]
    boost_start, boost_end, apogee_us = detect_flight_phases(records, t0_us)
    if boost_start:
        print(f"  Boost: {(boost_start-t0_us)/1e6:.2f}s - {(boost_end-t0_us)/1e6:.2f}s")
        print(f"  Apogee: {(apogee_us-t0_us)/1e6:.2f}s")
    else:
        print("  WARNING: No boost detected")

    # Build time-sorted event list
    events = build_event_list(records)
    print(f"  Total events: {len(events):,}")

    # ---- Initialize EKF ----
    ekf = GpsInsEKF()
    ekf_initialized = False
    next_ekf_us = None          # firmware-rate EKF clock (see ekf_rate_hz above)

    # #1190: the shock gate's settle window as the FC configures it at boot
    # (config.h EKF_SHOCK_SETTLE_MS); the saturation verdicts are judged per
    # sensor axis in _ekf_imu_from_window, on recovered raw counts when the
    # parser's conversion inverts cleanly.  Gate off = no verdict is ever raised.
    ekf.set_shock_gate_settle(SHOCK_SETTLE_MS * 1000)
    gyro_fs_dps = float(config.get("gyro_fs_dps") or 4000)
    high_g_fs_g = float(config.get("high_g_fs_g") or 256)
    si_bars = (SHOCK_GYRO_RAIL_FRAC * gyro_fs_dps,
               SHOCK_ACCEL_RAIL_FRAC * high_g_fs_g * G_MS2)
    if shock_gate:
        have_raw = _attach_raw_counts(records, config)
        print(f"  Shock gate bars (per sensor axis): gyro ≥ {GYRO_RAIL_LSB} LSB "
              f"({si_bars[0]:.0f} dps), high-g accel ≥ {ACCEL_RAIL_LSB} LSB "
              f"({SHOCK_ACCEL_RAIL_FRAC * high_g_fs_g:.0f} g), settle {SHOCK_SETTLE_MS} ms; "
              f"verdicts from {'recovered raw counts' if have_raw else 'SI values'}")
    rot_z_deg = float(config["ism6_rot_z_deg"]) if config.get("ism6_rot_z_deg") is not None else -45.0
    imu_win = []                # every logged IMU sample since the last EKF tick
    tick_t = []                 # per EKF tick: stamp and whether the gate held
    tick_held = []
    # Firmware flight state, tracked from the log (drives the AHRS accel gate).
    # Default to a non-INFLIGHT state so a log with no NonSensor records behaves
    # like the pad — AHRS on — rather than silently disabling the gravity update.
    log_rocket_state = ROCKET_STATE_INFLIGHT - 1
    # Master voted apogee_flag as logged (apogee_flags bit 2, #142/#143) —
    # followed live, exactly like the firmware reads kinematics.apogee_flag.
    log_apogee_master = False
    log_has_master = False
    # Fallback for pre-#143 logs (42/43-byte NonSensor): latched OR of the two
    # voters that byte layout carried.
    log_apogee_latched = False

    # Track latest sensor data for EKF
    latest_gnss = None
    latest_mag = None
    last_gnss_time_us = 0
    gnss_counter = 0  # monotonic counter for EKF new-data detection

    # Track latest baro reference pressure (from pad)
    baro_ref_pa = None
    baro_samples_for_ref = []
    baro_alt_offset = 0.0  # offset to align baro (pad=0) with GNSS altitude frame
    baro_alt_offset_set = False

    # Recording arrays
    log_time_us = []
    log_ekf_lat = []
    log_ekf_lon = []
    log_ekf_alt = []
    log_ekf_vn = []
    log_ekf_ve = []
    log_ekf_vd = []
    log_ekf_roll = []
    log_ekf_pitch = []
    log_ekf_yaw = []
    log_ekf_q = []
    log_ekf_gyro_bias = []
    log_ekf_accel_bias = []
    log_cov_pos = []
    log_cov_vel = []
    log_cov_att = []
    log_held = []               # #1190: the gate was holding at this sample

    # GNSS truth arrays
    gnss_log_time = []
    gnss_log_lat = []
    gnss_log_lon = []
    gnss_log_alt = []
    gnss_log_vn = []
    gnss_log_ve = []
    gnss_log_vd = []

    # Baro truth
    baro_log_time = []
    baro_log_alt = []

    n_imu = 0
    n_gnss_updates = 0
    n_baro_updates = 0

    print("\nReplaying...")
    for time_us, etype, rec in events:
        t_rel = (time_us - t0_us) / 1e6

        if etype == "gnss":
            # Skip packets without a solid fix (matches onboard GNSS_MIN_SATS)
            has_fix = rec.get("num_sats", 0) >= 4
            if has_fix:
                # Record GNSS truth only when fix is valid
                gnss_log_time.append(time_us)
                gnss_log_lat.append(rec["lat"])
                gnss_log_lon.append(rec["lon"])
                gnss_log_alt.append(rec["alt_m"])
                gnss_log_vn.append(rec["vel_n"])
                gnss_log_ve.append(rec["vel_e"])
                gnss_log_vd.append(-rec["vel_u"])  # Up → Down

            # De-duplicate: only count as new if fix is valid and the GPS
            # fix timestamp (second + milli_sec) changed.  The MCU time_us
            # is different every poll even for the same receiver fix.
            if has_fix and (latest_gnss is None or
                    rec["second"] != latest_gnss["second"] or
                    rec["milli_sec"] != latest_gnss["milli_sec"]):
                latest_gnss = rec
                gnss_counter += 1

        elif etype == "mag":
            latest_mag = rec

        elif etype == "nonsensor":
            # Track the firmware's own flight state — the AHRS gate reads it.
            log_rocket_state = rec["rocket_state"]
            if rec.get("has_apogee_flags"):
                # #529: the master voted apogee_flag is in the log — follow it.
                log_apogee_master = rec["apogee_flag"]
                log_has_master = True
            if rec["alt_apogee"] or rec["vel_apogee"]:
                log_apogee_latched = True

        elif etype == "baro":
            # Collect pad baro samples for reference pressure (first 20 samples)
            if baro_ref_pa is None:
                baro_samples_for_ref.append(rec["pressure_pa"])
                if len(baro_samples_for_ref) >= 20:
                    baro_ref_pa = np.mean(baro_samples_for_ref)
                    print(f"  Baro ref pressure: {baro_ref_pa:.2f} Pa "
                          f"(from {len(baro_samples_for_ref)} samples at t={t_rel:.2f}s)")
                continue

            # Defer baro offset until we have a valid GNSS fix.
            #   align_baro=True  -> add GNSS alt so baro shares the EKF's
            #                       absolute-hMSL state frame (the FIX).
            #   align_baro=False -> leave offset 0, feeding pad-relative
            #                       altitude — reproduces the firmware bug at
            #                       flight_computer/main/main.cpp:2812.
            if align_baro and not baro_alt_offset_set and latest_gnss is not None:
                baro_alt_offset = latest_gnss["alt_m"]
                baro_alt_offset_set = True
                print(f"  Baro alt offset: {baro_alt_offset:.1f}m "
                      f"(from GNSS at t={t_rel:.1f}s)")

            baro_alt = pressure_to_altitude(rec["pressure_pa"], baro_ref_pa) + baro_alt_offset
            baro_log_time.append(time_us)
            baro_log_alt.append(baro_alt)

            # Feed baro to EKF (if initialized and not during transonic)
            if ekf_initialized:
                baro_d = BaroData()
                baro_d.time_us = time_us
                baro_d.altitude_m = baro_alt
                ekf.baro_meas_update(baro_d)
                n_baro_updates += 1

        elif etype == "imu":
            if latest_gnss is None:
                continue  # Need at least one GNSS fix before init

            # Every logged sample lands in the window.  The EKF runs on the
            # firmware's CLOCK (ekf_rate_hz — logged since #529, constant
            # fallback before), and at each tick is handed the window's MEAN
            # (--imu-feed mean, firmware since #1214) or just the newest sample
            # (newest, the 1-in-N pick of the firmware before it).  The #1190
            # shock-gate verdicts come from every sample of the window either
            # way, as the FC drain loop's do.
            imu_win.append(rec)
            if ekf_initialized and (next_ekf_us is not None) and (time_us < next_ekf_us):
                continue
            imu_d = _ekf_imu_from_window(imu_win, rec, rot_z_deg, imu_feed,
                                         shock_gate, si_bars)
            imu_win = []

            # Prepare GNSS data (LLA path)
            gnss_d = GNSSDataLLA()
            gnss_d.time_us = gnss_counter  # EKF detects new data when this changes
            gnss_d.lat_rad = latest_gnss["lat"] * DEG2RAD
            gnss_d.lon_rad = latest_gnss["lon"] * DEG2RAD
            gnss_d.alt_m = latest_gnss["alt_m"]
            gnss_d.vel_n_mps = latest_gnss["vel_n"]
            gnss_d.vel_e_mps = latest_gnss["vel_e"]
            gnss_d.vel_d_mps = -latest_gnss["vel_u"]  # Up → Down

            # Prepare mag data (board FLU → FRD)
            mag_d = MagData()
            if latest_mag is not None:
                mag_d.time_us = latest_mag["time_us"]
                mag_d.mag_x = latest_mag["mag_x"]        # X same
                mag_d.mag_y = -latest_mag["mag_y"]        # FLU→FRD
                mag_d.mag_z = -latest_mag["mag_z"]        # FLU→FRD

            if not ekf_initialized:
                ekf.init_lla(imu_d, gnss_d, mag_d)

                # GpsInsEKF::initCore does NOT derive the attitude from the
                # sensors — it hard-codes the quaternion to nose-up vertical
                # (0.707, 0, 0.707, 0), which the firmware then immediately
                # overwrites (quatFromAccelHeading + setQuaternion). So the
                # attitude after init_lla is a placeholder, not an estimate, and
                # something must overwrite it here too. (An older comment in this
                # spot claimed the pad attitude "comes from init_lla ... exactly
                # as the firmware does". It does not.)
                ekf_initialized = True

                # #514: SEED ATTITUDE FROM THE LOG, not from a re-run of the
                # firmware's init.
                #
                # The firmware initializes its EKF the moment it gets a good GNSS
                # fix — which is BEFORE logging starts. By the time the first
                # record lands, its filter has already been running and converging
                # for an unlogged stretch of time. So re-running the coarse
                # alignment on the first LOGGED sample does not reproduce the
                # firmware's attitude at that instant; it reproduces the attitude
                # the firmware had at an earlier moment we have no record of.
                #
                # Measured: that leaves a flat ~11° pedestal from t=0 that no
                # amount of downstream fidelity work can remove, because the
                # information simply is not in the file.
                #
                # The log DOES carry the firmware's own attitude (NonSensorData
                # q0..q3). Seeding from it is the correct initial condition — the
                # same move as seeding position from GNSS — and it is not "fitting
                # to the answer": only t=0 is seeded, and every sample after it is
                # the replay's own integration, which is exactly what the fidelity
                # check then scores.
                q_seed = _first_logged_quat(records, time_us)
                if q_seed is not None:
                    ekf.set_quaternion(*q_seed)
                    align_src = "seeded from the logged quaternion"
                else:
                    # Legacy log with no quaternion: fall back to the firmware's
                    # coarse alignment and accept the pre-log-history error.
                    q_seed = _quat_from_accel_heading(
                        imu_d.acc_x, imu_d.acc_y, imu_d.acc_z,
                        math.radians(PAD_HEADING_DEG))
                    ekf.set_quaternion(*q_seed)
                    align_src = ("coarse-aligned from accel (legacy log: no "
                                 "logged quaternion to seed from)")

                # Velocity is logged too; seed it for the same reason.
                v_seed = _first_logged_vel(records, time_us)
                if v_seed is not None:
                    ekf.set_velocity(*v_seed)

                # #1412: and so are the IMU biases, with their covariance.
                # Without this the replay relearns them from a fresh filter's
                # prior, whose gyro-bias variance is ~6200x the converged one,
                # and GNSS + baro innovations drive the gyro bias to its 10
                # deg/s clamp within two seconds of launch — 14 deg/s of bias
                # error integrating to 79 deg of attitude by T+8 s on the
                # 2026-08-29 Rolly Polly 54 mm log, where the firmware's own
                # bias never left 0.13-0.45 deg/s. Seeding it takes T+5 s from
                # 45.6 deg to the dead-reckoning floor.
                b_seed = _first_logged_biases(records, time_us)
                if b_seed is not None:
                    (gx, gy, gz), gvar, (ax, ay, az), avar = b_seed
                    ekf.set_gyro_bias(gx, gy, gz, gvar)
                    ekf.set_accel_bias(ax, ay, az, avar)
                    bias_src = (f"biases seeded from the log "
                                f"(gyro {math.degrees(gx):+.3f},{math.degrees(gy):+.3f},"
                                f"{math.degrees(gz):+.3f} dps, 1sigma "
                                f"{math.degrees(math.sqrt(max(gvar, 0.0))):.4f} dps)")
                else:
                    bias_src = ("biases NOT seeded — no Snapshot record; the "
                                "filter must relearn them and its gyro bias may "
                                "run to the clamp (#1412)")

                # #514: reproduce the firmware's MAGNETIC DECLINATION.
                #
                # The mag update is heading-only, and it steers toward
                # magnetic north + declination. The firmware evaluates WMM2025
                # once at init (TR_GeoMag::declinationRad on the averaged pad
                # fix + GPS date) and hands it to setDeclination, so its heading
                # is TRUE north. A replay that skips this converges to MAGNETIC
                # north instead, leaving the attitude carrying a yaw offset the
                # size of the local declination (−11.9° here — big enough to matter,
                # and it was not being applied at all).
                #
                # Same routine as the firmware, so this is exact, not an estimate.
                decl_rad = declination_rad(
                    gnss_d.lat_rad, gnss_d.lon_rad, gnss_d.alt_m,
                    _decimal_year(latest_gnss))
                ekf.set_declination(decl_rad)

                print(f"  EKF initialized at t={t_rel:.2f}s  {align_src}, "
                      f"declination={math.degrees(decl_rad):.2f}°")
                print(f"    {bias_src}")
                continue

            # This is an EKF tick (the window gate above let it through).
            #
            # Accumulate the SCHEDULE, never restart it from the sample that
            # happened to cross it.  Ticks land on the first logged sample at or
            # after each scheduled instant, so restarting from that sample adds
            # the leftover every time and quantises the period UP to the sample
            # grid — always in the same direction.  Measured on the 2026-08-29
            # Rolly Polly 54 mm log (IMU ~3.9 kHz): 473 Hz against the firmware's
            # 495 Hz, i.e. the replay skipped 4.4 % of the filter's updates while
            # printing the firmware's rate as if it had matched it.  Accumulating
            # gives 494 Hz.  (It did NOT move that log's fidelity — see #1412 —
            # but "the replay must match this or it is not a replay" is the
            # premise of this whole file, so it should actually match.)
            if next_ekf_us is None:
                next_ekf_us = time_us + ekf_period_us
            else:
                next_ekf_us += ekf_period_us
                # A gap in the log (dropped samples, a pause) must not leave the
                # schedule behind real time and fire a burst catching up.
                if next_ekf_us <= time_us:
                    next_ekf_us = time_us + ekf_period_us

            # #514: AHRS accel gate — follow the LOGGED flight state.
            #
            # Firmware (flight_computer/main.cpp):
            #     post_apogee   = kinematics.apogee_flag;
            #     use_ahrs_acc  = (rocket_state != INFLIGHT) || post_apogee;
            #
            # This replay used to re-derive the phase from an accel>3g heuristic.
            # When that heuristic disagreed with the firmware about *when* the
            # vehicle went INFLIGHT, the replay kept applying the gravity-based
            # AHRS correction through boost — where accel is 10 g and points
            # nowhere near gravity — and the attitude got yanked. Measured on
            # flight_20260615_171305: a 70° divergence appearing right at launch,
            # on top of an otherwise <2° track.
            #
            # rocket_state is logged exactly, so the first term is exact. And
            # since #529 the second is too: kinematics.apogee_flag — the 4-voter
            # quorum (vel / alt / gps / pitch) — has been in the log all along
            # (apogee_flags bit 2, #142/#143), so post_apogee follows the logged
            # master directly. Only pre-#143 logs (42/43-byte NonSensor, no
            # apogee_flags byte) fall back to the old approximation: a LATCHED OR
            # of the two voters that layout carried.
            post_apogee = log_apogee_master if log_has_master else log_apogee_latched
            use_ahrs_acc = (log_rocket_state != ROCKET_STATE_INFLIGHT) or post_apogee

            ekf.update_lla(use_ahrs_acc, imu_d, gnss_d, mag_d)
            n_imu += 1
            tick_t.append(time_us)
            tick_held.append(ekf.shock_gate_held())

            if gnss_d.time_us != last_gnss_time_us:
                n_gnss_updates += 1
                last_gnss_time_us = gnss_d.time_us

            # Log EKF output (every 5th tick ≈ 100 Hz — a 120 ms shock burst
            # needs more than the 3 points a 25 Hz log would give it)
            if n_imu % 5 == 0:
                log_time_us.append(time_us)
                pos = ekf.get_position()
                vel = ekf.get_velocity()
                ori = ekf.get_orientation()
                q = ekf.get_quaternion()
                gb = ekf.get_rot_rate_bias()
                ab = ekf.get_accel_bias()
                cp = ekf.get_cov_pos()
                cv = ekf.get_cov_vel()
                ca = ekf.get_cov_orient()
                log_ekf_lat.append(pos[0])  # rad
                log_ekf_lon.append(pos[1])  # rad
                log_ekf_alt.append(pos[2])  # m
                log_ekf_vn.append(vel[0])
                log_ekf_ve.append(vel[1])
                log_ekf_vd.append(vel[2])
                log_ekf_roll.append(ori[0] * RAD2DEG)
                log_ekf_pitch.append(ori[1] * RAD2DEG)
                log_ekf_yaw.append(ori[2] * RAD2DEG)
                log_ekf_q.append(q)
                log_ekf_gyro_bias.append(
                    (gb[0]*RAD2DEG, gb[1]*RAD2DEG, gb[2]*RAD2DEG))
                log_ekf_accel_bias.append(ab)
                log_cov_pos.append(cp)
                log_cov_vel.append(cv)
                log_cov_att.append(ca)
                log_held.append(ekf.shock_gate_held())

    print(f"\n  Processed: {n_imu:,} IMU updates, {n_gnss_updates} GNSS updates, "
          f"{n_baro_updates} baro updates")

    # ── #1190 shock gate: what it did ──
    hold_spans = _spans(tick_t, tick_held)
    shock_trips = ekf.shock_gate_trips()
    shock_hold_ticks = ekf.shock_gate_hold_ticks()
    print(f"\n  ── Shock gate (#1190): {shock_trips} trips, attitude held "
          f"{shock_hold_ticks} ticks in {len(hold_spans)} span(s) ──")
    for a, b in hold_spans:
        print(f"     held t={(a - t0_us) / 1e6:8.3f}s .. {(b - t0_us) / 1e6:8.3f}s "
              f"({(b - a) / 1e3:.0f} ms)")

    # Show the achieved EKF rate against the firmware's, so a rate mismatch is
    # visible rather than inferred — it is the single easiest way to make this
    # tool silently wrong (see derive_ekf_rate_hz / DEFAULT_EKF_RATE_HZ).
    span_s = (imu_times[-1] - imu_times[0]) / 1e6
    if span_s > 0:
        imu_hz = len(records["ISM6HG256"]) / span_s
        ekf_hz = n_imu / span_s
        print(f"  IMU logged at {imu_hz:.0f} Hz; EKF run at {ekf_hz:.0f} Hz "
              f"(firmware {ekf_rate_hz:.0f} Hz — the EKF sees "
              f"1 in {imu_hz / max(ekf_hz, 1e-9):.1f} logged samples)")

    # ---- Convert to numpy ----
    t_ekf = (np.array(log_time_us) - t0_us) / 1e6
    held_arr = np.array(log_held, dtype=bool)
    ekf_lat = np.array(log_ekf_lat)
    ekf_lon = np.array(log_ekf_lon)
    ekf_alt = np.array(log_ekf_alt)
    ekf_vn = np.array(log_ekf_vn)
    ekf_ve = np.array(log_ekf_ve)
    ekf_vd = np.array(log_ekf_vd)
    ekf_roll = np.array(log_ekf_roll)
    ekf_pitch = np.array(log_ekf_pitch)
    ekf_yaw = np.array(log_ekf_yaw)
    gyro_bias = np.array(log_ekf_gyro_bias)
    accel_bias = np.array(log_ekf_accel_bias)
    cov_pos = np.array(log_cov_pos)
    cov_vel = np.array(log_cov_vel)
    cov_att = np.array(log_cov_att)

    # ── Replay fidelity: does this replay reproduce the FIRMWARE's own EKF? ──
    #
    # #514: without this the replay is trusted, never checked. The flight log
    # already carries the firmware EKF's attitude quaternion (NonSensorData
    # q0..q3), so we can compare directly instead of inferring — and a replay that
    # silently drifts from the firmware invalidates every conclusion drawn from it.
    #
    # This exists because on 2026-07-14 an unfaithful replay (plus the CSV's
    # mixed-convention Euler columns) manufactured a 50° "EKF attitude error" that
    # did not exist — the firmware's real error was 1.4°. The check below would
    # have caught it immediately.
    #
    # Compare with the geodesic angle 2·acos(|q_replay · q_logged|), which is
    # sign-agnostic (q and -q are the same rotation) and has no Euler conventions
    # anywhere near it.
    fidelity = None
    q_replay = np.array(log_ekf_q)
    if len(q_replay) and records.get("NonSensor"):
        ns = records["NonSensor"]
        t_ns = np.array([r["time_us"] for r in ns], dtype=float)
        q_ns = np.array([[r["q0"], r["q1"], r["q2"], r["q3"]] for r in ns], dtype=float)
        n_ns = np.linalg.norm(q_ns, axis=1)
        keep = n_ns > 0.5                      # a zero quaternion means "absent"
        t_ns, q_ns = t_ns[keep], q_ns[keep] / n_ns[keep][:, None]
        if len(t_ns) > 1:
            t_r = np.array(log_time_us, dtype=float)
            # Interpolate the LOGGED quaternion onto the replay's timestamps.
            # Component-wise interp is fine here: the log rate (~490 Hz) is far
            # above the attitude bandwidth, so successive quaternions are nearly
            # parallel. Hemisphere-align first so a sign flip can't corrupt it.
            qs = q_ns.copy()
            flip = np.sum(qs[1:] * qs[:-1], axis=1) < 0
            qs[1:][np.cumsum(flip) % 2 == 1] *= -1
            q_log = np.stack([np.interp(t_r, t_ns, qs[:, k]) for k in range(4)], 1)
            q_log /= np.linalg.norm(q_log, axis=1)[:, None]
            qr = q_replay / np.linalg.norm(q_replay, axis=1)[:, None]
            dot = np.abs(np.sum(qr * q_log, axis=1)).clip(0.0, 1.0)
            div = np.degrees(2.0 * np.arccos(dot))
            inside = (t_r >= t_ns[0]) & (t_r <= t_ns[-1])
            if inside.any():
                div = div[inside]
                div_t = (t_r[inside] - t0_us) / 1e6
                fidelity = {
                    "mean_deg": float(div.mean()),
                    "max_deg": float(div.max()),
                    "p95_deg": float(np.percentile(div, 95)),
                    "n": int(div.size),
                }
                # EARLY divergence is the discriminating number, not the mean.
                # Attitude is uncorrected between launch and apogee (the AHRS
                # accel gate is shut, and on a pre-#1304 filter so is the mag),
                # so both solutions dead-reckon there and any difference
                # compounds.  Once they decorrelate the geodesic angle is
                # roughly uniform and its mean sits near 90° NO MATTER how good
                # the replay was early — measured on one 2026-08-29 log (#1412):
                # 85.6° for a replay already 44° out at T+0.7 s, and 52.9° for
                # one that is 0.3° out there.  The mean cannot tell them apart.
                t_in = (t_r[inside] - t_r[inside][0]) / 1e6
                early = {}
                for mark in (0.5, 1.0, 2.0, 5.0):
                    j = int(np.searchsorted(t_in, mark))
                    if j < div.size:
                        early[mark] = float(div[j])
                fidelity["early_deg"] = early
                # Judge on the first second, where a faithful replay still
                # tracks and a broken one has already left.  Same 5° bar the
                # whole-flight p95 used to carry — but somewhere it can be met.
                probe_s = 1.0 if 1.0 in early else (0.5 if 0.5 in early else None)
                ok = probe_s is not None and early[probe_s] <= 5.0
                fidelity["verdict_basis_s"] = probe_s
                print("\n  ── Replay fidelity vs the FIRMWARE's logged quaternion ──")
                print(f"     mean {fidelity['mean_deg']:6.2f}°   "
                      f"p95 {fidelity['p95_deg']:6.2f}°   "
                      f"max {fidelity['max_deg']:6.2f}°   (n={fidelity['n']})")
                print("     (the mean saturates near 90° once the two decorrelate — "
                      "read the early row, not it)")
                if early:
                    print("     early divergence:  " + "   ".join(
                        f"T+{k:.1f}s {v:6.2f}°" for k, v in sorted(early.items())))
                # Divergence-vs-time is the diagnostic, not the summary: a replay
                # that is wrong at t=0 and stays wrong has an init/frame bug, one
                # that starts at 0 and grows has an integration/rate/bias bug, and
                # one that steps at a phase boundary has a gate bug.
                print("     divergence over time:")
                for f in (0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0):
                    i = min(int(f * (div.size - 1)), div.size - 1)
                    print(f"       t={t_in[i]:7.2f}s  {div[i]:7.2f}°")
                if ok:
                    print(f"     PASS — the replay tracks the firmware through the first "
                          f"{fidelity['verdict_basis_s']:.1f} s; conclusions about that "
                          "stretch are meaningful.")
                    print("     Divergence later, where neither filter is correcting "
                          "attitude, is expected — judge it against the rows above.")
                else:
                    print("     *** FAIL *** — this replay does NOT reproduce the "
                          "firmware EKF. Do not draw conclusions from it.")
                    print("     FIRST read the version guard at the top of this run:")
                    print("       • SKEW    → that's the cause, and it is enough on its own: #1304's")
                    print("                  mag gating alone moves T+0.7 s from 0.3° to 44° on a")
                    print("                  2026-08-29 log. Rebuild at the firmware's commit using")
                    print("                  the worktree recipe shown above.")
                    print("       • UNKNOWN → no sidecar SHA; the log may predate the "
                          "EKF you built. The 2026-06-15 logs are this case — they")
                    print("                  predate #243, which replaced the mag "
                          "heading fusion, so a modern EKF cannot reproduce them.")
                    print("       • MATCHED → versions agree, so it's a real replay gap. These were each")
                    print("                  measured and EXCLUDED on the 2026-08-29 logs (#1412), so don't")
                    print("                  start there: the IMU feed model, the baro frame")
                    print("                  (--emulate-firmware-baro), baro/IMU timestamp duplication, the")
                    print("                  quaternion interpolation, the gyro stream itself (it integrates")
                    print("                  to the logged attitude), GNSS acceptance, and the EKF rate.")
                    print("                  What survived there was a COAST-ONLY gap that needs baro AND")
                    print("                  GNSS both present — neither alone reproduces it.")
                    print("                  Still worth checking on other logs: the AHRS phase gate, GNSS")
                    print("                  velocity, and units (EkfIMUData gyro is DEG/S, not rad/s).")
    if fidelity is None:
        print("\n  ── Replay fidelity: no logged quaternion in this file "
              "(legacy log) — replay is UNVERIFIED ──")
        div_t, div = np.array([]), np.array([])

    # #1190: the replay's pitch across each hold span, against what the
    # firmware logged there.  Pitch here is the elevation of body +X.
    def _logged_pitch_at(tt):
        r = _nearest_nonsensor(records, t0_us + int(tt * 1e6))
        if r is None:
            return float("nan")
        q = np.array([r["q0"], r["q1"], r["q2"], r["q3"]], dtype=float)
        q /= np.linalg.norm(q)
        v = max(-1.0, min(1.0, 2.0 * (q[0] * q[2] - q[1] * q[3])))
        return math.degrees(math.asin(v))
    for a, b in hold_spans:
        ta, tb = (a - t0_us) / 1e6, (b - t0_us) / 1e6
        def _pitch_at(tt):
            return ekf_pitch[int(np.argmin(np.abs(t_ekf - tt)))]
        print(f"     hold {ta:8.3f}..{tb:8.3f}s: replay pitch "
              f"{_pitch_at(ta - 0.02):6.1f}° before → {_pitch_at(tb):6.1f}° at release "
              f"→ {_pitch_at(tb + 0.1):6.1f}° +100 ms   | firmware logged "
              f"{_logged_pitch_at(ta - 0.02):6.1f}° → {_logged_pitch_at(tb):6.1f}° → "
              f"{_logged_pitch_at(tb + 0.1):6.1f}°")

    t_gnss = (np.array(gnss_log_time) - t0_us) / 1e6
    g_lat = np.array(gnss_log_lat)
    g_lon = np.array(gnss_log_lon)
    g_alt = np.array(gnss_log_alt)
    g_vn = np.array(gnss_log_vn)
    g_ve = np.array(gnss_log_ve)
    g_vd = np.array(gnss_log_vd)

    # Convert EKF lat/lon to local NE (m) relative to first GNSS fix
    R_earth = 6378137.0
    ref_lat = g_lat[0] * DEG2RAD
    ref_lon = g_lon[0] * DEG2RAD
    ref_alt = g_alt[0]

    ekf_n = (ekf_lat - ref_lat) * R_earth
    ekf_e = (ekf_lon - ref_lon) * R_earth * math.cos(ref_lat)
    ekf_u = ekf_alt - ref_alt

    gnss_n = (g_lat * DEG2RAD - ref_lat) * R_earth
    gnss_e = (g_lon * DEG2RAD - ref_lon) * R_earth * math.cos(ref_lat)
    gnss_u = g_alt - ref_alt

    # Print summary
    print("\n" + "=" * 60)
    print("REPLAY SUMMARY")
    print("=" * 60)
    print(f"  EKF altitude range: {ekf_u.min():.1f} to {ekf_u.max():.1f} m")
    print(f"  GNSS altitude range: {gnss_u.min():.1f} to {gnss_u.max():.1f} m")
    print(f"  EKF pitch range: {ekf_pitch.min():.1f} to {ekf_pitch.max():.1f} deg")
    print(f"  Gyro bias final: X={gyro_bias[-1,0]:.3f} Y={gyro_bias[-1,1]:.3f} "
          f"Z={gyro_bias[-1,2]:.3f} dps")
    print(f"  Accel bias final: X={accel_bias[-1,0]:.3f} Y={accel_bias[-1,1]:.3f} "
          f"Z={accel_bias[-1,2]:.3f} m/s²")

    # Pad / pre-boost sanity metric: a stationary rocket's AGL altitude should
    # sit at ~0.  The baro frame bug drags the fused altitude negative by
    # roughly the launch-site MSL elevation.
    pad_alt = float('nan')
    if boost_start is not None:
        bs_rel = (boost_start - t0_us) / 1e6
        pad_mask = t_ekf < bs_rel
        if pad_mask.any():
            pad_alt = float(np.mean(ekf_u[pad_mask]))
    print(f"  Pad EKF altitude AGL (pre-boost mean): {pad_alt:.2f} m   [should be ~0]")

    # ---- Plot ----
    if plot_dir is None:
        plot_dir = Path(__file__).parent.parent / "plots"
    plot_dir = Path(plot_dir)
    plot_dir.mkdir(exist_ok=True)

    # Flight phase shading helper
    def shade_phases(ax):
        if boost_start and boost_end:
            bs = (boost_start - t0_us) / 1e6
            be = (boost_end - t0_us) / 1e6
            ax.axvspan(bs, be, color='red', alpha=0.08, label='Boost')
        if apogee_us:
            ap = (apogee_us - t0_us) / 1e6
            ax.axvline(ap, color='blue', ls=':', lw=0.8, alpha=0.5, label='Apogee')

    # --- Fig 1: Position (NE + altitude) ---
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('Flight Replay: EKF Position vs GNSS', fontsize=14, fontweight='bold')

    axes[0].plot(t_gnss, gnss_n, '.', ms=2, color='C0', alpha=0.3, label='GNSS')
    axes[0].plot(t_ekf, ekf_n, '-', lw=1, color='C3', label='EKF')
    axes[0].set_ylabel('North (m)')
    axes[0].legend(fontsize=9); axes[0].grid(True, alpha=0.3)
    shade_phases(axes[0])

    axes[1].plot(t_gnss, gnss_e, '.', ms=2, color='C0', alpha=0.3, label='GNSS')
    axes[1].plot(t_ekf, ekf_e, '-', lw=1, color='C3', label='EKF')
    axes[1].set_ylabel('East (m)')
    axes[1].legend(fontsize=9); axes[1].grid(True, alpha=0.3)
    shade_phases(axes[1])

    axes[2].plot(t_gnss, gnss_u, '.', ms=2, color='C0', alpha=0.3, label='GNSS')
    axes[2].plot(t_ekf, ekf_u, '-', lw=1, color='C3', label='EKF')
    if baro_log_time:
        t_baro = (np.array(baro_log_time) - t0_us) / 1e6
        baro_agl = np.array(baro_log_alt) - ref_alt
        axes[2].plot(t_baro, baro_agl, '.', ms=1, color='C2', alpha=0.2,
                     label='Baro')
    axes[2].set_ylabel('Altitude AGL (m)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(fontsize=9); axes[2].grid(True, alpha=0.3)
    shade_phases(axes[2])

    plt.tight_layout()
    out1 = plot_dir / 'replay_position.png'
    plt.savefig(out1, dpi=180, bbox_inches='tight')
    plt.close()

    # --- Fig 2: Velocity ---
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('Flight Replay: EKF Velocity vs GNSS', fontsize=14, fontweight='bold')

    for i, (label, ekf_v, gnss_v) in enumerate([
        ('North', ekf_vn, g_vn),
        ('East', ekf_ve, g_ve),
        ('Down', ekf_vd, g_vd),
    ]):
        axes[i].plot(t_gnss, gnss_v, '.', ms=2, color='C0', alpha=0.3, label='GNSS')
        axes[i].plot(t_ekf, ekf_v, '-', lw=1, color='C3', label='EKF')
        axes[i].set_ylabel(f'Vel {label} (m/s)')
        axes[i].legend(fontsize=9); axes[i].grid(True, alpha=0.3)
        shade_phases(axes[i])

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    out2 = plot_dir / 'replay_velocity.png'
    plt.savefig(out2, dpi=180, bbox_inches='tight')
    plt.close()

    # --- Fig 3: Attitude + gyro bias ---
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    gate_note = (f"shock gate ON: {shock_trips} trips, {len(hold_spans)} hold span(s)"
                 if shock_gate else "shock gate OFF")
    fig.suptitle(f'Flight Replay: EKF Attitude & Gyro Bias  [{imu_feed} feed, {gate_note}]',
                 fontsize=14, fontweight='bold')

    def shade_holds(ax):
        for k, (a, b) in enumerate(hold_spans):
            ax.axvspan((a - t0_us) / 1e6, (b - t0_us) / 1e6, color='magenta',
                       alpha=0.25, label='Shock-gate hold' if k == 0 else None)

    axes[0].plot(t_ekf, ekf_roll, '-', lw=1, color='C0')
    axes[0].set_ylabel('Roll (deg)'); axes[0].grid(True, alpha=0.3)
    shade_phases(axes[0]); shade_holds(axes[0])

    axes[1].plot(t_ekf, ekf_pitch, '-', lw=1, color='C1')
    axes[1].set_ylabel('Pitch (deg)'); axes[1].grid(True, alpha=0.3)
    shade_phases(axes[1]); shade_holds(axes[1])
    if hold_spans:
        axes[1].legend(fontsize=9)

    axes[2].plot(t_ekf, ekf_yaw, '-', lw=1, color='C2')
    axes[2].set_ylabel('Yaw (deg)'); axes[2].grid(True, alpha=0.3)
    shade_phases(axes[2])

    axes[3].plot(t_ekf, gyro_bias[:, 0], '-', lw=1, label='X')
    axes[3].plot(t_ekf, gyro_bias[:, 1], '-', lw=1, label='Y')
    axes[3].plot(t_ekf, gyro_bias[:, 2], '-', lw=1, label='Z')
    axes[3].set_ylabel('Gyro bias (dps)')
    axes[3].set_xlabel('Time (s)')
    axes[3].legend(fontsize=9); axes[3].grid(True, alpha=0.3)
    shade_phases(axes[3])

    plt.tight_layout()
    out3 = plot_dir / 'replay_attitude.png'
    plt.savefig(out3, dpi=180, bbox_inches='tight')
    plt.close()

    # --- Fig 4: Covariance (3-sigma bounds) ---
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    fig.suptitle('Flight Replay: EKF Covariance (3σ)', fontsize=14, fontweight='bold')

    sig_pos = 3 * np.sqrt(cov_pos[:, 0]**2 + cov_pos[:, 1]**2 + cov_pos[:, 2]**2)
    sig_vel = 3 * np.sqrt(cov_vel[:, 0]**2 + cov_vel[:, 1]**2 + cov_vel[:, 2]**2)
    sig_att = 3 * np.sqrt(cov_att[:, 0]**2 + cov_att[:, 1]**2 + cov_att[:, 2]**2) * RAD2DEG

    axes[0].plot(t_ekf, sig_pos, '-', lw=1, color='C0')
    axes[0].set_ylabel('3σ Position (m)'); axes[0].grid(True, alpha=0.3)
    shade_phases(axes[0])

    axes[1].plot(t_ekf, sig_vel, '-', lw=1, color='C1')
    axes[1].set_ylabel('3σ Velocity (m/s)'); axes[1].grid(True, alpha=0.3)
    shade_phases(axes[1])

    axes[2].plot(t_ekf, sig_att, '-', lw=1, color='C2')
    axes[2].set_ylabel('3σ Attitude (deg)')
    axes[2].set_xlabel('Time (s)')
    axes[2].grid(True, alpha=0.3)
    shade_phases(axes[2])

    plt.tight_layout()
    out4 = plot_dir / 'replay_covariance.png'
    plt.savefig(out4, dpi=180, bbox_inches='tight')
    plt.close()

    plots = [out1, out2, out3, out4]
    print(f"\nPlots saved to {plot_dir}/")
    return {
        "align_baro": align_baro,
        "imu_feed": imu_feed,
        "shock_gate": shock_gate,
        # #1190: what the gate did — trips, held ticks, the hold spans in
        # absolute time_us, and the per-log-sample held flag.
        "shock_trips": shock_trips,
        "shock_hold_ticks": shock_hold_ticks,
        "hold_spans": hold_spans,
        "held": held_arr,
        "ekf_pitch": ekf_pitch,
        "ekf_roll": ekf_roll,
        "ekf_q": np.array(log_ekf_q),
        # #514 divergence-vs-time series behind `fidelity` (empty on a legacy log).
        "div_t": div_t,
        "div_deg": div,
        "t0_us": t0_us,
        # #514: None for legacy logs (no quaternion). A FAIL here means every other
        # number in this dict is describing a replay, not the firmware.
        "fidelity": fidelity,
        "t_ekf": t_ekf,
        "ekf_u": ekf_u,
        "t_gnss": t_gnss,
        "gnss_u": gnss_u,
        "accel_bias": accel_bias,
        "pad_alt": pad_alt,
        "ekf_alt_range": (float(ekf_u.min()), float(ekf_u.max())),
        # Horizontal velocity / position / heading for fusion-tuning analysis.
        "ekf_vn": ekf_vn, "ekf_ve": ekf_ve, "ekf_lat": ekf_lat, "ekf_lon": ekf_lon,
        "ekf_yaw": ekf_yaw,
        "g_vn": g_vn, "g_ve": g_ve, "g_lat": g_lat, "g_lon": g_lon,
        "boost_start_rel": (boost_start - t0_us) / 1e6 if boost_start else None,
        "apogee_rel": (apogee_us - t0_us) / 1e6 if apogee_us else None,
        "plots": plots,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Replay flight data through EKF")
    parser.add_argument("binary_file", nargs="?",
                        default=str(Path(__file__).parent.parent.parent /
                                    "TestFlights/2026_03_08/Raw Downloads/"
                                    "Goblin Flight 2 F52/flight_20260308_190239.bin"))
    parser.add_argument("--plot-dir", default=None)
    parser.add_argument("--emulate-firmware-baro", action="store_true",
                        help="Feed pad-relative baro (no GNSS-frame offset) to "
                             "reproduce the firmware bug at main.cpp:2812.")
    parser.add_argument("--imu-feed", choices=("auto", "mean", "newest"),
                        default="auto",
                        help="What each EKF tick is handed: the drain-window MEAN "
                             "(firmware since #1214) or the newest drained sample "
                             "(firmware before it). Default 'auto' takes whichever "
                             "the version guard says wrote this log.")
    parser.add_argument("--no-shock-gate", action="store_true",
                        help="Run the pre-#1190 filter: integrate a saturated gyro / "
                             "accelerometer axis as if it were a measurement.")
    parser.add_argument("--no-open", action="store_true",
                        help="Do not open the plots when done.")
    args = parser.parse_args()

    result = replay(Path(args.binary_file), args.plot_dir,
                    align_baro=not args.emulate_firmware_baro,
                    imu_feed=args.imu_feed, shock_gate=not args.no_shock_gate)

    if not args.no_open:
        import subprocess
        subprocess.run(["open"] + [str(p) for p in result["plots"]])
