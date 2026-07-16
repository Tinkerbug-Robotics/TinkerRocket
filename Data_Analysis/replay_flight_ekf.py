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

    # git diff <sha> -- <files>: lists EKF sources that differ between the
    # firmware's commit and the working tree the extension was built from.
    _, out = _git("diff", "--name-only", fw_sha, "--", *_EKF_SRC)
    differ = [ln for ln in out.splitlines() if ln.strip()]
    if not differ:
        print(f"     EKF source matches that commit — VERSION-MATCHED.{dirty_note}")
        return {"fw_sha": fw_sha, "fw_dirty": fw_dirty, "matched": True}
    print("     *** EKF SOURCE DIFFERS from the firmware's commit — SKEW ***")
    for ln in differ:
        print(f"        changed: {ln}")
    print("     This replay runs a DIFFERENT filter than wrote the log. A fidelity")
    print("     FAIL below is EXPECTED, and its attitude output is not comparable.")
    print(f"     To match the firmware:  git checkout {fw_sha} -- "
          + " ".join(_EKF_SRC))
    print("     then rebuild the extension "
          "(cd tinkerrocket-sim && TR_SKIP_GUIDANCE=1 python setup.py build_ext --inplace).")
    return {"fw_sha": fw_sha, "fw_dirty": fw_dirty, "matched": False}


def replay(binary_file, plot_dir=None, align_baro=True):
    mode = ("aligned baro+GNSS frame (the FIX)" if align_baro
            else "pad-relative baro (firmware behaviour, the BUG)")
    print(f"Parsing: {binary_file}")
    print(f"  Baro frame mode: {mode}")
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

    check_ekf_version(binary_file)

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

            # Select accel source (low-g or high-g based on saturation)
            lx = rec["low_acc_x"]
            ly = rec["low_acc_y"]
            lz = rec["low_acc_z"]
            if (abs(lx) > LOW_G_SAT_THRESH or
                abs(ly) > LOW_G_SAT_THRESH or
                abs(lz) > LOW_G_SAT_THRESH):
                ax_board = rec["high_acc_x"]
                ay_board = rec["high_acc_y"]
                az_board = rec["high_acc_z"]
            else:
                ax_board = lx
                ay_board = ly
                az_board = lz

            # Board frame (FLU) → EKF body frame (FRD)
            imu_d = IMUData()
            imu_d.time_us = time_us
            imu_d.acc_x = ax_board           # X same
            imu_d.acc_y = -ay_board           # FLU Y=Left → FRD Y=Right
            imu_d.acc_z = -az_board           # FLU Z=Up → FRD Z=Down
            imu_d.gyro_x = rec["gyro_x"]      # X same (dps → the EKF converts internally?)
            imu_d.gyro_y = -rec["gyro_y"]      # FLU→FRD
            imu_d.gyro_z = -rec["gyro_z"]      # FLU→FRD

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
                continue

            # Run the EKF on the firmware's CLOCK, not on every logged IMU sample
            # (see ekf_rate_hz — logged since #529, constant fallback before). The
            # firmware feeds it the newest sample at that cadence and never sees
            # the rest, so this drops the intervening samples exactly as the
            # flight loop does — imu_d here is already the latest one.
            if (next_ekf_us is not None) and (time_us < next_ekf_us):
                continue
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

            if gnss_d.time_us != last_gnss_time_us:
                n_gnss_updates += 1
                last_gnss_time_us = gnss_d.time_us

            # Log EKF output (every 20th sample ≈ 100 Hz)
            if n_imu % 20 == 0:
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

    print(f"\n  Processed: {n_imu:,} IMU updates, {n_gnss_updates} GNSS updates, "
          f"{n_baro_updates} baro updates")

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
                fidelity = {
                    "mean_deg": float(div.mean()),
                    "max_deg": float(div.max()),
                    "p95_deg": float(np.percentile(div, 95)),
                    "n": int(div.size),
                }
                ok = fidelity["p95_deg"] <= 5.0
                print("\n  ── Replay fidelity vs the FIRMWARE's logged quaternion ──")
                print(f"     mean {fidelity['mean_deg']:6.2f}°   "
                      f"p95 {fidelity['p95_deg']:6.2f}°   "
                      f"max {fidelity['max_deg']:6.2f}°   (n={fidelity['n']})")
                # Divergence-vs-time is the diagnostic, not the summary: a replay
                # that is wrong at t=0 and stays wrong has an init/frame bug, one
                # that starts at 0 and grows has an integration/rate/bias bug, and
                # one that steps at a phase boundary has a gate bug.
                t_in = (t_r[inside] - t_r[inside][0]) / 1e6
                print("     divergence over time:")
                for f in (0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0):
                    i = min(int(f * (div.size - 1)), div.size - 1)
                    print(f"       t={t_in[i]:7.2f}s  {div[i]:7.2f}°")
                if ok:
                    print("     PASS — the replay reproduces the firmware EKF; "
                          "conclusions drawn from it are meaningful.")
                else:
                    print("     *** FAIL *** — this replay does NOT reproduce the "
                          "firmware EKF. Do not draw conclusions from it.")
                    print("     FIRST read the version guard at the top of this run:")
                    print("       • SKEW    → that's the cause. Check out the "
                          "firmware's EKF sources (command shown there) and rerun.")
                    print("       • UNKNOWN → no sidecar SHA; the log may predate the "
                          "EKF you built. The 2026-06-15 logs are this case — they")
                    print("                  predate #243, which replaced the mag "
                          "heading fusion, so a modern EKF cannot reproduce them.")
                    print("       • MATCHED → versions agree, so it's a real replay "
                          "gap: AHRS phase gate, GNSS velocity, EKF rate")
                    print("                  (derive_ekf_rate_hz / fallback), or units "
                          "(EkfIMUData gyro is DEG/S, not rad/s).")
    if fidelity is None:
        print("\n  ── Replay fidelity: no logged quaternion in this file "
              "(legacy log) — replay is UNVERIFIED ──")

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
    fig.suptitle('Flight Replay: EKF Attitude & Gyro Bias', fontsize=14, fontweight='bold')

    axes[0].plot(t_ekf, ekf_roll, '-', lw=1, color='C0')
    axes[0].set_ylabel('Roll (deg)'); axes[0].grid(True, alpha=0.3)
    shade_phases(axes[0])

    axes[1].plot(t_ekf, ekf_pitch, '-', lw=1, color='C1')
    axes[1].set_ylabel('Pitch (deg)'); axes[1].grid(True, alpha=0.3)
    shade_phases(axes[1])

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
    args = parser.parse_args()

    result = replay(Path(args.binary_file), args.plot_dir,
                    align_baro=not args.emulate_firmware_baro)

    import subprocess
    subprocess.run(["open"] + [str(p) for p in result["plots"]])
