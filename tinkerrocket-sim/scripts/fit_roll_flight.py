#!/usr/bin/env python3
"""Fit the sim's roll plant and the flown roll controller to a real flight log (#549).

Scenario (c) in `simulation/scenarios.py` was "2026-05-17-like": a representative
kick on a hand-tuned plant. #549 asked for a scenario whose plant is FITTED to a
logged flight of the current vehicle, with pass/fail thresholds derived from what
was observed rather than picked. This is the tool that does the fitting, so the
numbers in `scenarios.py` can be regenerated from the binary rather than trusted.

What comes out, in order of how well one flight constrains it:

  1. The controller. The log carries the gyro (ISM6, ~1.6 kHz), the EKF speed and
     the PID output `roll_cmd` (~495 Hz), and the firmware law is known
     (TR_ServoControl: rate-null PID, (V_ref/V)^2 gain schedule capped at
     GAIN_SCHEDULE_SCALE_CAP, integral separation, +/-MAX_CMD clamp). Replaying the
     logged rate through the law and fitting Kp/Ki to the logged command recovers
     the flown gains to a fraction of a degree. This part is near-exact.
  2. The misalignment. In coast the controller holds a standing tab command that
     cancels the built-in roll trim; the open-loop coast fit pins it.
  3. The kick. The roll rate climbs against a pinned tab during the burn; the
     sim models that disturbance as an impulsive roll_kick_dps, sized here to
     the flown peak rate. It is a disturbance, not a plant parameter, so it is
     NOT fitted: letting it float lets the optimiser shrink it to buy a better
     first swing (a 5-parameter run found 321 dps against 892 flown), which is
     the wrong trade for a regression that must exercise the kick as flown.
  4. Authority (Kt), damping (Cl_p) and the servo lag/delay. These trade off
     against each other, and the ring-down after the kick is a closed-loop
     oscillation, so an open-loop fit of it is ill-conditioned. They are fitted
     CLOSED-LOOP instead: run the real sim (same firmware controller, the gains
     from step 1, the motor, the kick) and minimise the window-normalised rate
     error against the flight over everything AFTER the kick (0.62 s on — an
     impulse cannot match a 0.15 s torque burst, so that window is reported,
     not scored). That is the honest test — "does the scenario reproduce the
     flight" — and it is what the regression thresholds come from.

Run:
    python scripts/fit_roll_flight.py <flight.bin> --out fit_out/ [--motor F52C]
                                       [--launch-angle 66] [--no-closed-loop]

Reads the binary, never the CSV (`binary-is-the-source-of-truth`). Needs the sim
extension built (`python setup.py build_ext --inplace`) for the closed-loop stage.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SIM_ROOT = os.path.dirname(_HERE)
_REPO = os.path.dirname(_SIM_ROOT)
sys.path.insert(0, os.path.join(_SIM_ROOT, "src"))
sys.path.insert(0, os.path.join(_REPO, "Data_Analysis"))

# ── firmware constants the law needs (config.h / TR_ServoControl) ───────────
V_REF, V_MIN, SCALE_CAP, MAX_CMD, INTEGRAL_SEP_DPS = 50.0, 25.0, 3.0, 20.0, 40.0
GYRO_LPF_HZ = 40.0      # the ~800 Hz motor tone and launch shock are not roll
FIT_RATE_HZ = 500.0


def load_traces(bin_path):
    """Roll rate, tab command and EKF speed from the binary, on the launch clock."""
    from plot_flight_data_mini import parse_binary_file
    recs, _, _ = parse_binary_file(str(bin_path))
    imu, ns = recs["ISM6HG256"], recs["NonSensor"]
    t_imu = np.array([r["time_us"] for r in imu]) / 1e6
    gx = np.array([r["gyro_x"] for r in imu], dtype=float)
    t_ns = np.array([r["time_us"] for r in ns]) / 1e6
    rc = np.array([r["roll_cmd"] for r in ns], dtype=float)
    speed = np.sqrt(sum(np.array([r[k] for r in ns], dtype=float) ** 2 for k in ("e_vel", "n_vel", "u_vel")))
    vu = np.array([r["u_vel"] for r in ns], dtype=float)
    launch = np.array([r["launch"] for r in ns])
    apo = np.array([r["apogee_flag"] for r in ns])
    t0 = t_ns[np.argmax(launch > 0)] if (launch > 0).any() else t_ns[0]
    t_apogee = (t_ns[np.argmax(apo > 0)] - t0) if (apo > 0).any() else (t_ns[-1] - t0)
    imu_rate = len(t_imu) / (t_imu[-1] - t_imu[0])
    from scipy.signal import butter, filtfilt
    b, a = butter(4, GYRO_LPF_HZ / (imu_rate / 2))
    return dict(t_imu=t_imu - t0, gx=gx, gxf=filtfilt(b, a, gx), t_ns=t_ns - t0, rc=rc,
                speed=speed, vu=vu, t_apogee=float(t_apogee), imu_rate=float(imu_rate))


def identify_controller(tr, t_end):
    """Kp/Ki of the firmware rate-null law from the logged rate and command."""
    from scipy.optimize import least_squares
    m = (tr["t_ns"] >= 0.0) & (tr["t_ns"] < t_end)
    tn, cmd = tr["t_ns"][m], tr["rc"][m]
    p = np.interp(tn, tr["t_imu"], tr["gxf"])
    V = tr["speed"][m]

    def law(q):
        kp0, ki0 = q
        out = np.empty_like(cmd)
        acc = 0.0
        prev_s = None
        lt = tn[0]
        for i in range(tn.size):
            v = max(abs(V[i]), V_MIN)
            s = min((V_REF / v) ** 2, SCALE_CAP)
            if prev_s is not None and abs(s - prev_s) > 0.1:
                acc = 0.0                       # the firmware resets I on a scale step
            prev_s = s
            e = p[i]                            # firmware passes -gyro_x; error = 0 - (-gx)
            dt = tn[i] - lt
            lt = tn[i]
            if i == 0:
                out[i] = 0.0
                continue
            kp, ki = kp0 * s, ki0 * s
            if INTEGRAL_SEP_DPS <= 0 or abs(e) <= INTEGRAL_SEP_DPS:
                acc += e * dt
            if ki > 0:
                acc = float(np.clip(acc, -MAX_CMD / ki, MAX_CMD / ki))
            out[i] = float(np.clip(kp * e + ki * acc, -MAX_CMD, MAX_CMD))
        return out

    r = least_squares(lambda q: law(q) - cmd, [0.1, 0.05], bounds=([0, 0], [5, 5]),
                      xtol=1e-10, ftol=1e-10, max_nfev=300)
    rms = float(np.sqrt(np.mean(r.fun ** 2)))
    return dict(kp=float(r.x[0]), ki=float(r.x[1]), kd=0.0, rms_deg=rms,
                cmd_rms_deg=float(np.sqrt(np.mean(cmd ** 2))), ticks=int(tn.size),
                saturated_frac=float(np.mean(np.abs(cmd) >= MAX_CMD - 0.01)))


def _servo(cmd, tau, lag, dt, slew=923.0):
    h = cmd[0]
    out = np.empty_like(cmd)
    for i in range(cmd.size):
        h += float(np.clip((cmd[i] - h) / max(tau, dt) * dt, -slew * dt, slew * dt))
        out[i] = h
    if lag > 0:
        t = np.arange(cmd.size) * dt
        out = np.interp(t - lag, t, out)
    return out


def coast_fit(tr, lo, hi):
    """Open-loop fit of p' = A V^2 (delta + m) - B V p on the coast window.

    Constrains the misalignment m well; A and B trade off (reported for
    reference, the closed-loop stage decides them). A is in dps/s per deg per
    (m/s)^2, negative = the tab opposes the rate in the gyro frame, which is
    what makes the firmware's law stabilising."""
    from scipy.optimize import least_squares
    dt = 1.0 / FIT_RATE_HZ
    t = np.arange(lo, hi, dt)
    p = np.interp(t, tr["t_imu"], tr["gxf"])
    cmd = np.interp(t, tr["t_ns"], tr["rc"])
    V = np.interp(t, tr["t_ns"], tr["speed"])

    def sim(q):
        A, m, B, tau, lag = q
        delta = _servo(cmd, tau, lag, dt)
        ps = np.empty_like(p)
        x = p[0]
        for i in range(t.size):
            x += (A * V[i] ** 2 * (delta[i] + m) - B * V[i] * x) * dt
            ps[i] = x
        return ps

    r = least_squares(lambda q: sim(q) - p, [-0.05, -2.0, 0.1, 0.03, 0.03],
                      bounds=([-2, -6, 0, 0.005, 0], [0, 6, 1.0, 0.2, 0.08]),
                      xtol=1e-9, ftol=1e-9, max_nfev=400)
    A, m, B, tau, lag = r.x
    return dict(A=float(A), misalign_deg=float(m), B=float(B), tau_s=float(tau), lag_s=float(lag),
                rms_dps=float(np.sqrt(np.mean(r.fun ** 2))), flight_rms_dps=float(np.sqrt(np.mean(p ** 2))),
                trim_cmd_deg=float(cmd.mean()))


def kick(tr, lo, hi):
    """When and how hard the roll rate was kicked during the burn."""
    m = (tr["t_imu"] >= lo) & (tr["t_imu"] < hi)
    t, p = tr["t_imu"][m], tr["gxf"][m]
    i = int(np.argmax(np.abs(p)))
    # onset: the last time before the peak the rate was inside 10 % of the peak
    small = np.where(np.abs(p[:i]) < 0.1 * abs(p[i]))[0]
    onset = float(t[small[-1]]) if small.size else float(t[0])
    return dict(peak_dps=float(p[i]), t_peak_s=float(t[i]), t_onset_s=onset)


def closed_loop_fit(tr, args, gains, misalign, kick_info, out_dir):
    """Fit A, B, servo tau and servo delay by running the real sim; the kick is
    fixed at the flown peak rate."""
    from scipy.optimize import minimize
    from tinkerrocket_sim.simulation.scenarios import build_rollypolly_54, RP54_SERVO
    from tinkerrocket_sim.simulation.closed_loop_sim import SimConfig, run_closed_loop

    WIN = [(0.15, 0.62), (0.62, 1.2), (1.2, 2.5), (2.5, 4.5), (4.5, min(7.7, tr["t_apogee"]))]
    SCORED = WIN[1:]                       # the kick window is reported, not scored
    kick_dps = abs(kick_info["peak_dps"])

    def run(x, seed=42):
        A, B, tau, delay = x
        rd = build_rollypolly_54(motor=args.motor, authority=A, misalign_deg=misalign, damping=B)
        cfg = SimConfig(
            pad_time=10.0, duration=WIN[-1][1] + 1.0, physics_dt=1e-3,
            launch_angle_deg=args.launch_angle, control_enabled=True,
            use_firmware_roll_controller=True, roll_gain_schedule_enabled=True,
            guidance_enabled=False, enable_mag_updates=False, pad_heading_deg=0.0,
            sensor_seed=seed, pid_kp=gains["kp"], pid_ki=gains["ki"], pid_kd=0.0,
            kp_angle=2.0, rate_cap_dps=60.0, integral_sep_threshold=INTEGRAL_SEP_DPS,
            gain_V_ref=V_REF, gain_V_min=V_MIN, roll_delay_s=0.0, roll_min_speed_mps=0.0,
            roll_profile=[(0.0, 0.0, "null_rate")], roll_targeting="hold",
            roll_kick_time_s=kick_info["t_kick_s"], roll_kick_dps=kick_dps,
            servo_tau_s=tau, servo_cmd_delay_s=delay,
            servo_rate_limit=RP54_SERVO["rate_limit"], servo_deadband_us=RP54_SERVO["deadband_us"],
        )
        df = run_closed_loop(rd, cfg).df
        t = df["time"].to_numpy()
        ts = t - t[np.argmax(df["speed"].to_numpy() > 1.0)]
        ps = df["roll_rate_dps"].to_numpy()
        pf = np.interp(ts, tr["t_imu"], tr["gxf"])
        parts = []
        for lo, hi in WIN:
            w = (ts >= lo) & (ts < hi)
            parts.append((float(np.sqrt(np.mean((ps[w] - pf[w]) ** 2))), float(np.sqrt(np.mean(pf[w] ** 2)))))
        score = sum(e / r for (lo, hi), (e, r) in zip(WIN, parts) if (lo, hi) in SCORED) / len(SCORED)
        return score, parts, (ts, ps, pf, df["fin_tab_cmd"].to_numpy())

    lo_b = np.array([-0.4, 0.0, 0.005, 0.0])
    hi_b = np.array([-0.01, 1.5, 0.12, 0.15])
    n = [0]
    t0 = time.time()

    def f(x):
        x = np.clip(x, lo_b, hi_b)
        s, _, _ = run(x)
        n[0] += 1
        if n[0] % 10 == 0:
            print(f"  eval {n[0]:3d} ({time.time() - t0:4.0f}s) score {s:.4f}  A={x[0]:.4f} B={x[1]:.3f} "
                  f"tau={x[2] * 1e3:.0f}ms delay={x[3] * 1e3:.0f}ms", flush=True)
        return s

    x0 = np.array([-0.08, 0.2, 0.03, 0.05])
    simplex = np.array([x0, x0 + [-0.04, 0, 0, 0], x0 + [0, 0.15, 0, 0],
                        x0 + [0, 0, 0.02, 0], x0 + [0, 0, 0, 0.03]])
    r = minimize(f, x0, method="Nelder-Mead",
                 options={"xatol": 1e-3, "fatol": 1e-4, "maxfev": args.max_evals, "initial_simplex": simplex})
    x = np.clip(r.x, lo_b, hi_b)
    score, parts, (ts, ps, pf, cmd) = run(x)
    seeds = {sd: [e for e, _ in run(x, seed=sd)[1]] for sd in (1, 2, 3, 4)}
    res = dict(A=float(x[0]), B=float(x[1]), tau_s=float(x[2]), delay_s=float(x[3]), kick_dps=float(kick_dps),
               score=float(score), windows=WIN, scored_windows=SCORED, window_err_and_flight_rms=parts,
               seed_window_errs=seeds, evals=n[0])
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        ax[0].plot(ts, pf, lw=0.8, label="flight (gyro X, 40 Hz LPF)")
        ax[0].plot(ts, ps, lw=0.8, label="sim, fitted plant + flown gains")
        ax[0].set_ylabel("roll rate (dps)"); ax[0].legend(); ax[0].grid(alpha=.3); ax[0].set_xlim(0, WIN[-1][1])
        ax[1].plot(tr["t_ns"], tr["rc"], lw=0.8, label="flight roll_cmd")
        ax[1].plot(ts, cmd, lw=0.8, label="sim fin_tab_cmd")
        ax[1].set_ylabel("deg"); ax[1].set_xlabel("t since launch (s)"); ax[1].legend(); ax[1].grid(alpha=.3)
        fig.suptitle(f"closed-loop fit: A={x[0]:.4f} B={x[1]:.3f} tau={x[2] * 1e3:.0f} ms delay={x[3] * 1e3:.0f} ms "
                     f"kick={kick_dps:.0f} dps (flown peak)  score={score:.3f} over 0.62 s on")
        fig.tight_layout(); fig.savefig(os.path.join(out_dir, "closed_loop_fit.png"), dpi=110)
    except Exception as e:  # plotting is a courtesy, not the result
        print("  (plot skipped:", e, ")")
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("flight_bin")
    ap.add_argument("--out", default="fit_roll_flight_out")
    ap.add_argument("--motor", default="F52C", help="motor in motors/*.eng the flight flew (default F52C)")
    ap.add_argument("--launch-angle", type=float, default=66.0,
                    help="sim launch angle from horizontal; pick it so the coast airspeed matches the flight's arc")
    ap.add_argument("--no-closed-loop", action="store_true", help="skip the sim-in-the-loop stage")
    ap.add_argument("--max-evals", type=int, default=220)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    tr = load_traces(args.flight_bin)
    print(f"flight: IMU {tr['imu_rate']:.0f} Hz, apogee at T+{tr['t_apogee']:.2f} s")
    gains = identify_controller(tr, min(7.7, tr["t_apogee"]))
    print(f"controller: Kp={gains['kp']:.4f} Ki={gains['ki']:.4f} Kd=0  ({gains['rms_deg']:.2f} deg RMS over "
          f"{gains['ticks']} ticks; command RMS {gains['cmd_rms_deg']:.2f}, clamped {gains['saturated_frac'] * 100:.0f}%)")
    kk = kick(tr, 0.15, 0.62)
    kk["t_kick_s"] = round(0.5 * (kk["t_onset_s"] + kk["t_peak_s"]), 2)
    print(f"kick: peak {kk['peak_dps']:+.0f} dps at T+{kk['t_peak_s']:.2f} s (onset {kk['t_onset_s']:.2f} s) -> inject at {kk['t_kick_s']:.2f} s")
    cf = coast_fit(tr, 2.5, min(7.7, tr["t_apogee"]))
    print(f"coast fit: misalign={cf['misalign_deg']:+.2f} deg (standing trim command {cf['trim_cmd_deg']:+.2f} deg), "
          f"A={cf['A']:.4f} B={cf['B']:.3f} (trade off), RMS {cf['rms_dps']:.1f} of {cf['flight_rms_dps']:.1f} dps")
    result = dict(flight=os.path.basename(args.flight_bin), motor=args.motor, launch_angle_deg=args.launch_angle,
                  controller=gains, kick=kk, coast=cf)
    if not args.no_closed_loop:
        print("closed-loop fit (each eval is one sim run)...")
        result["closed_loop"] = closed_loop_fit(tr, args, gains, cf["misalign_deg"], kk, args.out)
        cl = result["closed_loop"]
        print(f"closed-loop optimum: A={cl['A']:.4f} B={cl['B']:.3f} tau={cl['tau_s'] * 1e3:.0f} ms "
              f"delay={cl['delay_s'] * 1e3:.0f} ms (kick fixed at {cl['kick_dps']:.0f} dps)  score {cl['score']:.3f} over 0.62 s on")
        for (lo, hi), (e, r) in zip(cl["windows"], cl["window_err_and_flight_rms"]):
            print(f"   {lo:4.2f}-{hi:4.2f} s: err RMS {e:6.1f} dps vs flight RMS {r:6.1f}")
    with open(os.path.join(args.out, "fit.json"), "w") as fh:
        json.dump(result, fh, indent=1)
    print("wrote", os.path.join(args.out, "fit.json"))


if __name__ == "__main__":
    main()
