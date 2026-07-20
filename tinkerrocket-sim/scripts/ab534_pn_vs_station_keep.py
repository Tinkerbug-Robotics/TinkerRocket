#!/usr/bin/env python3
"""#534 acceptance study: PN vs station-keep A/B on the canonical guided
scenario (scenario (a), RollyPolly III / G80T).

Regenerates the full 144-run matrix deterministically (every run is seeded;
no other RNG) and emits one JSON object per run as JSONL:

    python3 scripts/ab534_pn_vs_station_keep.py            # full matrix -> stdout
    python3 scripts/ab534_pn_vs_station_keep.py --legs core gains  # subset

Results and conclusions: docs/plans/534-station-keep-sim-ab.md (repo root).

Matrix legs (case_id prefixes):
  core  ab_*        2 laws + unguided x 5 seeds x {calm, wind4, wind8, gust}
  gains gain_*      station-keep kp x kd 3x3 grid, calm + wind8, seed 7
  sing  sing_*      PN target_alt BELOW apogee (reachable -> CPA) vs SK, same cfg
  ang   ang_*       80/88 deg launch angle, both laws
  hdg   hdg*_*      EKF heading bias 10-90 deg (the known real-vehicle defect)
  gnss  gnssdeny_*  GNSS updates disabled in flight (EKF coasts on IMU)
  lotto lotto_*     gust latch lottery: 20 seeds, both laws, count tilt latches

Metric notes (learned the hard way; see the report):
  - final_horiz_offset_m from metrics.summarize_guidance is the offset at the
    LAST GUIDED row.  For tilt-latched runs that is the offset at latch time,
    not apogee.  Use eff_drift (below) for apogee drift.
  - max_tilt_coast_deg covers the whole post-burnout window INCLUDING after the
    benign speed-gate quit near apogee, so values above the 20 deg coast latch
    can coexist with deactivation_cause == speed_gate (the latch only samples
    while guidance is active, on the EKF quaternion).
  - wind8 is a null condition: 8 m/s weathercocks past the coast tilt latch at
    guidance activation for BOTH laws; all wind8 rows fly the same ballistic
    arc and carry no law or gain information.
"""
import argparse
import dataclasses
import json
import sys
import os
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from tinkerrocket_sim.simulation.closed_loop_sim import run_closed_loop
from tinkerrocket_sim.simulation import scenarios as S
from tinkerrocket_sim.simulation import metrics as M

FIN_LIMIT = 20.0  # pn_max_fin_deg in the canonical config

CONDS = {
    "calm":  {},
    "wind4": {"wind_speed": 4.0, "wind_direction_deg": 270.0},
    "wind8": {"wind_speed": 8.0, "wind_direction_deg": 45.0},
    "gust":  {"wind_speed": 4.0, "wind_direction_deg": 270.0, "gust_w20_mps": 6.0},
}
SEEDS = [7, 11, 23, 42, 101]


def build_matrix(legs):
    cases = []
    if "core" in legs:
        for law in ("pn", "station_keep", "unguided"):
            for seed in SEEDS:
                for cname, cond in CONDS.items():
                    cases.append({"case_id": f"ab_{law}_{cname}_s{seed}", "law": law,
                                  "sensor_seed": seed, **cond})
    if "gains" in legs:
        for kp in (0.4, 0.8, 1.6):
            for kd in (0.75, 1.5, 3.0):
                for cname in ("calm", "wind8"):
                    cases.append({"case_id": f"gain_kp{kp}_kd{kd}_{cname}",
                                  "law": "station_keep", "sensor_seed": 7,
                                  "pn_kp_pos": kp, "pn_kd_vel": kd, **CONDS[cname]})
    if "sing" in legs:
        for alt in (200.0, 300.0):
            for law in ("pn", "station_keep"):
                cases.append({"case_id": f"sing_{law}_alt{int(alt)}", "law": law,
                              "sensor_seed": 7, "pn_target_alt_m": alt})
    if "ang" in legs:
        for ang in (80.0, 88.0):
            for law in ("pn", "station_keep"):
                cases.append({"case_id": f"ang_{law}_{int(ang)}", "law": law,
                              "sensor_seed": 7, "launch_angle_deg": ang})
    if "hdg" in legs:
        for bias in (10.0, 20.0, 45.0, 90.0):
            for law in ("pn", "station_keep"):
                cases.append({"case_id": f"hdg_{law}_b{int(bias)}", "law": law,
                              "sensor_seed": 7, "inject_heading_bias_deg": bias})
        for bias in (20.0, 45.0):
            for law in ("pn", "station_keep"):
                cases.append({"case_id": f"hdgw_{law}_b{int(bias)}", "law": law,
                              "sensor_seed": 7, "inject_heading_bias_deg": bias,
                              **CONDS["wind4"]})
    if "gnss" in legs:
        for law in ("pn", "station_keep", "unguided"):
            for seed in (7, 42):
                cases.append({"case_id": f"gnssdeny_{law}_s{seed}", "law": law,
                              "sensor_seed": seed, "enable_gnss_updates": False})
    if "lotto" in legs:
        for seed in range(201, 221):
            for law in ("pn", "station_keep"):
                cases.append({"case_id": f"lotto_{law}_s{seed}", "law": law,
                              "sensor_seed": seed, **CONDS["gust"]})
    return cases


def eff_drift(row):
    """Apogee drift: final guided offset when guidance ran to (near) apogee,
    max offset when latched early or unguided (ballistic growth is monotone)."""
    f = row["final_horiz_offset_m"]
    if row["guided_frac"] > 0.1 and f == f:
        return f
    return row["max_horiz_offset_m"]


def run_case(case):
    cfg = S.guidance_coast_config()
    law = case.get("law", "pn")
    over = {k: v for k, v in case.items() if k not in ("law", "case_id")}
    if law == "unguided":
        over["guidance_enabled"] = False
    else:
        over["guidance_mode"] = law
    cfg = dataclasses.replace(cfg, **over)

    t0 = time.time()
    df = run_closed_loop(S.build_rollypolly_iii(), cfg).df
    wall = time.time() - t0

    apogee_idx = int(df["altitude"].idxmax())
    df = df.iloc[: apogee_idx + 1].reset_index(drop=True)

    out = dict(case)
    out["wall_s"] = round(wall, 1)
    out["apogee_m"] = float(df["altitude"].max())
    out["apogee_time_s"] = float(df["time"].iloc[-1])
    out.update(M.summarize_guidance(df))

    guided = (df[df.get("guidance_active", False) == True]
              if "guidance_active" in df else df.iloc[0:0])
    out["guided_frac"] = float(len(guided) / len(df)) if len(df) else 0.0
    out["max_los_angle_deg"] = (
        float(guided["pn_los_angle"].abs().max())
        if len(guided) and "pn_los_angle" in guided else float("nan"))

    # Deactivation cause from the LAST guided row:
    #   pn_v_cl <= 0          -> CPA latch (PN's sticky singularity-path quit)
    #   speed <= min gate     -> speed gate (benign, expected near apogee)
    #   else                  -> tilt latch
    cause = "never_active"
    cpa_time = float("nan")
    if len(guided):
        last = guided.iloc[-1]
        last_idx = guided.index[-1]
        min_speed = case.get("pn_min_speed_mps", 10.0)
        if last_idx >= len(df) - 2:
            cause = "none"
        elif "pn_v_cl" in guided and float(last["pn_v_cl"]) <= 0.0:
            cause = "cpa"
            cpa_time = float(last["time"])
        elif float(last["speed"]) <= min_speed + 0.5:
            cause = "speed_gate"
        else:
            cause = "tilt_latch"
    out["deactivation_cause"] = cause
    out["cpa_reached"] = cause == "cpa"
    out["cpa_time_s"] = cpa_time

    # Tilt from the TRUTH quaternion (same nose_up formula as the sim's latch;
    # never the Euler triple).  Covers the whole window incl. post-quit.
    if "true_q0_ned" in df:
        q0 = df["true_q0_ned"].to_numpy(); q1 = df["true_q1_ned"].to_numpy()
        q2 = df["true_q2_ned"].to_numpy(); q3 = df["true_q3_ned"].to_numpy()
        nose_up = np.clip(-2.0 * (q1 * q3 - q0 * q2), -1.0, 1.0)
        tilt = np.degrees(np.arccos(nose_up))
        out["max_tilt_deg"] = float(tilt.max())
        if "burnout_detected" in df and df["burnout_detected"].any():
            out["max_tilt_coast_deg"] = float(
                tilt[df["burnout_detected"].to_numpy(bool)].max())
        else:
            out["max_tilt_coast_deg"] = float("nan")
    else:
        out["max_tilt_deg"] = float("nan")
        out["max_tilt_coast_deg"] = float("nan")

    fin_cols = [c for c in ("fin1_cmd", "fin2_cmd", "fin3_cmd", "fin4_cmd")
                if c in df.columns]
    if fin_cols and len(guided):
        fins = guided[fin_cols].to_numpy()
        pinned = (np.abs(fins) >= FIN_LIMIT - 1e-3).any(axis=1)
        out["fin_saturation_pct"] = float(100.0 * pinned.mean())
        out["peak_fin_deg"] = float(np.abs(fins).max())
    else:
        out["fin_saturation_pct"] = float("nan")
        out["peak_fin_deg"] = float("nan")

    # Singularity blow-up probe: max |a_cmd| over the last 1.5 s guided.
    acc_cols = [c for c in ("pn_a_e", "pn_a_n", "pn_a_u") if c in df.columns]
    if acc_cols and len(guided):
        t_last = float(guided["time"].iloc[-1])
        late = guided[guided["time"] > t_last - 1.5]
        out["late_max_accel_mps2"] = (
            float(np.linalg.norm(late[acc_cols].to_numpy(), axis=1).max())
            if len(late) else float("nan"))
    else:
        out["late_max_accel_mps2"] = float("nan")

    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--legs", nargs="+",
                    default=["core", "gains", "sing", "ang", "hdg", "gnss", "lotto"],
                    choices=["core", "gains", "sing", "ang", "hdg", "gnss", "lotto"])
    args = ap.parse_args()
    for case in build_matrix(args.legs):
        try:
            print(json.dumps(run_case(case)), flush=True)
        except Exception as e:
            print(json.dumps({**case, "error": repr(e)}), flush=True)


if __name__ == "__main__":
    main()
