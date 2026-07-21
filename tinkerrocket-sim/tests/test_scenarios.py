"""Canonical closed-loop regression scenarios (#170).

Four end-to-end scenarios with pass/fail thresholds, running the real firmware
controllers (TR_ServoControl / TR_GuidancePN) against the validated RollyPolly
III plant:

  (a) nominal coast guidance to an overhead aim point — CPA improvement
  (b) roll-angle profile tracking, no disturbance
  (c) roll tracking under a boost-roll perturbation (2026-05-17-like regression)
  (d) sensor degradation — gyro full-scale clipping + GNSS dropout

Thresholds are grounded in observed behavior with margin; runs are seeded so
CI is deterministic.  Roll pass/fail is stated on the body roll RATE, which is
free of the body-Z azimuth artifact that inflates the angle error near apogee.
"""
import dataclasses

import numpy as np
import pytest

from tinkerrocket_sim.simulation.closed_loop_sim import run_closed_loop
from tinkerrocket_sim.simulation import scenarios as S
from tinkerrocket_sim.simulation import metrics as M


def _run_to_apogee(cfg):
    df = run_closed_loop(S.build_rollypolly_iii(), cfg).df
    apogee_idx = int(df['altitude'].idxmax())
    return df.iloc[:apogee_idx + 1].reset_index(drop=True)


def _coast(df, t0=4.0, t1=7.0):
    return df[(df['time'] > t0) & (df['time'] < t1)]


# --- (b) roll-angle tracking, no disturbance ---------------------------------

def test_scenario_b_roll_tracking_clean():
    df = _run_to_apogee(S.roll_tracking_clean_config())
    s = M.summarize_roll(df)

    assert 300.0 < df['altitude'].max() < 420.0        # sane G80T flight
    assert s['peak_roll_rate_dps'] < 60.0               # no disturbance -> modest
    assert abs(s['final_roll_rate_dps']) < 8.0          # controller nulls
    assert _coast(df)['roll_rate_dps'].abs().median() < 6.0
    assert s['fin_saturation_pct'] == 0.0               # never pinned
    assert s['peak_fin_deg'] < 10.0


# --- (c) roll tracking under a boost-roll perturbation (05-17-like) -----------

def test_scenario_c_boost_roll_disturbance():
    cfg = S.roll_boost_disturbance_config()
    df = _run_to_apogee(cfg)
    s = M.summarize_roll(df)

    # The 120 dps kick shows up in the true rate, and is NOT amplified.
    assert 115.0 < s['peak_roll_rate_dps'] < 150.0
    # Controller nulls the kick back down and holds through coast.
    assert abs(s['final_roll_rate_dps']) < 8.0
    assert _coast(df)['roll_rate_dps'].abs().median() < 6.0
    # Settles to within 5 dps within a few seconds of the kick.
    settle = M.settling_time(df['time'].to_numpy(), df['roll_rate_dps'].to_numpy(),
                             target=0.0, tol=5.0,
                             start_time=cfg.roll_kick_time_s + 0.05)
    assert settle is not None and settle < 6.0
    assert s['fin_saturation_pct'] == 0.0


# --- (d) sensor degradation: gyro saturation + GNSS dropout ------------------

def test_scenario_d_sensor_degradation():
    cfg = S.sensor_degradation_config()
    df = _run_to_apogee(cfg)

    # The gyro actually clipped: logged rate pinned at the range while the true
    # body rate exceeded it.
    logged_gyro_peak = df['imu_gyro_x'].abs().max()
    true_rate_peak = df['roll_rate_dps'].abs().max()
    assert logged_gyro_peak <= cfg.gyro_full_scale_dps + 1.0
    assert true_rate_peak > cfg.gyro_full_scale_dps + 5.0   # clip genuinely bit

    # GNSS dropped out under the high-g / high-spin boost.
    assert (df['gnss_valid'] == False).any()

    # Despite the degraded gyro, the controller still nulls and stays bounded.
    s = M.summarize_roll(df)
    assert abs(s['final_roll_rate_dps']) < 10.0
    assert s['peak_roll_rate_dps'] < 150.0
    assert np.isfinite(df['altitude'].max()) and 300.0 < df['altitude'].max() < 420.0


# --- (a) nominal coast guidance to an overhead aim point ----------------------

def test_scenario_a_guidance_cpa_improvement():
    pytest.importorskip(
        "tinkerrocket_sim._guidance",
        reason="TR_GuidancePN submodule not initialized — _guidance extension "
               "not built. Enable: git submodule update --init "
               "tinkerrocket-idf/components/TR_GuidancePN",
    )
    cfg = S.guidance_coast_config()
    df_g = _run_to_apogee(cfg)
    df_u = _run_to_apogee(dataclasses.replace(cfg, guidance_enabled=False))

    assert 300.0 < df_g['altitude'].max() < 420.0

    # Guidance engaged for the bulk of coast and stayed well within its tilt cap.
    assert (df_g['guidance_active'] == True).mean() > 0.5
    guided = df_g[df_g['guidance_active'] == True]
    assert guided['pn_los_angle'].abs().max() < 20.0

    # CPA: guidance must materially reduce horizontal drift off the pad axis
    # vs. the otherwise-identical ballistic (unguided) coast.
    final_g = float(np.hypot(df_g['x'].iloc[-1], df_g['y'].iloc[-1]))
    final_u = float(np.hypot(df_u['x'].iloc[-1], df_u['y'].iloc[-1]))
    assert final_g < 0.6 * final_u


# --- (a2) station-keep A/B: #534 acceptance pin -------------------------------

def test_scenario_a2_station_keep_beats_pn_no_cpa():
    """#534 acceptance: station-keep <= PN drift on the canonical guided
    scenario, with no CPA (singularity-path) quit.  Invariants, not constants:
    the full 144-run study lives in docs/plans/534-station-keep-sim-ab.md and
    scripts/ab534_pn_vs_station_keep.py regenerates it."""
    pytest.importorskip(
        "tinkerrocket_sim._guidance",
        reason="TR_GuidancePN submodule not initialized — _guidance extension "
               "not built. Enable: git submodule update --init "
               "tinkerrocket-idf/components/TR_GuidancePN",
    )
    cfg = S.guidance_coast_config()
    df_pn = _run_to_apogee(cfg)
    df_sk = _run_to_apogee(dataclasses.replace(cfg, guidance_mode='station_keep'))

    assert 300.0 < df_sk['altitude'].max() < 420.0

    # Both laws guided for the bulk of coast (neither latched out).
    assert (df_sk['guidance_active'] == True).mean() > 0.5
    assert (df_pn['guidance_active'] == True).mean() > 0.5

    # Station-keep must not do worse than PN on apogee drift (the #534
    # criterion), and must in fact materially beat it on this scenario
    # (study: 3.0 m median vs 13.8 m over 5 seeds; margin below is generous).
    final_sk = float(np.hypot(df_sk['x'].iloc[-1], df_sk['y'].iloc[-1]))
    final_pn = float(np.hypot(df_pn['x'].iloc[-1], df_pn['y'].iloc[-1]))
    assert final_sk <= final_pn
    assert final_sk < 0.6 * final_pn

    # No singularity behavior: station-keep never rides the closing-velocity
    # CPA path (the library hard-codes cpa_reached_ False in this law), so its
    # guided window must run to the low-speed gate near apogee, and its late
    # accel commands stay far from the 20 m/s^2 clamp that a 1/range blow-up
    # pins (study: PN with a reachable target pins 20.0; SK shows ~2.7).
    guided_sk = df_sk[df_sk['guidance_active'] == True]
    late = guided_sk[guided_sk['time'] > guided_sk['time'].iloc[-1] - 1.5]
    late_a = np.hypot(late['pn_a_e'], late['pn_a_n']).max()
    assert late_a < 10.0
    assert float(guided_sk['speed'].iloc[-1]) < 12.0  # exited via the speed gate


# --- (a3) station-keep with an OFFSET aim point: #435 SIL pin ------------------

def test_scenario_a3_station_keep_offset_target():
    """#435 SIL: station-keep converges toward a commanded off-pad aim point
    (the sim-side twin of BLE cmd 28 -> LLA->ENU -> setHorizontalTarget).
    This is the mutation of a2's fixed-(0,0)-target assumption: a regression
    that ignores the target and converges on the pad fails the toward-target
    invariants below.  Invariants, not constants."""
    pytest.importorskip(
        "tinkerrocket_sim._guidance",
        reason="TR_GuidancePN submodule not initialized — _guidance extension "
               "not built. Enable: git submodule update --init "
               "tinkerrocket-idf/components/TR_GuidancePN",
    )
    # r = 15 m: inside the FC's 100 m aim-radius gate, large vs a2's ~3 m
    # overhead residual, well inside the vehicle's coast authority.
    target_e, target_n = 12.0, -9.0
    cfg = S.guidance_coast_config()
    df = _run_to_apogee(dataclasses.replace(
        cfg, guidance_mode='station_keep',
        pn_target_e_m=target_e, pn_target_n_m=target_n))

    # Same flight sanity as a2.
    assert 300.0 < df['altitude'].max() < 420.0
    assert (df['guidance_active'] == True).mean() > 0.5

    # Load-bearing pair: it converged toward the POINT, not the pad.
    dist_to_target = float(np.hypot(df['x'].iloc[-1] - target_e,
                                    df['y'].iloc[-1] - target_n))
    dist_to_pad = float(np.hypot(df['x'].iloc[-1], df['y'].iloc[-1]))
    assert dist_to_target < 7.5           # 0.5*r (a2 overhead residual ~3 m)
    assert dist_to_target < dist_to_pad   # demonstrably steered off-vertical

    # Redundant magnitude bracket on the pad-relative offset (~= r = 15 m).
    assert 7.5 < dist_to_pad < 22.5

    # No CPA/singularity path (a2 pattern): late guided accel stays far from
    # the clamp and the guided window runs to low speed near apogee (observed:
    # coast tilt latch at 20 deg fires at ~10 m/s — an offset target demands
    # more tilt as speed drops; that is a low-speed exit, NOT an early
    # CPA/singularity quit, which the speed bound below would catch).
    guided = df[df['guidance_active'] == True]
    late = guided[guided['time'] > guided['time'].iloc[-1] - 1.5]
    assert np.hypot(late['pn_a_e'], late['pn_a_n']).max() < 10.0
    assert float(guided['speed'].iloc[-1]) < 12.0
