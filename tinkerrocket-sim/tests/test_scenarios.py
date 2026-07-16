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
