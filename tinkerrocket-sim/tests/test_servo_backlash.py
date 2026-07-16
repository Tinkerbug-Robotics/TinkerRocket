"""Servo mechanical-backlash model tests (#170).

Unit tests of the backlash operator plus end-to-end checks that linkage lash
degrades roll control the way it does on real hardware (the known-bad servo
measured ~10 deg of lash and could not hold roll).  The gap=0 rigid-linkage
identity is covered by the canonical scenario thresholds in test_scenarios.py,
which run the same actuator path with backlash off.
"""
import dataclasses

import pytest

from tinkerrocket_sim.simulation.closed_loop_sim import (
    _backlash_step, run_closed_loop)
from tinkerrocket_sim.simulation import scenarios as S
from tinkerrocket_sim.simulation import metrics as M


# --- operator unit tests ------------------------------------------------------

def test_zero_gap_is_rigid():
    assert _backlash_step(0.0, 3.7, 0.0) == 3.7
    assert _backlash_step(5.0, -2.0, 0.0) == -2.0


def test_holds_inside_gap():
    # gap 2 -> half-gap 1: input within ±1 of output leaves output unchanged
    assert _backlash_step(0.0, 0.9, 2.0) == 0.0
    assert _backlash_step(0.0, -0.9, 2.0) == 0.0


def test_engages_at_half_gap():
    # input beyond the half-gap drags the output along, offset by half the gap
    assert _backlash_step(0.0, 1.5, 2.0) == pytest.approx(0.5)
    assert _backlash_step(0.0, -1.5, 2.0) == pytest.approx(-0.5)


def test_reversal_traverses_full_gap():
    gap = 2.0
    out = 0.0
    # drive up: output follows at input - half
    out = _backlash_step(out, 3.0, gap)
    assert out == pytest.approx(2.0)
    # reverse: input must cross the FULL gap before the output moves again
    out2 = _backlash_step(out, 1.1, gap)   # within out ± 1 -> holds
    assert out2 == pytest.approx(2.0)
    out3 = _backlash_step(out2, 0.9, gap)  # now beyond -> engages at input + half
    assert out3 == pytest.approx(1.9)


# --- end-to-end: lash degrades roll control -----------------------------------

def _coast_median_rate(gap_deg):
    cfg = dataclasses.replace(S.roll_boost_disturbance_config(),
                              servo_backlash_deg=gap_deg)
    df = run_closed_loop(S.build_rollypolly_iii(), cfg).df
    apogee_idx = int(df['altitude'].idxmax())
    df = df.iloc[:apogee_idx + 1].reset_index(drop=True)
    coast = df[(df['time'] > 4.0) & (df['time'] < 7.0)]
    settle = M.settling_time(df['time'].to_numpy(),
                             df['roll_rate_dps'].to_numpy(),
                             target=0.0, tol=5.0,
                             start_time=cfg.roll_kick_time_s + 0.05)
    return float(coast['roll_rate_dps'].abs().median()), settle


def test_moderate_lash_degrades_roll_null():
    # 2 deg of lash: the loop limit-cycles instead of settling into the 5 dps
    # band, and the coast residual grows well past the rigid-linkage ~1.4 dps
    # (still bounded — the controller fights through the slack).
    med, settle = _coast_median_rate(2.0)
    assert settle is None
    assert 5.0 < med < 25.0


def test_severe_lash_loses_roll_control():
    # 10 deg of lash (the measured gap on the known-bad servo): commands live
    # inside the slack, the fin barely engages, and coast roll runs away.
    med, settle = _coast_median_rate(10.0)
    assert settle is None
    assert med > 20.0
