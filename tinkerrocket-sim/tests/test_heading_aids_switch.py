"""#1281/#1282: the two GNSS-derived heading aids ship OFF, and a replay has to
be able to turn them back on to score them against flight data.

This is the in-tree record of what that switch is worth, so the cost of the
decision is a measured number in CI rather than a claim in an issue.
"""
import dataclasses

import pytest

import tinkerrocket_sim._ekf as _ekf
from tinkerrocket_sim.simulation import scenarios as S
from tinkerrocket_sim.simulation import metrics as M


def test_aids_are_off_by_default():
    ekf = _ekf.GpsInsEKF()
    assert ekf.gnss_heading_aids_enabled() is False


def test_the_switch_round_trips():
    ekf = _ekf.GpsInsEKF()
    ekf.set_gnss_heading_aids(True)
    assert ekf.gnss_heading_aids_enabled() is True
    ekf.set_gnss_heading_aids(False)
    assert ekf.gnss_heading_aids_enabled() is False


def _roll_metrics(fuse):
    """Run the boost-roll-disturbance scenario with the aids forced on/off."""
    import tinkerrocket_sim.simulation.closed_loop_sim as C
    original = _ekf.GpsInsEKF

    class Patched(original):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self.set_gnss_heading_aids(fuse)

    _ekf.GpsInsEKF = Patched
    try:
        cfg = S.roll_boost_disturbance_config()
        df = C.run_closed_loop(S.build_rollypolly_iii(), cfg).df
    finally:
        _ekf.GpsInsEKF = original
    apogee = int(df['altitude'].idxmax())
    df = df.iloc[:apogee + 1].reset_index(drop=True)
    coast = df[(df['time'] > 4.0) & (df['time'] < 7.0)]
    settle = M.settling_time(df['time'].to_numpy(), df['roll_rate_dps'].to_numpy(),
                             target=0.0, tol=5.0,
                             start_time=cfg.roll_kick_time_s + 0.05)
    return float(coast['roll_rate_dps'].abs().median()), settle


def test_switching_the_aids_on_tightens_the_coast_roll_null():
    # Measured 2026-09-09: 6.41 dps with the aids off, 1.34 dps with them fused.
    # The sim's GNSS velocity carries no noise, so course-over-ground near
    # vertical is genuinely informative there and the loop leans on it. On the
    # four 2026-08-29 flights that same measurement is not informative — see
    # #1281 — which is why the vehicle flies with these off despite this number.
    off_med, _ = _roll_metrics(False)
    on_med, _ = _roll_metrics(True)
    assert on_med < off_med
    assert on_med < 3.0
    assert 4.0 < off_med < 9.0


def test_the_default_path_matches_the_explicitly_disabled_path():
    # Nothing else may be quietly enabling them.
    from tinkerrocket_sim.simulation import scenarios as _S
    import tinkerrocket_sim.simulation.closed_loop_sim as C
    cfg = _S.roll_boost_disturbance_config()
    df = C.run_closed_loop(_S.build_rollypolly_iii(), cfg).df
    apogee = int(df['altitude'].idxmax())
    coast = df.iloc[:apogee + 1]
    coast = coast[(coast['time'] > 4.0) & (coast['time'] < 7.0)]
    default_med = float(coast['roll_rate_dps'].abs().median())
    off_med, _ = _roll_metrics(False)
    assert default_med == pytest.approx(off_med, rel=1e-6)
