"""Unit tests for the control-performance metrics (#170)."""
import numpy as np
import pandas as pd
import pytest

from tinkerrocket_sim.simulation import metrics as m


def test_rms_basic():
    assert m.rms([3.0, 4.0]) == pytest.approx(np.sqrt(12.5))
    assert m.rms([0.0, 0.0, 0.0]) == 0.0


def test_rms_empty_is_nan():
    assert np.isnan(m.rms([]))
    assert np.isnan(m.rms([np.nan, np.nan]))


def test_rms_tracking_error_scalar_target():
    actual = np.array([1.0, 2.0, 3.0])
    # error vs constant target 2.0 -> [-1, 0, 1] -> rms sqrt(2/3)
    assert m.rms_tracking_error(actual, 2.0) == pytest.approx(np.sqrt(2.0 / 3.0))


def test_rms_tracking_error_angular_wrap():
    # actual 179, target -179 -> true error is +2 deg (not -358)
    err = m.rms_tracking_error([179.0], [-179.0], angular=True)
    assert err == pytest.approx(2.0)
    # without wrapping it would be a huge error
    assert m.rms_tracking_error([179.0], [-179.0], angular=False) == pytest.approx(358.0)


def test_peak_abs():
    assert m.peak_abs([1.0, -5.0, 3.0]) == 5.0
    assert np.isnan(m.peak_abs([]))


def test_saturation_pct():
    cmd = np.array([0.0, 20.0, -20.0, 10.0])  # 2 of 4 pinned at ±20
    assert m.saturation_pct(cmd, -20.0, 20.0) == pytest.approx(50.0)
    assert m.saturation_pct(np.zeros(10), -20.0, 20.0) == 0.0


def test_settling_time_step():
    # signal decays past a ±0.5 band around 0 by t=2.0 and stays in
    t = np.linspace(0.0, 5.0, 501)
    x = 10.0 * np.exp(-t / 0.5)  # crosses 0.5 at t = 0.5*ln(20) ≈ 1.498
    ts = m.settling_time(t, x, target=0.0, tol=0.5)
    assert ts is not None
    assert ts == pytest.approx(0.5 * np.log(20.0), abs=0.05)


def test_settling_time_never_settles():
    t = np.linspace(0.0, 3.0, 301)
    x = np.ones_like(t) * 10.0  # stays outside any small band
    assert m.settling_time(t, x, target=0.0, tol=0.5) is None


def test_settling_time_already_settled():
    t = np.linspace(0.0, 3.0, 301)
    x = np.zeros_like(t)
    assert m.settling_time(t, x, target=0.0, tol=0.5) == 0.0


def test_summarize_roll_columns():
    df = pd.DataFrame({
        'time': np.linspace(0, 2, 5),
        'roll_rate_dps': [50.0, 20.0, 5.0, 1.0, 0.5],
        'roll_target_deg': [0.0, 0.0, 0.0, 0.0, 0.0],
        'current_roll_deg': [10.0, 4.0, 1.0, 0.2, 0.0],
        'fin_tab_cmd': [20.0, 5.0, 1.0, 0.0, 0.0],
    })
    s = m.summarize_roll(df)
    assert s['peak_roll_rate_dps'] == 50.0
    assert s['final_roll_rate_dps'] == 0.5
    assert s['peak_fin_deg'] == 20.0
    assert 0.0 <= s['fin_saturation_pct'] <= 100.0
    assert s['rms_angle_error_deg'] >= 0.0


def test_summarize_guidance_offsets():
    df = pd.DataFrame({
        'x': [0.0, 3.0, 1.0],
        'y': [0.0, 4.0, 0.0],
        'guidance_active': [False, True, True],
    })
    s = m.summarize_guidance(df)
    # guided rows: offsets 5.0 (t1) and 1.0 (t2) -> min 1.0, final 1.0
    assert s['min_horiz_offset_m'] == pytest.approx(1.0)
    assert s['final_horiz_offset_m'] == pytest.approx(1.0)
    assert s['max_horiz_offset_m'] == pytest.approx(5.0)
