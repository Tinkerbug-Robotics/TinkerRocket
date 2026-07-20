"""Dryden gust model tests (#170).

The statistical tests drive the filter at a FIXED airspeed and altitude and at
a coarse dt: the recursions are exact at any step size, so a 100 Hz record
gives the same statistics as a 10 kHz one for a thousandth of the runtime.
dt-independence is asserted separately rather than assumed.

Variances use mean(x**2), not np.var: the process mean is known to be zero,
and subtracting the *sample* mean of a strongly autocorrelated record biases
the estimate low by roughly 2*tau/T.
"""
import math

import numpy as np
import pandas as pd
import pytest

from tinkerrocket_sim.physics.dryden import (
    DrydenGust, dryden_scales, L_FREE_AIR_M, W20_MODERATE, H_FLOOR_M,
)
from tinkerrocket_sim.simulation.closed_loop_sim import SimConfig, run_closed_loop
from tinkerrocket_sim.simulation.scenarios import build_rollypolly_iii

_FT = 0.3048


def _record(w20, v, h, dt, n, seed, vertical=True):
    """Drive the gust at constant V and h; return an (n, 3) ENU record."""
    g = DrydenGust(w20, seed=seed, vertical=vertical)
    return np.array([g.step(dt, v, h) for _ in range(n)])


def _acf(x, lag):
    return float(np.mean(x[:-lag] * x[lag:]) / np.mean(x * x))


# --- scale lengths / intensities ------------------------------------------

def test_low_altitude_form_matches_mil_f_8785b():
    """Spot-check the published low-altitude formulas at 20 ft."""
    h = 20.0 * _FT
    L_u, L_w, s_u, s_w = dryden_scales(h, 15.4)
    den = 0.177 + 0.000823 * 20.0
    assert L_w == pytest.approx(h)                      # L_w = h
    assert L_u == pytest.approx(h / den**1.2)
    assert s_w == pytest.approx(0.1 * 15.4)
    assert s_u == pytest.approx(s_w / den**0.4)
    assert s_u > s_w                                    # horizontal is stronger near the ground


def test_scales_continuous_at_1000ft():
    """The low-altitude form and the blend must meet without a kink — a step
    here would read as a real disturbance during the transit."""
    below = dryden_scales((1000.0 - 1e-3) * _FT, 15.4)
    above = dryden_scales((1000.0 + 1e-3) * _FT, 15.4)
    for b, a in zip(below, above):
        assert b == pytest.approx(a, abs=1e-3)
    # and both scale lengths have converged to h = 1000 ft there
    assert below[0] == pytest.approx(below[1], abs=1e-3)
    assert below[2] == pytest.approx(below[3], abs=1e-6)


def test_scales_reach_free_air_above_2000ft():
    L_u, L_w, s_u, s_w = dryden_scales(2500.0 * _FT, 15.4)
    assert L_u == pytest.approx(L_FREE_AIR_M)
    assert L_w == pytest.approx(L_FREE_AIR_M)
    assert s_u == pytest.approx(0.1 * 15.4)


def test_altitude_floor_applied():
    """Below the 20 ft floor, L_w = h would collapse to zero."""
    assert dryden_scales(0.0, 15.4) == dryden_scales(H_FLOOR_M, 15.4)


# --- filter statistics -----------------------------------------------------

def test_horizontal_variance():
    v, h, w20 = 90.0, 300.0, W20_MODERATE
    _, _, sigma_u, _ = dryden_scales(h, w20)
    rec = _record(w20, v, h, 1e-2, 400_000, 1)
    for axis in (0, 1):
        assert math.sqrt(np.mean(rec[:, axis] ** 2)) == pytest.approx(sigma_u, rel=0.05)


def test_vertical_variance():
    v, h, w20 = 90.0, 300.0, W20_MODERATE
    _, _, _, sigma_w = dryden_scales(h, w20)
    rec = _record(w20, v, h, 1e-2, 400_000, 2)
    assert math.sqrt(np.mean(rec[:, 2] ** 2)) == pytest.approx(sigma_w, rel=0.05)


def test_horizontal_autocorrelation_is_first_order():
    """R_u(L/V) = 1/e for the 1st-order (exponential) spectrum."""
    v, h, w20 = 90.0, 300.0, W20_MODERATE
    L_u, _, _, _ = dryden_scales(h, w20)
    dt = 1e-2
    rec = _record(w20, v, h, dt, 600_000, 3)
    lag = int(round((L_u / v) / dt))
    assert _acf(rec[:, 0], lag) == pytest.approx(1.0 / math.e, abs=0.02)


_ACF_V, _ACF_H, _ACF_DT = 90.0, 60.0, 1e-2


@pytest.fixture(scope="module")
def vertical_record():
    """One long vertical record, shared by the autocorrelation tests — it is
    the most expensive thing in this file and both tests want the same run."""
    return _record(W20_MODERATE, _ACF_V, _ACF_H, _ACF_DT, 1_500_000, 4)[:, 2]


def test_vertical_autocorrelation_matches_dryden(vertical_record):
    """R_w(t) = e^(-Vt/L) (1 - Vt/2L) — the shape that distinguishes the true
    2nd-order vertical spectrum from an accidentally 1st-order one."""
    _, L_w, _, _ = dryden_scales(_ACF_H, W20_MODERATE)
    for mult in (0.5, 1.0, 1.5):
        lag = int(round(mult * (L_w / _ACF_V) / _ACF_DT))
        t = lag * _ACF_DT
        want = math.exp(-_ACF_V * t / L_w) * (1.0 - _ACF_V * t / (2.0 * L_w))
        assert _acf(vertical_record, lag) == pytest.approx(want, abs=0.03)


def test_vertical_autocorrelation_reverses_sign(vertical_record):
    """The 2nd-order spectrum crosses zero at t = 2L/V and goes negative
    beyond it; a 1st-order filter never does."""
    _, L_w, _, _ = dryden_scales(_ACF_H, W20_MODERATE)
    lag_of = lambda m: int(round(m * (L_w / _ACF_V) / _ACF_DT))
    assert abs(_acf(vertical_record, lag_of(2.0))) < 0.03
    assert _acf(vertical_record, lag_of(3.0)) < 0.0


def test_variance_independent_of_timestep():
    """Catches a missing sqrt(dt) normalization, which would scale RMS by
    sqrt(10) per decade and is otherwise invisible."""
    v, h, w20 = 90.0, 300.0, W20_MODERATE
    _, _, sigma_u, _ = dryden_scales(h, w20)
    rms = []
    for dt, n in ((1e-2, 400_000), (1e-3, 1_000_000)):
        rec = _record(w20, v, h, dt, n, 6)
        rms.append(math.sqrt(np.mean(rec[:, 0] ** 2)))
    assert rms[0] == pytest.approx(rms[1], rel=0.10)
    assert rms[0] == pytest.approx(sigma_u, rel=0.10)


def test_components_are_independent():
    """A shared RNG draw across axes would show up as correlation.

    Run near the ground, where the scale lengths (and so the correlation
    times) are shortest: the useful sample count is T/(2*tau), not the raw
    step count.  At h = 20 ft and V = 90 the slowest component has
    tau = L_u/V ~ 0.5 s, so a 3000 s record carries ~3000 effective samples
    and the sampling std of the correlation is ~0.018.  0.08 is ~4 sigma.
    """
    rec = _record(W20_MODERATE, 90.0, H_FLOOR_M, 1e-2, 300_000, 7)
    c = np.corrcoef(rec.T)
    for i in range(3):
        for j in range(i + 1, 3):
            assert abs(c[i, j]) < 0.08


def test_intensity_scales_linearly():
    a = _record(W20_MODERATE, 90.0, 300.0, 1e-2, 100_000, 8)
    b = _record(2.0 * W20_MODERATE, 90.0, 300.0, 1e-2, 100_000, 8)
    assert np.allclose(b, 2.0 * a, rtol=1e-9)


# --- guards and API --------------------------------------------------------

def test_finite_at_zero_airspeed_and_altitude():
    """V -> 0 (pad, apogee) and h -> 0 (off the rail) are both real states in
    this flight profile and both are singular without the floors."""
    rec = _record(W20_MODERATE, 0.0, 0.0, 1e-3, 2000, 9)
    assert np.all(np.isfinite(rec))


def test_deterministic_for_a_given_seed():
    a = _record(W20_MODERATE, 90.0, 300.0, 1e-2, 500, 10)
    b = _record(W20_MODERATE, 90.0, 300.0, 1e-2, 500, 10)
    c = _record(W20_MODERATE, 90.0, 300.0, 1e-2, 500, 11)
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)


def test_vertical_flag_zeroes_up_without_disturbing_horizontals():
    on = _record(W20_MODERATE, 90.0, 300.0, 1e-2, 500, 12, vertical=True)
    off = _record(W20_MODERATE, 90.0, 300.0, 1e-2, 500, 12, vertical=False)
    assert np.all(off[:, 2] == 0.0)
    assert np.array_equal(on[:, :2], off[:, :2])


def test_reset_burn_in_leaves_filter_stationary():
    """After the burn-in the state should already be a typical sample, not
    parked near zero."""
    _, _, sigma_u, _ = dryden_scales(300.0, W20_MODERATE)
    finals = []
    for seed in range(60):
        g = DrydenGust(W20_MODERATE, seed=seed)
        g.reset(h_agl_m=300.0, v_mps=90.0)
        finals.append(g.step(1e-2, 90.0, 300.0)[0])
    assert math.sqrt(np.mean(np.square(finals))) == pytest.approx(sigma_u, rel=0.35)


def test_zero_intensity_rejected():
    with pytest.raises(ValueError):
        DrydenGust(0.0)


def test_gust_is_continuous_across_an_altitude_change():
    """The gain must stay INSIDE the recursion, not be hoisted out as a
    unit-variance filter scaled by sigma(h) at the output.

    sigma_u varies with altitude below 1000 ft, so a hoisted gain rescales the
    whole carried state the instant h changes — the gust velocity teleports.
    Physically it must decay toward the new intensity at the filter's own rate.

    Taking the altitude step with a very short dt makes this a wide-margin
    check rather than a statistical one: the only change the true filter can
    make in that step is the noise it injects, of order sigma*sqrt(2*V*dt/L)
    (~0.1% here), whereas a hoisted gain jumps by the full sigma ratio (~46%).
    The 2% threshold sits about 20x above the former and 20x below the latter.

    This is the one invariant the module docstring singles out as its reason
    for existing, and it is invisible to every constant-altitude test: with h
    fixed, hoisting the gain is mathematically exact.
    """
    h_low, h_high, v = H_FLOOR_M, 900.0 * _FT, 90.0
    sigma_low = dryden_scales(h_low, W20_MODERATE)[2]
    sigma_high = dryden_scales(h_high, W20_MODERATE)[2]
    assert sigma_low > 1.8 * sigma_high      # the altitudes really do differ

    g = DrydenGust(W20_MODERATE, seed=17)
    for _ in range(4000):                     # build up state at low altitude
        before = g.step(1e-2, v, h_low)
    after = g.step(1e-6, v, h_high)           # same instant, new altitude

    assert abs(before[0]) > 0.5 * sigma_low   # there is real state to rescale
    assert after[0] == pytest.approx(before[0], rel=0.02)
    assert after[1] == pytest.approx(before[1], rel=0.02)


# --- integration with the closed-loop sim ----------------------------------

def _short_flight(**overrides):
    cfg = dict(pad_time=0.0, duration=6.0, physics_dt=1e-3,
               launch_angle_deg=85.0, control_enabled=False,
               guidance_enabled=False, enable_mag_updates=False,
               pad_heading_deg=0.0, sensor_seed=0)
    cfg.update(overrides)
    return run_closed_loop(build_rollypolly_iii(), SimConfig(**cfg))


def test_gust_off_leaves_the_sim_bit_for_bit_unchanged():
    """The knob defaults off, and off must mean the gust model is not merely
    quiet but absent — otherwise every existing scenario baseline moves."""
    a = _short_flight()
    b = _short_flight(gust_w20_mps=0.0, gust_vertical=False)
    pd.testing.assert_frame_equal(a.df, b.df)


def _n_nonfinite(result):
    """Count non-finite cells. Stated differentially against a calm run rather
    than as an absolute zero: baro_alt is already NaN in the first logged row
    of every flight, gust or not (the row precedes the first baro sample)."""
    num = result.df.select_dtypes('number').to_numpy(dtype=float)
    return int((~np.isfinite(num)).sum())


def test_gust_perturbs_angle_of_attack():
    """The gust has to actually reach the aerodynamics. AoA is the signal it
    drives (a uniform gust exerts no roll moment on a symmetric airframe)."""
    calm = _short_flight()
    windy = _short_flight(gust_w20_mps=W20_MODERATE)
    assert windy.df['alpha_deg'].max() > calm.df['alpha_deg'].max()
    assert _n_nonfinite(windy) == _n_nonfinite(calm)
    # Still a plausible flight, not a force-free coast from a poisoned wind.
    assert 0.5 * calm.apogee_m < windy.apogee_m < 1.5 * calm.apogee_m


def test_gust_is_reproducible_and_seed_dependent():
    a = _short_flight(gust_w20_mps=W20_MODERATE, sensor_seed=3)
    b = _short_flight(gust_w20_mps=W20_MODERATE, sensor_seed=3)
    c = _short_flight(gust_w20_mps=W20_MODERATE, sensor_seed=4)
    pd.testing.assert_frame_equal(a.df, b.df)
    assert a.df['alpha_deg'].max() != c.df['alpha_deg'].max()


def test_gust_composes_with_steady_wind():
    """Steady wind and gust are independent knobs that add."""
    steady = _short_flight(wind_speed=5.0, wind_direction_deg=90.0)
    both = _short_flight(wind_speed=5.0, wind_direction_deg=90.0,
                         gust_w20_mps=W20_MODERATE)
    assert both.df['alpha_deg'].max() != steady.df['alpha_deg'].max()
    assert _n_nonfinite(both) == _n_nonfinite(steady)
