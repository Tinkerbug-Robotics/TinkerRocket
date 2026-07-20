"""Dryden gust model tests (#170).

Most statistical tests here drive the filter at a FIXED airspeed and altitude
and at a coarse dt: the recursions are exact at any step size, so a 100 Hz
record gives the same statistics as a 10 kHz one for a thousandth of the
runtime. dt-independence is asserted separately rather than assumed.

BUT constant-parameter tests are structurally blind to the thing this model is
built around. The filter's gains are recomputed every step from live airspeed
and altitude, and with those held fixed an implementation that hoists the gain
out of the recursion is mathematically IDENTICAL to the correct one — so the
whole suite below can pass while the central design invariant is violated.
That is not hypothetical: it is exactly what an adversarial review found after
this file already had 21 passing tests.

So the time-varying path gets its own section at the end of this file, and
anything asserting a property of the varying-parameter dynamics belongs there.

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


# --- TIME-VARYING PARAMETERS ------------------------------------------------
#
# Everything above holds V and h fixed. These do not, and they are the only
# tests here capable of failing when the varying-parameter path is wrong: with
# constant parameters, hoisting the gain out of the recursion produces bitwise
# identical output, so no amount of variance/autocorrelation testing at a
# fixed operating point can see it.
#
# Three independent properties, cheapest first:
#   1. the gust does not teleport when a parameter jumps
#   2. it relaxes toward the new intensity at the Dryden rate, not instantly
#   3. airspeed enters only as distance travelled through the gust field


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


def test_variance_relaxes_at_the_dryden_rate_after_an_altitude_change():
    """Not just "does not teleport" — it must relax at the RIGHT rate.

    Step an ensemble from h_low to h_high and watch the ensemble variance
    decay from sigma_low^2 toward sigma_high^2. For the exact OU transition
    the discrete solution is closed-form:

        Var(t) = sigma_hi^2 + (Var(0) - sigma_hi^2) * exp(-2*V*t/L_hi)

    (variance relaxes at twice the amplitude rate, hence the 2).

    The rate is recovered by FITTING the whole decay rather than checking
    individual points. Per-point assertions were tried first and are the wrong
    shape: as the excess decays, the remaining signal shrinks while the
    sampling noise does not, so late points swing wildly (one seed block came
    out 27% off a 15% tolerance). A log-linear fit over the well-conditioned
    part of the curve lands within 14% across every seed block tried.

    The two failure modes this separates:
      - hoisted gain  -> Var(0) is already sigma_hi^2, i.e. no relaxation at
        all. Part 1 catches it with a >3x margin.
      - right initial value, wrong dynamics (a stale or hard-coded L or V in
        the coefficient) -> part 2. A mutant using the free-air scale length
        everywhere fits at 0.57x the true rate, well outside the 0.30 band.
    """
    v = 90.0
    h_low, h_high = H_FLOOR_M, 900.0 * _FT
    L_low, _, sigma_low, _ = dryden_scales(h_low, W20_MODERATE)
    L_high, _, sigma_high, _ = dryden_scales(h_high, W20_MODERATE)

    # A large ensemble is what makes the rate fit meaningful; the sampling
    # spread of a variance estimate goes as sqrt(2/N). The burn-in is the
    # expensive part, so it runs at 3 s -- ~6 relaxation times at h_low
    # (tau = L_low/v = 0.49 s), which leaves a residual of order 1e-5.
    n_members, dt, n_steps = 1200, 5e-3, 300
    members = [DrydenGust(W20_MODERATE, seed=90000 + i) for i in range(n_members)]
    for g in members:
        g.reset(h_agl_m=h_low, v_mps=v, burn_in_s=3.0)
    assert 3.0 > 6.0 * (L_low / v)          # burn-in really is long enough

    var = np.empty(n_steps)
    for k in range(n_steps):
        var[k] = np.mean([g.step(dt, v, h_high)[0] ** 2 for g in members])

    # 1. The carried state is still at the OLD intensity right after the jump.
    assert var[0] / sigma_high**2 > 2.5           # correct ~3.5, hoisted ~1.0
    assert var[0] == pytest.approx(sigma_low**2, rel=0.20)

    # 2. ...and decays toward the new one at the Dryden rate. Fit only where
    # the excess is still well above the noise; beyond that the log is
    # dominated by sampling error.
    t = np.arange(1, n_steps + 1) * dt
    excess = var - sigma_high**2
    usable = excess > 0.25 * excess[0]
    assert usable.sum() > 100                     # enough lever arm to fit
    fitted_rate = -np.polyfit(t[usable], np.log(excess[usable]), 1)[0]
    assert fitted_rate == pytest.approx(2.0 * v / L_high, rel=0.30)

    # 3. Sanity: it decayed, and toward the right target rather than past it.
    assert var[-1] < 0.75 * var[0]
    assert var[-1] > sigma_high**2


def test_airspeed_enters_only_as_distance_travelled():
    """Every filter coefficient depends on V and dt only through their product
    — Taylor's frozen-field hypothesis: what matters is how far the vehicle
    moved through the gust field, not how fast or for how long separately.

    So halving dt while doubling V must reproduce the record exactly, sample
    for sample, given the same noise draws. This is an EXACT equality with no
    statistics, and it pins the airspeed dependence that no constant-V test
    can: a hard-coded or stale V still passes every fixed-operating-point
    test in this file, but breaks this one immediately.
    """
    h = 200.0
    slow = _record(W20_MODERATE, 45.0, h, 4e-3, 500, seed=4)
    fast = _record(W20_MODERATE, 90.0, h, 2e-3, 500, seed=4)
    assert np.array_equal(slow, fast)

    # Guard against a degenerate pass: V must actually be doing something, so
    # changing it WITHOUT compensating dt has to change the record.
    uncompensated = _record(W20_MODERATE, 90.0, h, 4e-3, 500, seed=4)
    assert not np.allclose(slow, uncompensated)


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
    """Count non-finite cells. A poisoned wind vector propagates into
    v_air_mag, which silently switches off both the aero and fin-torque gates
    rather than raising, so this is the check that the gust reached the
    physics intact. (This was a differential count against a calm run until
    the row-0 baro_alt hole was fixed; see tests/test_log_columns.py.)"""
    num = result.df.select_dtypes('number').to_numpy(dtype=float)
    return int((~np.isfinite(num)).sum())


def test_gust_perturbs_angle_of_attack():
    """The gust has to actually reach the aerodynamics. AoA is the signal it
    drives (a uniform gust exerts no roll moment on a symmetric airframe)."""
    calm = _short_flight()
    windy = _short_flight(gust_w20_mps=W20_MODERATE)
    assert windy.df['alpha_deg'].max() > calm.df['alpha_deg'].max()
    assert _n_nonfinite(calm) == 0
    assert _n_nonfinite(windy) == 0
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
    assert _n_nonfinite(both) == 0
