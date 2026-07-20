"""Dryden atmospheric turbulence (continuous gust spectrum) — issue #170.

Generates a time-varying wind vector whose power spectral density matches the
Dryden form of MIL-F-8785B / MIL-HDBK-1797, so the sim can exercise the
guidance loop and the EKF against continuous buffeting rather than only the
steady wind and the impulsive `roll_kick_dps` perturbation.

WHAT THIS IS AND IS NOT A TEST OF
    A *uniform* gust produces exactly ZERO roll moment on this symmetric
    airframe: `sixdof.derivatives` zeroes the normal-force roll contribution,
    and every other roll term depends on the wind only through the scalar
    airspeed.  So this is a pitch/yaw, angle-of-attack, EKF-buffeting and
    dispersion stressor — NOT a roll-loop stressor.  The roll loop is
    disturbed by `roll_misalign_deg` (a V^2 fin-tab build misalignment) and
    `roll_kick_dps` (an impulsive boost transient); those remain the right
    tools for it.  The one coupling here is that the gust modulates airspeed,
    which scales the V^2 misalignment torque by ~3% 1-sigma at light
    turbulence.

FRAME — earth (ENU), not body
    The gust is generated directly in ENU and subtracted from the vehicle
    velocity, matching how `SixDOF.derivatives` already consumes wind.  East
    and North carry the horizontal (u/v) spectrum, Up carries the vertical (w)
    spectrum, which is the earth-frame reading of the standard: the vertical
    component has its own scale length (L_w = h) because the ground constrains
    vertical eddies.  Generating in the body frame would be actively wrong
    here — a constant body-frame gust becomes a *rotating* ENU gust as the
    rocket rolls, injecting a fictitious disturbance perfectly correlated with
    the very state the roll controller regulates, and the usual body triad is
    singular for a vertical rocket (i.e. for all of boost).

APPROXIMATION, stated rather than hidden
    Strict MIL-F-8785 gives the longitudinal component a 1st-order spectrum
    and the lateral a 2nd-order one.  Both horizontals here use the 1st-order
    form.  Splitting them would tie the spectral shape to the arbitrary
    East/North axes (the two are interchangeable with no mean wind), and
    horizontal isotropy matters more for this vehicle than the exact
    high-frequency rolloff of one component.  The vertical — the component
    that feeds axial velocity and therefore apogee — uses the correct
    2nd-order form.

The filter gains are recomputed from the instantaneous airspeed and altitude
on every step and stay INSIDE the recursion.  They are deliberately not
factored out as a unit-variance filter scaled by sigma(t): that shortcut is
only valid for constant V and L, and a rocket going 0 -> 110 m/s in under two
seconds is the maximally invalid case for it (the defect NASA's LaSRS++ code
review documents).
"""
import math

import numpy as np

_FT = 0.3048
# Free-atmosphere scale length, 1750 ft.  Above ~2000 ft the standard drops
# the ground-proximity dependence entirely.
L_FREE_AIR_M = 1750.0 * _FT
# Airspeed floor for the filter coefficients.  V appears only as V/L (an
# inverse time constant), and V -> 0 on the pad and at apogee would divide by
# zero.  Dynamic pressure at 5 m/s is ~0.2% of the flight maximum, so the
# gust is aerodynamically irrelevant wherever this floor binds.
V_FLOOR_MPS = 5.0
# Altitude floor, 20 ft — W20's own reference height, below which the
# low-altitude scale lengths (L_w = h) collapse to zero.
H_FLOOR_M = 20.0 * _FT

_INV_SQRT3 = 1.0 / math.sqrt(3.0)

# Turbulence intensity presets, as the 20 ft reference wind W20 (m/s).
# MIL-F-8785B defines light/moderate/severe at 15/30/45 kt.
W20_LIGHT = 15.0 * 0.514444
W20_MODERATE = 30.0 * 0.514444
W20_SEVERE = 45.0 * 0.514444


def dryden_scales(h_agl_m: float, w20_mps: float):
    """Dryden scale lengths and intensities at altitude `h_agl_m`.

    Returns (L_u, L_w, sigma_u, sigma_w) in metres and m/s.

    Below 1000 ft this is the MIL-F-8785B low-altitude form verbatim (h in
    feet)::

        L_w = h                                  sigma_w = 0.1 * W20
        L_u = h / (0.177 + 0.000823 h)**1.2      sigma_u = sigma_w / (...)**0.4

    At exactly 1000 ft the denominator is 1, so both scale lengths meet at
    1000 ft and both intensities meet at 0.1*W20 — the form is naturally
    continuous there.  Above it the standard is undefined until 2000 ft
    (MIL-F-8785C leaves an explicit gap), so the scale length is blended
    linearly from its 1000 ft value to L_FREE_AIR_M across that gap and the
    intensities are held.  A rocket transits this band in a few seconds; a
    discontinuity there would read as a real disturbance rather than as a
    modelling seam.
    """
    h = max(float(h_agl_m), H_FLOOR_M)
    h_ft = h / _FT
    sigma_w = 0.1 * w20_mps

    if h_ft < 1000.0:
        den = 0.177 + 0.000823 * h_ft
        return h / den**1.2, h, sigma_w / den**0.4, sigma_w

    blend = min((h_ft - 1000.0) / 1000.0, 1.0)
    L_1000 = 1000.0 * _FT
    L = L_1000 + blend * (L_FREE_AIR_M - L_1000)
    return L, L, sigma_w, sigma_w


class DrydenGust:
    """Dryden gust generator producing an ENU wind-velocity vector.

    Advance it once per physics step with the current airspeed and altitude;
    the returned vector is meant to be added to the steady wind and passed to
    `SixDOF.derivatives` / `step_rk4` as `wind_enu`.
    """

    def __init__(self, w20_mps: float, seed: int = None,
                 vertical: bool = True,
                 v_floor: float = V_FLOOR_MPS,
                 h_floor: float = H_FLOOR_M):
        if w20_mps <= 0.0:
            raise ValueError("w20_mps must be > 0 (build no gust to disable)")
        self.w20_mps = float(w20_mps)
        self.vertical = bool(vertical)
        self.v_floor = float(v_floor)
        self.h_floor = float(h_floor)
        self._rng = np.random.default_rng(seed)
        # First-order (horizontal) states, and the 2nd-order vertical pair.
        self._e = 0.0
        self._n = 0.0
        self._w_x = 0.0   # intermediate (1st stage of the vertical cascade)
        self._w = 0.0

    def reset(self, h_agl_m: float = 0.0, v_mps: float = 30.0,
              burn_in_s: float = 60.0, burn_in_dt: float = 0.01):
        """Zero the states, then burn in to the stationary distribution.

        Starting from zero would bias the whole of boost: the horizontal time
        constant L_u/V reaches several seconds against a ~10 s climb, and
        early in flight V is small enough that the filter barely leaves the
        origin.  The recursions are exact at any step size and the stationary
        distribution does not depend on it, so the burn-in runs at a coarse
        100 Hz — a few thousand cheap steps, not `physics_dt` ones.
        """
        self._e = self._n = self._w_x = self._w = 0.0
        for _ in range(int(burn_in_s / burn_in_dt)):
            self.step(burn_in_dt, v_mps, h_agl_m)

    def step(self, dt: float, v_air_mps: float, h_agl_m: float) -> np.ndarray:
        """Advance one step; return the gust velocity as an ENU (3,) array."""
        V = max(float(v_air_mps), self.v_floor)
        L_u, L_w, sigma_u, sigma_w = dryden_scales(h_agl_m, self.w20_mps)

        # --- Horizontal: exact Ornstein-Uhlenbeck sampling -------------
        # x[k] = a x[k-1] + sigma sqrt(1 - a^2) eta  is the exact transition
        # of the 1st-order Dryden process, for any dt.
        b = V * dt / L_u
        a = math.exp(-b)
        c = sigma_u * math.sqrt(-math.expm1(-2.0 * b))
        self._e = a * self._e + c * self._rng.standard_normal()
        self._n = a * self._n + c * self._rng.standard_normal()

        # --- Vertical: 2nd-order Dryden as a ZOH stage feeding a lead-lag --
        # Reproduces R_w(t) = sigma_w^2 e^(-Vt/L) (1 - Vt/2L), including the
        # sign reversal past t = 2L/V that distinguishes the true 2nd-order
        # spectrum from a 1st-order one (see tests/test_dryden.py).
        # The vertical filter is advanced unconditionally — `vertical=False`
        # suppresses the OUTPUT only.  Skipping the draw instead would shift
        # every subsequent horizontal sample, so the flag would silently
        # change the horizontal gust too.
        bw = V * dt / L_w
        aw = math.exp(-bw)
        om = -math.expm1(-bw)
        g1 = sigma_w * math.sqrt(3.0 * L_w / (V * dt))
        x_prev = self._w_x
        self._w_x = aw * x_prev + g1 * om * self._rng.standard_normal()
        # Written as C2*(x - x_prev) + r*om*x_prev rather than the literal
        # C2*x + C3*x_prev: the latter has C2 -> +1 and C3 -> -1 while
        # their sum is ~1e-5 at our step sizes, and cancels away most of
        # the mantissa.
        C2 = _INV_SQRT3 + om * (1.0 - _INV_SQRT3) / bw
        self._w = (aw * self._w + C2 * (self._w_x - x_prev)
                   + _INV_SQRT3 * om * x_prev)

        out = np.array([self._e, self._n,
                        self._w if self.vertical else 0.0])
        if not np.all(np.isfinite(out)):
            # A NaN here would propagate into v_air_mag and silently switch
            # off both the aero and the fin-torque gates in derivatives(),
            # producing a force-free ballistic coast instead of an error.
            raise FloatingPointError(
                f"non-finite Dryden gust {out} at V={V}, h={h_agl_m}")
        return out
