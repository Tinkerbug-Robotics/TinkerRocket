"""Control-performance metrics for closed-loop sim runs.

Shared by the canonical scenario tests (pass/fail thresholds) and the tuning
sweep harness (#170).  The primitives take plain arrays so they are trivially
unit-testable; the `summarize_*` helpers pull the right columns out of a
`run_closed_loop` result DataFrame.

Angles are handled with shortest-arc wrapping so a target crossing ±180° does
not register a spurious 360° error.
"""
import numpy as np


def _wrap180(deg):
    """Wrap angle(s) in degrees to (-180, 180]."""
    return (np.asarray(deg, dtype=float) + 180.0) % 360.0 - 180.0


def rms(error) -> float:
    """Root-mean-square of an error series.  NaN for an empty series."""
    e = np.asarray(error, dtype=float)
    e = e[np.isfinite(e)]
    if e.size == 0:
        return float('nan')
    return float(np.sqrt(np.mean(e * e)))


def rms_tracking_error(actual, target, angular: bool = False) -> float:
    """RMS of (actual - target).

    angular=True treats the signals as degrees and wraps the error to the
    shortest arc, so tracking a setpoint across the ±180° seam is not penalized.
    """
    a = np.asarray(actual, dtype=float)
    t = np.broadcast_to(np.asarray(target, dtype=float), a.shape)
    err = a - t
    if angular:
        err = _wrap180(err)
    return rms(err)


def peak_abs(series) -> float:
    """Peak absolute value of a series (e.g. peak body rate, deg/s)."""
    s = np.asarray(series, dtype=float)
    s = s[np.isfinite(s)]
    if s.size == 0:
        return float('nan')
    return float(np.max(np.abs(s)))


def saturation_pct(cmd, lo: float, hi: float, tol: float = 1e-3) -> float:
    """Percent of samples at (within tol of) either actuator limit.

    A command sitting on the mechanical stop is "saturated"; this reports how
    much of the run the actuator spent pinned, a proxy for authority exhaustion.
    """
    c = np.asarray(cmd, dtype=float)
    c = c[np.isfinite(c)]
    if c.size == 0:
        return float('nan')
    at_stop = (c >= hi - tol) | (c <= lo + tol)
    return 100.0 * float(np.count_nonzero(at_stop)) / c.size


def settling_time(time, signal, target=0.0, tol=None, tol_frac=0.05,
                  start_time=None):
    """Time (s) after `start_time` for `signal` to enter and stay within a band
    around `target`.

    The band is ±`tol` if given, else ±`tol_frac` of the initial deviation
    |signal(start) - target| (a classic settling-time definition).  Returns the
    elapsed time from `start_time` to the last moment the signal leaves the
    band; None if it never settles (still outside the band at the end) or the
    window is empty.
    """
    t = np.asarray(time, dtype=float)
    x = np.asarray(signal, dtype=float)
    if t.size == 0 or t.size != x.size:
        return None

    if start_time is None:
        start_time = t[0]
    mask = t >= start_time
    if not np.any(mask):
        return None
    t = t[mask]
    x = x[mask]

    dev0 = abs(float(x[0]) - target)
    band = tol if tol is not None else max(tol_frac * dev0, 1e-9)

    outside = np.abs(x - target) > band
    if not np.any(outside):
        return 0.0
    if outside[-1]:
        return None  # never settles within the window
    last_out = int(np.max(np.nonzero(outside)[0]))
    # Settled at the first sample after the last excursion.
    return float(t[last_out + 1] - t[0])


# --- DataFrame convenience wrappers -------------------------------------------

def summarize_roll(df, deflection_min=-20.0, deflection_max=20.0,
                   settle_tol_dps=5.0, settle_start_time=None):
    """Roll-control metrics from a run_closed_loop result DataFrame.

    Uses `current_roll_deg` vs `roll_target_deg` for angle tracking when a
    profile is present, `roll_rate_dps` for the rate/peak, and `fin_tab_cmd`
    for saturation.  Missing columns yield NaN entries rather than raising.
    """
    out = {}
    rate = df['roll_rate_dps'].to_numpy() if 'roll_rate_dps' in df else np.array([])
    out['peak_roll_rate_dps'] = peak_abs(rate)
    out['final_roll_rate_dps'] = float(rate[-1]) if rate.size else float('nan')

    if 'roll_target_deg' in df and 'current_roll_deg' in df:
        actual = df['current_roll_deg'].to_numpy()
        target = df['roll_target_deg'].to_numpy()
        out['rms_angle_error_deg'] = rms_tracking_error(actual, target, angular=True)
    else:
        out['rms_angle_error_deg'] = float('nan')

    if 'fin_tab_cmd' in df:
        out['fin_saturation_pct'] = saturation_pct(
            df['fin_tab_cmd'].to_numpy(), deflection_min, deflection_max)
        out['peak_fin_deg'] = peak_abs(df['fin_tab_cmd'].to_numpy())
    else:
        out['fin_saturation_pct'] = float('nan')
        out['peak_fin_deg'] = float('nan')

    if 'time' in df and rate.size:
        out['roll_rate_settle_s'] = settling_time(
            df['time'].to_numpy(), rate, target=0.0, tol=settle_tol_dps,
            start_time=settle_start_time)
    else:
        out['roll_rate_settle_s'] = None
    return out


def summarize_guidance(df):
    """Guidance metrics from a run_closed_loop result DataFrame.

    Reports the minimum horizontal offset from the pad axis achieved during the
    guided window (a proxy for closest approach to an overhead aim point) and
    the final offset.  Horizontal miss is sqrt(x^2 + y^2) in the ENU ground
    plane (the PN target is straight up over the pad).
    """
    out = {}
    if 'x' in df and 'y' in df:
        horiz = np.hypot(df['x'].to_numpy(), df['y'].to_numpy())
    else:
        horiz = np.array([])
    if 'guidance_active' in df:
        guided = df[df['guidance_active'] == True]
    else:
        guided = df.iloc[0:0]

    if len(guided):
        gh = np.hypot(guided['x'].to_numpy(), guided['y'].to_numpy())
        out['min_horiz_offset_m'] = float(np.min(gh))
        out['final_horiz_offset_m'] = float(gh[-1])
    else:
        out['min_horiz_offset_m'] = float('nan')
        out['final_horiz_offset_m'] = float('nan')
    out['max_horiz_offset_m'] = float(np.max(horiz)) if horiz.size else float('nan')

    if 'guid_lateral_offset_m' in df and len(guided):
        lat = guided['guid_lateral_offset_m'].to_numpy()
        out['final_lateral_offset_m'] = float(lat[-1]) if lat.size else float('nan')
    else:
        out['final_lateral_offset_m'] = float('nan')
    return out
