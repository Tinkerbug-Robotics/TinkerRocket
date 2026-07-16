"""Roll-controller tuning sweep harness (#170).

Varies the roll-control gains (Kp, Ki, Kd, KP_ANGLE, rate cap) over a grid,
runs each through the boost-roll disturbance scenario with the real firmware
controller, and emits per-config performance metrics so a tune can be chosen
at a keyboard instead of on the rail:

  - rms_rate_error_dps : RMS body roll rate over the controlled window (the
        disturbance-rejection tracking error; the objective is rate ≈ 0)
  - peak_rate_dps      : peak |roll rate| over the window
  - settle_s           : time to null the kick to within 5 dps
  - saturation_pct     : % of the run the fin sat on a mechanical stop
  - peak_fin_deg, coast_resid_dps : actuator effort / steady residual

Roll metrics use the body roll RATE, which is free of the body-Z azimuth
artifact that corrupts the angle error near apogee.

Runs are seeded (deterministic).  `run_sweep(..., parallel=True)` fans the grid
across processes; `evaluate` is module-level so it pickles for the pool.
"""
import dataclasses
import itertools
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from .closed_loop_sim import run_closed_loop
from . import scenarios as S
from . import metrics as M

# The gain axes the harness knows how to sweep -> the SimConfig field they map
# to.  Order defines the column order in the emitted table.
GAIN_FIELDS = {
    'kp': 'pid_kp',
    'ki': 'pid_ki',
    'kd': 'pid_kd',
    'kp_angle': 'kp_angle',
    'rate_cap': 'rate_cap_dps',
}

# Analysis window: from control activation to just before the apogee pitch-over
# (which sweeps the azimuth but leaves body roll rate well-behaved).
_WINDOW_END_S = 7.5


def default_grid():
    """A small, meaningful default grid around the flown roll-null tune."""
    return build_grid(
        kp=[0.01, 0.02, 0.04],
        ki=[0.0, 0.03],
        kd=[0.0],
        kp_angle=[2.0],
        rate_cap=[60.0],
    )


def build_grid(**axes):
    """Cartesian product of the given gain axes -> list of gain dicts.

    Each axis is a list of values; omitted axes are left at the scenario
    default.  e.g. build_grid(kp=[0.01, 0.02], ki=[0.0, 0.03]) -> 4 configs.
    """
    names = [n for n in GAIN_FIELDS if n in axes]
    value_lists = [list(axes[n]) for n in names]
    grid = []
    for combo in itertools.product(*value_lists):
        grid.append(dict(zip(names, combo)))
    return grid


def _config_for(gains):
    """Boost-roll disturbance scenario with the swept gains applied."""
    cfg = S.roll_boost_disturbance_config()
    overrides = {GAIN_FIELDS[k]: v for k, v in gains.items() if k in GAIN_FIELDS}
    return dataclasses.replace(cfg, **overrides)


def evaluate(gains):
    """Run one gain set through the disturbance scenario and return its metrics.

    Module-level so ProcessPoolExecutor can pickle it.  Returns a flat dict of
    the swept gains plus the metric columns (NaN/None on a failed run rather
    than raising, so one bad config doesn't sink the whole sweep).
    """
    row = dict(gains)
    try:
        cfg = _config_for(gains)
        df = run_closed_loop(S.build_rollypolly_iii(), cfg).df
        apogee_idx = int(df['altitude'].idxmax())
        df = df.iloc[:apogee_idx + 1].reset_index(drop=True)

        t = df['time'].to_numpy()
        rate = df['roll_rate_dps'].to_numpy()
        win = (t >= cfg.roll_delay_s) & (t <= _WINDOW_END_S)

        row['rms_rate_error_dps'] = M.rms(rate[win])          # objective: rate→0
        row['peak_rate_dps'] = M.peak_abs(rate[win])
        row['settle_s'] = M.settling_time(
            t, rate, target=0.0, tol=5.0,
            start_time=cfg.roll_kick_time_s + 0.05)
        row['saturation_pct'] = M.saturation_pct(
            df['fin_tab_cmd'].to_numpy(), cfg.deflection_min, cfg.deflection_max)
        row['peak_fin_deg'] = M.peak_abs(df['fin_tab_cmd'].to_numpy())
        coast = df[(df['time'] > 4.0) & (df['time'] < 7.0)]
        row['coast_resid_dps'] = float(coast['roll_rate_dps'].abs().median()) \
            if len(coast) else float('nan')
        row['apogee_m'] = float(df['altitude'].max())
        row['ok'] = True
    except Exception as exc:  # keep the sweep going; surface the failure inline
        row['error'] = repr(exc)
        row['ok'] = False
    return row


def run_sweep(grid, parallel=True, max_workers=None):
    """Evaluate every gain dict in `grid`; return a list of metric rows."""
    if not parallel or len(grid) == 1:
        return [evaluate(g) for g in grid]
    with ProcessPoolExecutor(max_workers=max_workers) as pool:
        return list(pool.map(evaluate, grid))


_COLUMNS = (list(GAIN_FIELDS)
            + ['rms_rate_error_dps', 'peak_rate_dps', 'settle_s',
               'saturation_pct', 'peak_fin_deg', 'coast_resid_dps', 'apogee_m'])


def _fmt(v):
    if v is None:
        return '  --  '
    if isinstance(v, float):
        return f'{v:7.3f}' if abs(v) < 100 else f'{v:7.1f}'
    return f'{v}'


def format_table(rows, sort_by='rms_rate_error_dps'):
    """Render sweep rows as a fixed-width table, best (lowest sort_by) first."""
    def key(r):
        v = r.get(sort_by)
        return v if isinstance(v, (int, float)) and v == v else float('inf')
    rows = sorted(rows, key=key)

    cols = [c for c in _COLUMNS if any(c in r for r in rows)]
    header = ' | '.join(f'{c:>16s}' for c in cols)
    lines = [header, '-' * len(header)]
    for r in rows:
        if not r.get('ok', True):
            gaincols = ' | '.join(f'{_fmt(r.get(c)):>16s}' for c in list(GAIN_FIELDS))
            lines.append(f'{gaincols}   FAILED: {r.get("error", "?")}')
            continue
        lines.append(' | '.join(f'{_fmt(r.get(c)):>16s}' for c in cols))
    return '\n'.join(lines)


def write_csv(rows, path):
    """Write the sweep rows to a CSV at `path`."""
    import csv
    cols = [c for c in _COLUMNS if any(c in r for r in rows)]
    if any(not r.get('ok', True) for r in rows):
        cols = cols + ['error']
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow(r)
