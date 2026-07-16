#!/usr/bin/env python3
"""CLI for the roll-controller tuning sweep (#170).

Sweeps roll-control gains through the boost-roll disturbance scenario and
prints per-config metrics (RMS rate error, peak rate, settling time,
saturation %). Each gain axis takes a comma-separated list; the Cartesian
product is the grid.

Examples:
  # default grid around the flown tune
  python scripts/tuning_sweep.py

  # sweep Kp and Ki, hold the rest, save a CSV
  python scripts/tuning_sweep.py --kp 0.01,0.02,0.04 --ki 0,0.01,0.03 --csv sweep.csv

  # serial (easier to profile / debug)
  python scripts/tuning_sweep.py --kp 0.02 --serial
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'src'))

from tinkerrocket_sim.simulation import tuning_sweep as TS  # noqa: E402


def _floats(s):
    return [float(x) for x in s.split(',') if x.strip() != '']


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--kp', type=_floats, help='comma list of Kp values')
    p.add_argument('--ki', type=_floats, help='comma list of Ki values')
    p.add_argument('--kd', type=_floats, help='comma list of Kd values')
    p.add_argument('--kp-angle', type=_floats, help='comma list of KP_ANGLE values')
    p.add_argument('--rate-cap', type=_floats, help='comma list of rate-cap (deg/s)')
    p.add_argument('--sort', default='rms_rate_error_dps',
                   help='metric column to sort by (default: rms_rate_error_dps)')
    p.add_argument('--csv', help='also write the results to this CSV path')
    p.add_argument('--serial', action='store_true', help='run one at a time')
    p.add_argument('--max-workers', type=int, default=None)
    args = p.parse_args(argv)

    axes = {}
    for name in ('kp', 'ki', 'kd'):
        if getattr(args, name) is not None:
            axes[name] = getattr(args, name)
    if args.kp_angle is not None:
        axes['kp_angle'] = args.kp_angle
    if args.rate_cap is not None:
        axes['rate_cap'] = args.rate_cap

    grid = TS.build_grid(**axes) if axes else TS.default_grid()
    print(f"Sweeping {len(grid)} config(s) "
          f"({'serial' if args.serial else 'parallel'})...", file=sys.stderr)

    rows = TS.run_sweep(grid, parallel=not args.serial, max_workers=args.max_workers)
    print(TS.format_table(rows, sort_by=args.sort))

    if args.csv:
        TS.write_csv(rows, args.csv)
        print(f"\nWrote {args.csv}", file=sys.stderr)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
