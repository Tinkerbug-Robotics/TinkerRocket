"""Smoke test for the roll-controller tuning sweep harness (#170)."""
import numpy as np

from tinkerrocket_sim.simulation import tuning_sweep as TS


def test_build_grid_cartesian_product():
    grid = TS.build_grid(kp=[0.01, 0.02], ki=[0.0, 0.03])
    assert len(grid) == 4
    assert {'kp', 'ki'} == set(grid[0])
    assert {(g['kp'], g['ki']) for g in grid} == {
        (0.01, 0.0), (0.01, 0.03), (0.02, 0.0), (0.02, 0.03)}


def test_evaluate_produces_metric_row():
    row = TS.evaluate({'kp': 0.02, 'ki': 0.03})
    assert row['ok'] is True
    for col in ('rms_rate_error_dps', 'peak_rate_dps', 'saturation_pct',
                'peak_fin_deg', 'coast_resid_dps', 'apogee_m'):
        assert col in row and np.isfinite(row[col])
    # The disturbance scenario should fly and the controller should keep the
    # rate bounded and largely unsaturated at the flown gains.
    assert 300.0 < row['apogee_m'] < 420.0
    assert row['peak_rate_dps'] < 150.0
    assert 0.0 <= row['saturation_pct'] <= 100.0


def test_run_sweep_serial_two_configs():
    grid = [{'kp': 0.02, 'ki': 0.03}, {'kp': 0.04, 'ki': 0.03}]
    rows = TS.run_sweep(grid, parallel=False)
    assert len(rows) == 2
    assert all(r['ok'] for r in rows)
    table = TS.format_table(rows)
    assert 'rms_rate_error_dps' in table
