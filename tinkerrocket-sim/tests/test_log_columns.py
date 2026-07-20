"""The logged DataFrame must not carry unintended holes.

Log rows are built as dicts and only get a key when the corresponding
`latest_*` value exists, so a field that is still None when the first row is
emitted is simply absent from that dict — and pandas silently backfills NaN
for it when the frame is assembled. The gap is invisible to pandas-side
aggregation (mean/min skip NaN) but poisons anything that goes through
.to_numpy(), which is how the Data_Analysis-style consumers read these frames.

`baro_alt` had exactly this hole in row 0 of every run: the first log row is
emitted on the same tick that initializes the EKF, and the barometer is
sampled in the other branch of that if/elif, so no reading existed yet.
"""
import numpy as np
import pytest

from tinkerrocket_sim.simulation.closed_loop_sim import SimConfig, run_closed_loop
from tinkerrocket_sim.simulation.scenarios import build_rollypolly_iii

# roll_target_deg / current_roll_deg are legitimately absent before the roll
# controller engages (explicitly set back to None while rate-nulling), so
# their NaN region is meaningful "not applicable", not an initialization hole.
INTENTIONALLY_SPARSE = {"roll_target_deg", "current_roll_deg"}


def _flight(**overrides):
    cfg = dict(pad_time=0.0, duration=3.0, physics_dt=1e-3,
               control_enabled=False, guidance_enabled=False,
               enable_mag_updates=False, pad_heading_deg=0.0, sensor_seed=0)
    cfg.update(overrides)
    return run_closed_loop(build_rollypolly_iii(), SimConfig(**cfg)).df


def _holes(df):
    num = df.select_dtypes("number")
    out = {}
    for col in num.columns:
        if col in INTENTIONALLY_SPARSE:
            continue
        n = int((~np.isfinite(num[col].to_numpy(dtype=float))).sum())
        if n:
            out[col] = n
    return out


@pytest.mark.parametrize("overrides", [
    {},
    {"pad_time": 2.0},
    {"enable_baro_updates": False},
    {"ref_alt_m": 500.0},
    {"control_enabled": True},
])
def test_logged_frame_has_no_non_finite_cells(overrides):
    assert _holes(_flight(**overrides)) == {}


def test_baro_alt_is_populated_from_the_very_first_row():
    """Row 0 used to be NaN in every single run."""
    df = _flight()
    assert np.isfinite(df["baro_alt"].iloc[0])


def test_baro_alt_seed_matches_the_starting_altitude():
    """The pre-first-sample seed is the noiseless ISA inversion, so it must sit
    at the launch altitude and agree with the samples that follow it."""
    df = _flight(ref_alt_m=500.0)
    assert df["baro_alt"].iloc[0] == pytest.approx(500.0, abs=1e-6)
    # ...and the real samples scatter tightly around it, i.e. the seed is on
    # the same scale as the sensor rather than an arbitrary placeholder.
    assert df["baro_alt"].iloc[1:20].mean() == pytest.approx(500.0, abs=0.5)
