"""The apogee detector's baro rate gate needs a 5 m step as well as 300 m/s.

At the BMP585's 2 ms spacing the rate alone trips on a 0.6 m step, and a
vented bay's static port passes the airflow's pressure noise at speed: on
BARC 2026-10-24 it dropped 30 to 50% of the 65 mm roll-control flight's coast
samples and up to 78% of Rolly Polly V's main bay's, all of it real pressure.

This test covers the Python mirror in `sim_kinematic_checks.py`. The firmware
is covered by `tests_cpp/test_kinematic_checks.cpp::BaroGate_*`.
"""
from __future__ import annotations

import math
from pathlib import Path
import sys

_SIM_DIR = Path(__file__).resolve().parent.parent.parent / "Data_Analysis"
sys.path.insert(0, str(_SIM_DIR))

from sim_kinematic_checks import KinematicChecks  # noqa: E402


def _drive(kc, now_ms, alt):
    kc.kinematic_checks(
        pressure_altitude=alt, acc_mag=9.80665, position=(0.0, 0.0, alt),
        velocity=(0.0, 0.0, 0.0), roll_rate=0.0, new_baro=True,
        gps_altitude=0.0, new_gps=False, pitch_rad=math.pi / 2,
        burnout_detected=False, baro_locked_out=False, now_ms=now_ms,
    )


def test_static_port_noise_inside_the_step_floor_is_kept():
    # ±2 m about a level 100 m: the rate alone dropped every sample on the
    # far side of the first one it accepted, and the filter sat on +2 m.
    kc = KinematicChecks()
    for i in range(2000):
        _drive(kc, i * 2, 100.0 + (-2.0 if i & 1 else 2.0))
    assert kc._consec_baro_rejects == 0
    assert abs(kc.alt_est - 100.0) < 0.5, kc.alt_est


def test_a_spike_past_the_step_floor_is_still_dropped():
    kc = KinematicChecks()
    for i in range(1000):
        _drive(kc, i * 2, 100.0)
    before = kc.alt_est
    _drive(kc, 2000, 106.0)
    assert kc._consec_baro_rejects == 1
    assert abs(kc.alt_est - before) < 0.05
    # A real 4 m step inside the floor is taken at once.
    _drive(kc, 2002, 104.0)
    assert kc._consec_baro_rejects == 0
    assert kc.alt_est > before + 0.05
