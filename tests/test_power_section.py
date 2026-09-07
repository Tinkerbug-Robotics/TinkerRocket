"""The flight-pack section (#1193).

The detectors are exercised on synthetic series where the answer is known,
then the section is run on the two flights checked into the repo: the sample
flight, whose 1.5 A current spread carries a source-impedance fit, and the
golden capture, a 5 V bench run whose voltage-derived SOC clamps at zero and
must not be reported as a state of charge.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
SAMPLE_BIN = REPO_ROOT / "examples" / "flights" / "flight_20260705_174532.bin"
GOLDEN_BIN = REPO_ROOT / "tests" / "test_data" / "flight_20260615_170318.bin"

if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))

from flight_report.flight import Flight  # noqa: E402
from flight_report.modules import power  # noqa: E402
from flight_report.units import Quantity  # noqa: E402


def _steady(n: int = 200, fs: float = 100.0, v: float = 7.8, i: float = 1.2):
    t = np.arange(n) / fs
    return t, np.full(n, v), np.full(n, i)


# ---- contact chatter --------------------------------------------------------

def test_chatter_flags_a_lone_drop_with_no_load_change() -> None:
    t, v, i = _steady()
    v[100] = 7.61                       # the 2026-08-29 V8 sample, near enough
    hits = power.find_chatter(t, v, i)
    assert len(hits) == 1
    assert hits[0]["t"] == pytest.approx(1.0)
    assert hits[0]["v"] == pytest.approx(7.61)
    assert hits[0]["around"] == pytest.approx(7.8)


def test_chatter_ignores_a_drop_the_load_explains() -> None:
    t, v, i = _steady()
    v[100] = 7.61
    i[100] = 1.2 + 0.8                  # a rail switched on for that sample
    assert power.find_chatter(t, v, i) == []


def test_chatter_flags_a_drop_where_the_current_fell_with_it() -> None:
    # The 2026-08-29 V8 sample: 7.61 V between 7.8 V neighbours with the draw
    # 0.37 A LOWER — a joint going high-resistance starves both.
    t, v, i = _steady()
    v[100] = 7.61
    i[100] = 1.2 - 0.37
    hits = power.find_chatter(t, v, i)
    assert len(hits) == 1 and hits[0]["di"] == pytest.approx(-0.37)


def test_chatter_ignores_a_real_step() -> None:
    t, v, i = _steady()
    v[100:] = 7.61                      # the pack settled lower and stayed there
    assert power.find_chatter(t, v, i) == []


def test_chatter_respects_the_launch_boundary() -> None:
    t, v, i = _steady()
    v[20] = 7.5                         # plugging in on the pad
    v[150] = 7.5                        # in flight
    hits = power.find_chatter(t, v, i, after_s=1.0)
    assert [h["t"] for h in hits] == [pytest.approx(1.5)]


# ---- source impedance -------------------------------------------------------

def test_impedance_fit_recovers_the_slope() -> None:
    rng = np.random.default_rng(1)
    i = np.linspace(0.2, 1.8, 400)
    v = 8.2 - 0.150 * i + rng.normal(0, 0.003, i.size)
    fit = power.fit_impedance(v, i)
    assert fit is not None
    assert fit["r_mohm"] == pytest.approx(150.0, abs=5.0)
    assert fit["v0"] == pytest.approx(8.2, abs=0.01)


def test_impedance_needs_a_current_spread() -> None:
    i = np.linspace(0.11, 0.20, 400)    # the 2026-08-29 nose computer's whole flight
    v = 8.1 - 0.15 * i
    assert power.fit_impedance(v, i) is None


def test_impedance_reports_how_much_it_explains() -> None:
    rng = np.random.default_rng(2)
    i = rng.uniform(1.1, 2.3, 37)       # the V8 fragment: spread but no relation
    v = 7.8 + rng.normal(0, 0.03, i.size)
    fit = power.fit_impedance(v, i)
    assert fit is not None and fit["r2"] < 0.3


# ---- energy, pyro dips, the end of the log ----------------------------------

def test_energy_used_in_joules_and_milliamp_hours() -> None:
    t = np.linspace(0.0, 3600.0, 3601)  # one hour at 1 Hz
    v = np.full(t.size, 8.0)
    i = np.full(t.size, 0.5)
    joules, mah = power.energy_used(t, v, i)
    assert joules == pytest.approx(8.0 * 0.5 * 3600.0, rel=1e-6)
    assert mah == pytest.approx(500.0, rel=1e-6)


def test_pyro_dip_measures_against_the_baseline_before_the_fire() -> None:
    t, v, _ = _steady(n=400)
    v[205:208] = 7.55                   # a 250 mV dip right after the fire at t=2.0
    d = power.pyro_dip(t, v, 2.0)
    assert d is not None
    assert d["dip"] == pytest.approx(0.25)
    assert d["baseline"] == pytest.approx(7.8)


def test_pyro_dip_is_none_outside_the_record() -> None:
    t, v, _ = _steady()
    assert power.pyro_dip(t, v, 50.0) is None


def _state_records(states: list[int], launch_at: int | None, landed_at: int | None) -> dict:
    ns = []
    for k, st in enumerate(states):
        ns.append({"time_us": 1_000_000 + k * 2_000, "rocket_state": st,
                   "launch": launch_at is not None and k >= launch_at,
                   "alt_landed": landed_at is not None and k >= landed_at})
    return {"NonSensor": ns}


def test_log_end_is_clean_when_the_flight_landed() -> None:
    recs = _state_records([2] * 10 + [3] * 50 + [4] * 10, launch_at=10, landed_at=60)
    t, v, i = _steady(n=70, fs=500.0)
    end = power.log_end(recs, 1_000_000, t, v, i)
    assert end["clean"] is True
    assert "landed" in end["text"]


def test_log_end_flags_a_record_cut_in_flight() -> None:
    recs = _state_records([2] * 10 + [3] * 50, launch_at=10, landed_at=None)
    t, v, i = _steady(n=60, fs=500.0)
    end = power.log_end(recs, 1_000_000, t, v, i)
    assert end["clean"] is False
    assert "INFLIGHT" in end["text"]
    assert "no sag" in end["text"]
    assert "7.80 V" in end["text"]


def test_log_end_reports_a_sag_into_the_cut() -> None:
    recs = _state_records([2] * 10 + [3] * 50, launch_at=10, landed_at=None)
    t, v, i = _steady(n=60, fs=500.0)
    v[-1] = 7.5                         # collapsing, not severed
    end = power.log_end(recs, 1_000_000, t, v, i)
    assert end["clean"] is False
    assert "sagging 300 mV" in end["text"]


def test_log_end_is_indifferent_to_a_bench_run() -> None:
    recs = _state_records([1] * 60, launch_at=None, landed_at=None)
    t, v, i = _steady(n=60, fs=500.0)
    assert power.log_end(recs, 1_000_000, t, v, i)["clean"] is None


# ---- the section on real flights -------------------------------------------

@pytest.mark.skipif(not SAMPLE_BIN.exists(), reason="sample flight missing")
def test_sample_flight_carries_a_fit_and_a_landing() -> None:
    result = power.analyze(Flight.from_bin(SAMPLE_BIN))
    assert result.error is None
    assert not result.warnings, result.warnings
    m = result.metrics
    assert isinstance(m["Source impedance"], Quantity), m["Source impedance"]
    assert m["Source impedance"].unit == "mΩ"
    assert m["Single-sample drops in flight"] == "none"
    assert "landed" in m["Log end"]
    assert m["Energy used"].unit == "mAh" and m["Energy used"].value > 0
    assert [c["id"] for c in result.charts] == ["chart-power"]


@pytest.mark.skipif(not GOLDEN_BIN.exists(), reason="golden flight missing")
def test_bench_capture_does_not_report_a_state_of_charge() -> None:
    result = power.analyze(Flight.from_bin(GOLDEN_BIN))
    assert result.error is None
    assert "State of charge" not in result.metrics
    assert "Pack minimum" in result.metrics
    assert result.charts, "the pack chart should draw on any log with pack telemetry"
