"""The vibration section (#1192).

The signal helpers are checked on synthetic series where the answer is
known — a pure tone, Gaussian noise, a train of impacts, a jittered and
gapped sample clock, a railed sensor behind the 45° mounting rotation — and
then the section is run on the two flights in the repo.
"""

from __future__ import annotations

import math
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
from flight_report.modules import vibration as vib  # noqa: E402

FS = 3840.0


def _tone(f0: float, amp: float, seconds: float = 2.0, fs: float = FS) -> np.ndarray:
    t = np.arange(int(seconds * fs)) / fs
    return amp * np.sin(2 * np.pi * f0 * t)


# ---- the sample clock -------------------------------------------------------

def test_uniform_removes_jitter_and_fills_a_dropped_slot() -> None:
    rng = np.random.default_rng(0)
    n = 2000
    t = np.arange(n) / FS + rng.uniform(-60e-6, 60e-6, n)   # ±60 µs read jitter
    x = np.arange(n, dtype=float)
    keep = np.ones(n, bool)
    keep[1000] = False                                        # one dropped sample
    u = vib.uniform(t[keep], {"ax": x[keep]})
    assert u is not None
    assert u["fs"] == pytest.approx(FS, rel=0.01)
    assert u["filled"] == 1
    assert u["t"].size == n
    assert u["chans"]["ax"][1000] == pytest.approx(1000.0, abs=0.01)


def test_uniform_refuses_a_stream_that_is_not_sampled() -> None:
    t = np.array([0.0, 1.0, 5.0, 5.001, 9.0, 20.0] * 4)
    assert vib.uniform(np.sort(t), {"ax": np.zeros(t.size)}) is None


# ---- spectra ----------------------------------------------------------------

def test_band_rms_and_dominant_tone_of_a_pure_tone() -> None:
    x = _tone(800.0, 30.0)
    f, p = vib.welch(x, FS)
    assert vib.band_rms(f, p, 500, 1000) == pytest.approx(30.0 / math.sqrt(2), rel=0.03)
    assert vib.band_rms(f, p, 2, 50) < 0.5
    tone = vib.dominant_tone(f, p)
    assert tone is not None
    assert tone[0] == pytest.approx(800.0, abs=5.0)
    assert tone[1] > 0.95


def test_highpass_removes_the_slow_part() -> None:
    x = _tone(10.0, 100.0) + _tone(800.0, 5.0)
    hp = vib.highpass(x, FS, 100.0)
    assert np.std(hp) == pytest.approx(5.0 / math.sqrt(2), rel=0.05)


# ---- shaking vs knocking ----------------------------------------------------

def test_kurtosis_separates_oscillation_from_impacts() -> None:
    rng = np.random.default_rng(1)
    gaussian = rng.normal(0, 1, 20000)
    assert abs(vib.kurtosis(gaussian)) < 0.3
    impacts = rng.normal(0, 0.05, 20000)
    impacts[::500] = 3.0                                     # sharp knocks
    assert vib.kurtosis(impacts) > vib._IMPACT_KURTOSIS


def test_envelope_tracks_the_amplitude() -> None:
    x = np.concatenate([_tone(800.0, 10.0, 1.0), _tone(800.0, 40.0, 1.0)])
    idx, rms = vib.envelope(x, FS)
    t = idx / FS
    assert np.median(rms[(t > 0.2) & (t < 0.8)]) == pytest.approx(10.0 / math.sqrt(2), rel=0.05)
    assert np.median(rms[(t > 1.2) & (t < 1.8)]) == pytest.approx(40.0 / math.sqrt(2), rel=0.05)


# ---- rails behind the mounting rotation ------------------------------------

def test_rail_fraction_sees_through_the_45_degree_rotation() -> None:
    fs = 16.0 * vib.G
    # The sensor's X axis pinned at its rail, Y quiet: in body axes (−45° rotation)
    # that reads as fs/√2 on both X and Y — nowhere near a body-axis rail test.
    sx = np.array([fs, fs, 0.0, 0.0])
    sy = np.zeros(4)
    th = math.radians(-45.0)
    bx = math.cos(th) * sx - math.sin(th) * sy
    by = math.sin(th) * sx + math.cos(th) * sy
    frac, worst = vib.rail_fraction(bx, by, np.zeros(4), fs, -45.0, 0.99)
    assert frac == pytest.approx(0.5)
    assert worst == "X"


def test_to_sensor_axes_inverts_the_converter() -> None:
    sx, sy = np.array([1.0, 0.0]), np.array([0.0, 1.0])
    th = math.radians(-45.0)
    bx = math.cos(th) * sx - math.sin(th) * sy
    by = math.sin(th) * sx + math.cos(th) * sy
    rx, ry = vib.to_sensor_axes(bx, by, -45.0)
    assert rx == pytest.approx(sx) and ry == pytest.approx(sy)


# ---- the section on real flights -------------------------------------------

@pytest.mark.skipif(not SAMPLE_BIN.exists(), reason="sample flight missing")
def test_sample_flight_has_every_phase_and_both_charts() -> None:
    result = vib.analyze(Flight.from_bin(SAMPLE_BIN))
    assert result.error is None
    keys = list(result.metrics)
    for phase in ("Boost", "Coast", "Descent"):
        assert any(k.startswith(phase) for k in keys), phase
    # The sample flight lit 0.32 s into its log: too little pad to measure.
    assert not any(k.startswith("Pad") for k in keys)
    assert not result.warnings, result.warnings
    assert "Boost · gyro at full scale" in result.metrics
    assert "Boost · ±16 g accelerometer at its rail" in result.metrics
    assert [c["id"] for c in result.charts] == ["chart-vibration-envelope", "chart-vibration-spectrum"]
    assert all(c["layout"]["yaxis"]["type"] == "log" for c in result.charts)


@pytest.mark.skipif(not GOLDEN_BIN.exists(), reason="golden flight missing")
def test_golden_capture_opening_mid_boost_still_measures_boost() -> None:
    result = vib.analyze(Flight.from_bin(GOLDEN_BIN))
    assert result.error is None
    assert any(k.startswith("Boost") for k in result.metrics)
    assert result.charts
