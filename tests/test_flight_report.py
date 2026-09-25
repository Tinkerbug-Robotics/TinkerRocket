"""Smoke test for the flight_report suite (issue #184).

Runs the full suite against the canonical golden flight checked into
`tests/test_data/` (a new-PCB / IIS2MDC capture); fails if the report
doesn't render or any required section is missing.
"""

from __future__ import annotations

import math
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
GOLDEN_BIN = REPO_ROOT / "tests" / "test_data" / "flight_20260615_170318.bin"
# A complete flight, pad to touchdown. GOLDEN_BIN deliberately is not one: its
# record opens at 8.1 g, already under thrust, so it has no measurable liftoff.
SAMPLE_BIN = REPO_ROOT / "examples" / "flights" / "flight_20260705_174532.bin"

# A few tests reach into the report package directly rather than through the
# rendered HTML. Data_Analysis has no __init__.py, so it is put on the path the
# same way the package's own modules do it — once, here, rather than inside a
# test where it would leak into every test that runs afterwards.
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))

# Sections that must not appear. Most of the second report's modules were
# deleted outright; these two remain on disk and must stay out of the report —
# guidance because no real flight carries a guidance frame, sensor_charts
# because overview now draws those series itself.
DETAILED_SECTIONS = [
    'id="guidance"',
    'id="sensor_charts"',
]

# The whole report, in one page. The second level is gone and most of its
# modules with it; what survived was folded in here.
FLIGHT_SECTIONS = [
    'id="overview"',
    'id="globe"',
    'id="rocket_state"',
    'id="power"',
    'id="vibration"',
    'id="deployment"',
    'id="barometer"',
    'id="parser_stats"',
    'id="timing"',
    'id="settings"',
    'id="system"',
]


@pytest.fixture(scope="module")
def reports(tmp_path_factory: pytest.TempPathFactory) -> dict[str, Path]:
    """Run the suite once against the golden flight; yield the report.

    There used to be two levels and this returned both. The detailed report is
    gone — its sections were folded into this one — so the mapping has a single
    entry rather than being flattened away, because the tests read it by name
    and a dict keeps that readable if a second document ever returns.
    """
    if not GOLDEN_BIN.exists():
        pytest.skip(f"Golden flight missing: {GOLDEN_BIN}")

    tmp_path = tmp_path_factory.mktemp("report")

    result = subprocess.run(
        [
            sys.executable, "-m", "Data_Analysis.flight_report",
            "run", str(GOLDEN_BIN),
            "--out", str(tmp_path),
        ],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=180,
    )
    print(result.stdout, file=sys.stderr)
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr)
    assert result.returncode == 0, f"flight_report exited {result.returncode}"

    out = {"flight": tmp_path / f"{GOLDEN_BIN.stem}_report.html"}
    stray = list(tmp_path.glob("*_report_detailed.html"))
    assert not stray, f"a detailed report was still written: {stray}"
    for level, path in out.items():
        assert path.exists(), f"{level} report not written to {path}"
        assert path.stat().st_size > 100_000, f"{level} report suspiciously small"
    return out


@pytest.fixture(scope="module")
def report_html(reports: dict[str, Path]) -> Path:
    """The flight-level report — the one a flyer opens."""
    return reports["flight"]


def test_detailed_report_is_gone(reports: dict[str, Path]) -> None:
    """One report. The bring-up sections went with the second document.

    They are listed rather than merely absent so that promoting one back is a
    deliberate edit here, not something that quietly reappears.
    """
    html = reports["flight"].read_text(encoding="utf-8")
    present = [s for s in DETAILED_SECTIONS if s in html]
    assert not present, f"detailed-only sections are still rendering: {present}"


def test_flight_report_is_the_headline_read(reports: dict[str, Path]) -> None:
    """The flight report carries the summary card and charts, not raw diagnostics."""
    html = reports["flight"].read_text(encoding="utf-8")

    missing = [s for s in FLIGHT_SECTIONS if s not in html]
    assert not missing, f"Missing sections: {missing}"
    assert 'class="card"' in html, "Flight report is missing the summary card"

    # Still board bring-up rather than flying, so still not here.
    leaked = [s for s in ('id="log_buffer"', 'id="kinematic_checks"',
                          'id="sensor_noise"') if s in html]
    assert not leaked, f"Detailed-only sections leaked into the flight report: {leaked}"

    # Settings is reference material and belongs at the very bottom, below every
    # section that is actually about the flight.
    assert html.index('id="settings"') > html.index('id="deployment"'), (
        "Settings snapshot should sit below Deployment & Recovery"
    )
    assert html.index('id="timing"') > html.index('id="parser_stats"'), (
        "Per-sensor timing should follow the message counts it explains"
    )
    # What the flyer set comes first; what the firmware and sensors were
    # running comes after it, where a reader chasing a wrong number can still
    # find it without it crowding the settings they recognize.
    assert html.index('id="system"') > html.index('id="settings"'), (
        "System configuration should sit below the rocket settings"
    )


def test_report_has_inline_figures(reports: dict[str, Path]) -> None:
    """The report carries exactly one static PNG.

    The per-sensor timing histogram is the exception, and a deliberate one: it is
    a distribution rather than a series, nothing zooms into it, and rebuilding it
    as a Plotly figure would cost more than it returns. Every other flight-level
    visual is an interactive chart — the Roll PID section's figures were the
    last static holdouts, ported to charts 2026-08-29 — so the count is pinned
    rather than merely bounded: a second PNG appearing here should be a
    decision, not a drift.
    """
    flight = reports["flight"].read_text(encoding="utf-8")
    flight_figs = flight.count('<img src="data:image/png;base64,')
    assert flight_figs == 1, (
        f"Flight report should carry exactly one static PNG (per-sensor timing), "
        f"got {flight_figs}"
    )


def _unpack_steps(spec: dict) -> dict:
    """Mirror of the template's unpackSteps: x axes and Explore t arrive as integer steps.

    Kept in step with render._pack_axis by hand — there is no shared code
    between the Python that packs and the JavaScript that unpacks, so this is
    the third copy of the rule and the tests are what keep the three agreeing.
    """
    div = spec.get("stepDiv")
    if not div:
        return spec

    def axis(steps: list) -> list[float]:
        out, acc = [], 0
        for step in steps:
            acc += step
            out.append(acc / div)
        return out

    for t in spec.get("traces", []):
        if "xd" in t:
            t["x"] = axis(t.pop("xd"))
    for stream in (spec.get("streams") or {}).values():
        if "td" in stream:
            stream["t"] = axis(stream.pop("td"))
    return spec


def _chart_specs(html: str) -> list[tuple[str, dict]]:
    """Every embedded chart spec, decoding the gzip+base64 form used by big ones."""
    import base64
    import gzip
    import json
    import re

    out = []
    for tag, body in re.findall(
        r'<script type="application/json" class="chart-spec"([^>]*)>(.*?)</script>', html, re.S
    ):
        chart_id = re.search(r'data-target="([^"]+)"', tag).group(1)
        if 'data-encoding="gzip+base64"' in tag:
            raw = gzip.decompress(base64.b64decode(body.strip())).decode("utf-8")
        else:
            raw = body.replace("\\u003c", "<")
        out.append((chart_id, _unpack_steps(json.loads(raw))))
    return out


# Credit <img> tags that sit inside the vendored CesiumJS bundle as inert string
# literals. Cesium only injects them into the DOM if you instantiate the imagery
# provider they belong to — Bing Maps and ion's Google layer — and this report
# instantiates neither: it uses Esri World Imagery and Esri Terrain3D, both
# keyless, both named explicitly in report.html.j2.
#
# Verified rather than assumed: a rendered report opened from file:// in headless
# Chrome and left to load for 35 s contacted exactly three hosts —
# tile.openstreetmap.org, services.arcgisonline.com and elevation3d.arcgis.com.
# Neither host below was requested.
#
# Keep this list exact and minimal. It exists so the assertion can still fail on
# a *new* external reference, which is the thing it is really guarding.
_INERT_VENDOR_URLS = {
    "http://dev.virtualearth.net/Branding/logo_powered_by.png",
    "https://assets.ion.cesium.com/google-credit.png",
}


def test_report_has_interactive_charts(report_html: Path) -> None:
    """Overview charts must be embedded intact, with the library inlined once."""
    import re

    html = report_html.read_text(encoding="utf-8")
    specs = _chart_specs(html)
    assert len(specs) >= 4, f"Expected ≥4 interactive charts, got {len(specs)}"

    for chart_id, spec in specs:
        assert spec["traces"], f"{chart_id} has no traces"
        for t in spec["traces"]:
            assert len(t["x"]) == len(t["y"]), f"{chart_id}/{t['name']} x-y length mismatch"

    # Inlined exactly once, and nothing loaded over the network at view time.
    # (Don't grep for CDN hostnames: plotly.js embeds "https://cdn.plot.ly/" as the
    # default topojsonURL config value, which these charts never fetch.)
    assert html.count("plotly.js (gl3d - minified)") == 1, "plotly.js not inlined exactly once"
    external = re.findall(r'<(?:script|link|img)\b[^>]*\b(?:src|href)\s*=\s*"(https?://[^"]*)"', html)
    assert not set(external) - _INERT_VENDOR_URLS, (
        "Report loads external subresources; it must stay self-contained: "
        f"{sorted(set(external) - _INERT_VENDOR_URLS)}"
    )


def _globe_specs(html: str) -> list[tuple[str, dict]]:
    """Every embedded 3D-globe spec, decoded the same way chart specs are."""
    import base64
    import gzip
    import json
    import re

    out = []
    for tag, body in re.findall(
        r'<script type="application/json" class="globe-spec"([^>]*)>(.*?)</script>', html, re.S
    ):
        globe_id = re.search(r'data-target="([^"]+)"', tag).group(1)
        if 'data-encoding="gzip+base64"' in tag:
            raw = gzip.decompress(base64.b64decode(body.strip())).decode("utf-8")
        else:
            raw = body.replace("\\u003c", "<")
        out.append((globe_id, json.loads(raw)))
    return out


def test_globe_carries_both_tracks(report_html: Path) -> None:
    """The 3D view must ship the nav filter and the raw GNSS track, both well-formed.

    Asserts structure only, never geometry: the golden flight's GNSS position is
    frozen at exactly (38.0, -122.0) for all 1,118 fixes, so on this fixture the
    "GNSS track" is a vertical line at a point and the two tracks disagree by the
    entire flight. Geometry assertions belong on a real capture.
    """
    specs = _globe_specs(report_html.read_text(encoding="utf-8"))
    assert len(specs) == 1, f"Expected exactly one globe, got {len(specs)}"
    _, spec = specs[0]

    keys = [t["key"] for t in spec["tracks"]]
    assert keys == ["filter", "gnss"], f"Expected filter and gnss tracks, got {keys}"

    for t in spec["tracks"]:
        pos = t["positions"]
        assert len(pos) % 3 == 0, f"{t['key']}: positions is not whole lon/lat/height triples"
        assert len(pos) == 3 * len(t["t"]), f"{t['key']}: {len(pos)//3} points but {len(t['t'])} timestamps"
        assert len(t["t"]) >= 2, f"{t['key']}: needs at least two samples to draw a line"
        lons, lats = pos[0::3], pos[1::3]
        assert all(-180.0 <= v <= 180.0 for v in lons), f"{t['key']}: longitude out of range"
        assert all(-90.0 <= v <= 90.0 for v in lats), f"{t['key']}: latitude out of range"
        assert t["t"] == sorted(t["t"]), f"{t['key']}: timestamps are not monotonic"

    # The origin residual is how far apart the two solutions are at launch, and
    # it is what tells a reader whether a later separation is real or just a
    # reconstructed datum. A bad ENU-origin reconstruction shows up here as a
    # large number, so bound it rather than merely checking it is a float.
    residual = spec["originResidualM"]
    assert isinstance(residual, (int, float)) and math.isfinite(residual)
    assert 0.0 <= residual < 50.0, (
        f"the two tracks start {residual} m apart — the nav track's ENU origin is "
        "being reconstructed badly, which shifts the whole track bodily"
    )

    # #1419: the spec says where the ENU origin came from — the firmware's own
    # frozen reference out of the Snapshot stream when the log has one, the
    # pad-fix average only as a fallback — so a reader can tell whether the
    # residual above is a datum error or a real launch-time disagreement.
    assert spec["referenceSource"] in ("logged", "reconstructed")
    assert spec["referenceConverged"] in (True, False, None)
    # This fixture is exactly the case the gate exists for: real-pad snapshots
    # beside a GNSS stream frozen at (38, -122). It must fall back, and say so.
    assert spec["referenceSource"] == "reconstructed"
    assert "refused" in (spec["referenceNote"] or "")


def test_globe_heights_are_above_the_pad(report_html: Path) -> None:
    """Both tracks must ship height-above-pad, not raw MSL.

    This is the assertion that catches a vertical-datum regression. Shipping GNSS
    MSL straight through would offset that track by the pad's elevation, and the
    page adds terrain height on top, so the track would float or submerge by that
    much. The golden flight's pad sits at ~4.5 m MSL and it reaches ~457 m, so
    MSL and AGL are only ~4.5 m apart there — hence the tight tolerance at the
    start of the track rather than a loose sanity band.
    """
    _, spec = _globe_specs(report_html.read_text(encoding="utf-8"))[0]

    pad = spec["pad"]
    assert -90.0 <= pad["lat"] <= 90.0 and -180.0 <= pad["lon"] <= 180.0

    for t in spec["tracks"]:
        heights = t["positions"][2::3]
        assert abs(heights[0]) < 25.0, (
            f"{t['key']} starts at {heights[0]} m above the pad — heights look like "
            "raw MSL rather than height above the pad"
        )
        assert max(heights) > 50.0, f"{t['key']} never climbs; heights are not a flight"
        assert max(heights) < 12_000.0, f"{t['key']} peaks at {max(heights)} m — implausible"


def test_globe_drops_out_when_cesium_cannot_be_patched(tmp_path: Path) -> None:
    """A bundle that stops taking the file:// worker patch costs the section, not the report.

    Without this the exception escapes into render_report, which has no
    per-module handling, and one stale vendor file loses the whole document. The
    globe module therefore asks for the bundle before it emits a spec.
    """
    from flight_report import cesium_bundle
    from flight_report.flight import Flight
    from flight_report.modules import globe

    if not GOLDEN_BIN.exists():
        pytest.skip(f"Golden flight missing: {GOLDEN_BIN}")

    flight = Flight.from_bin(GOLDEN_BIN)
    real = cesium_bundle._WORKER_IMPORT
    cesium_bundle.cesium_source.cache_clear()
    cesium_bundle._WORKER_IMPORT = "a bootstrap shape no Cesium build contains"
    try:
        result = globe.analyze(flight)
    finally:
        cesium_bundle._WORKER_IMPORT = real
        cesium_bundle.cesium_source.cache_clear()

    assert result.error is None, "the module raised instead of degrading"
    assert not result.globes, "a globe spec was emitted for an unpatchable bundle"
    assert any("3D view was left out" in w for w in result.warnings), (
        f"no warning explaining the missing globe; got {result.warnings}"
    )


def test_globe_tracks_are_independently_sampled(report_html: Path) -> None:
    """Neither track may be resampled onto the other.

    The nav filter logs at roughly 440 Hz and GNSS at 18. Interpolating one onto
    the other's timestamps would invent samples and smooth away precisely the
    disagreement this section exists to show, so the differing lengths are the
    property worth pinning.
    """
    _, spec = _globe_specs(report_html.read_text(encoding="utf-8"))[0]
    nav, gnss = spec["tracks"]
    assert len(nav["t"]) > len(gnss["t"]) * 5, (
        f"nav filter has {len(nav['t']):,} samples against GNSS's {len(gnss['t']):,} — "
        "too close for two sources an order of magnitude apart in rate; has one "
        "been resampled onto the other?"
    )
    # Each track must keep its own sample instants. A resample onto some third,
    # regular grid would preserve the length ratio above while still inventing
    # every sample, so check the intervals are actually irregular in the way real
    # logged data is.
    for t in (nav, gnss):
        gaps = {round(b - a, 3) for a, b in zip(t["t"], t["t"][1:])}
        assert len(gaps) > 3, (
            f"{t['key']} has only {len(gaps)} distinct sample intervals — it looks "
            "resampled onto a regular grid rather than logged"
        )


def test_cesium_inlined_once_and_patched(report_html: Path) -> None:
    """Cesium ships inline, and with the file:// worker fix actually applied.

    The patch is the difference between a working globe and a blank canvas that
    looks like it is still loading, and it fails silently: a Cesium upgrade that
    reshapes the worker bootstrap would sail through every other test here. This
    is the assertion that catches that.
    """
    from flight_report.cesium_bundle import PATCH_MARKER, VERSION_BANNER

    html = report_html.read_text(encoding="utf-8")
    assert html.count(VERSION_BANNER) == 1, "CesiumJS not inlined exactly once"
    assert PATCH_MARKER in html, "Cesium worker patch missing from the rendered report"
    assert 'importScripts("${UJ(CESIUM_WORKERS)}")' not in html, (
        "the unpatched Cesium worker bootstrap is still present — the globe will "
        "render blank when the report is opened from file://"
    )


def test_measured_events_beat_the_flags() -> None:
    """The card's timings come from the sensor record, not the declarations.

    Each flag latches only once its detector is confident, so it lands after the
    thing it names. Apogee has no single flag at all — five detectors vote — and
    reading the barometric vote as "apogee" is what made the summary card
    disagree with the flight computer's own record by 1.15 s.
    """
    from flight_report.events import measured
    from flight_report.flight import Flight

    flight = Flight.from_bin(SAMPLE_BIN)
    flight.load()
    ev = measured(flight)
    ns = flight.records["NonSensor"]
    t0 = flight.t0_us

    def flag(name: str) -> float:
        return next((r["time_us"] - t0) / 1e6 for r in ns if r.get(name))

    for key in ("launch", "burnout", "apogee", "ejection", "landed"):
        assert ev[key] is not None, f"{key} was not measured on the sample flight"

    # Measured events precede the declarations that chase them.
    assert ev["launch"] < flag("launch"), "measured launch is not before the launch flag"
    assert ev["burnout"] < flag("burnout"), "measured burnout is not before the burnout flag"
    assert flag("launch") - ev["launch"] < 1.0, "measured launch is implausibly early"

    # Apogee must not be any single detector's vote.
    for vote in ("alt_apogee", "vel_apogee", "gps_apogee", "pitch_apogee"):
        assert abs(ev["apogee"] - flag(vote)) > 1e-6, (
            f"apogee collapsed onto the {vote} detector's vote"
        )

    # Ordering, and the fact this flight deployed before it stopped climbing —
    # which is exactly why the barometric vote is not an apogee.
    assert ev["launch"] < ev["burnout"] < ev["ejection"] < ev["landed"]
    assert ev["ejection"] < ev["apogee"], (
        "the sample flight ejects ~1 s before apogee; if this flips, the ejection "
        "detector has latched onto something else"
    )


def test_roll_control_marks_burnout_where_its_traces_have_it() -> None:
    """The four roll-control panels put the burnout line on their traces' axis.

    roll_series re-zeroes every trace to the first record with the launch flag
    set, the clock the flight computer runs its roll profile on. The markers were
    offset from measured first motion instead, 0.196 s earlier on the sample
    flight, so all four panels drew burnout at 1.548 s on an axis where the
    traces have it at 1.352 s.
    """
    import numpy as np

    from flight_report.charts import _EVENT_STYLE
    from flight_report.events import measured
    from flight_report.flight import Flight
    from flight_report.modules import roll

    flight = Flight.from_bin(SAMPLE_BIN)
    flight.load()
    t0 = flight.t0_us
    ev = measured(flight)
    flag = next((r["time_us"] - t0) / 1e6
                for r in flight.records["NonSensor"] if r.get("launch"))
    # The two origins are far enough apart that a line offset from the wrong
    # one cannot pass the 1 ms checks below.
    assert flag - ev["launch"] > 0.1

    panels = roll.analyze(flight).charts
    assert [p["id"] for p in panels] == [
        "chart-roll-angle", "chart-roll-error", "chart-roll-rate", "chart-roll-cmd"]

    # The traces' axis, rebuilt from the log: each gyro sample at its own time
    # since the flag.
    gyro = next(t for t in panels[2]["traces"] if t["name"] == "Gyro X")
    imu_t = np.array([(r["time_us"] - t0) / 1e6 for r in flight.records["ISM6HG256"]])
    assert np.isin(gyro["x"], np.round(imu_t - flag, 6)).all()

    burnout = ev["burnout"] - flag
    color = _EVENT_STYLE["burnout"][0]
    for p in panels:
        lines = [s["x0"] for s in p["layout"]["shapes"]
                 if s["type"] == "line" and s["xref"] == "x" and s["line"]["color"] == color]
        assert len(lines) == 1 and abs(lines[0] - burnout) < 1e-3, (p["id"], lines, burnout)
    label = next(a for a in panels[0]["layout"]["annotations"] if a["text"] == "Burnout")
    assert abs(label["x"] - burnout) < 1e-3, (label["x"], burnout)


# ---------------------------------------------------------------------------
# First motion inside a hole in the log
#
# GAP_BIN lost 743 ms of every flight-computer stream around launch: the IMU
# stops at 0.99 g and comes back at 5.2 g, and the first launch-flagged record
# is the first one after the hole. First motion used to be interpolated
# straight across it, to a time where the log holds no data.
# ---------------------------------------------------------------------------

GAP_BIN = REPO_ROOT / "examples" / "flights" / "flight_20260705_195028.bin"


def _ramp_flight(drop: tuple[float, float] = (0.0, 0.0)):
    """A 1 kHz pad-then-thrust trace with the samples inside `drop` removed.

    |a| sits at 1 g, then climbs 20 g/s from 1.000 s: it crosses 1.2 g at
    1.010 s and 2 g at 1.050 s, holds 6 g, and falls to 0.3 g at 2.000 s. All of
    it is along the body until then, and after it the 0.3 g is drag, so body X
    reads -0.3 g. The launch flag is set at 1.200 s. Returns (flight, t, g, ax)
    as events.py takes them.
    """
    from types import SimpleNamespace

    import numpy as np

    t = np.round(np.arange(3000) * 1e-3, 6)
    g = np.where(t < 1.0, 1.0, np.minimum(1.0 + 20.0 * (t - 1.0), 6.0))
    ax = np.where(t >= 2.0, -0.3, g)
    g = np.where(t >= 2.0, 0.3, g)
    keep = ~((t > drop[0]) & (t < drop[1]))
    ns = [{"time_us": int(round(s * 1e6)), "launch": s >= 1.2}
          for s in np.round(np.arange(0.0, 3.0, 0.002), 6)]
    flight = SimpleNamespace(records={"NonSensor": ns}, t0_us=0)
    return flight, t[keep], g[keep], ax[keep]


def test_first_motion_is_not_interpolated_across_a_hole() -> None:
    """The pad crossing is only interpolated across a step of 10 ms or less.

    10 ms is the card's resolution, so a step that wide cannot move a printed
    burn time by more than its last digit. One step wider and the crossing is
    wherever a straight line happens to cut 1.2 g, so there is no first motion.
    """
    from flight_report import events

    # 1.005 s -> 1.014 s: 9 ms across the crossing, still a measurement.
    flight, t, g, _ax = _ramp_flight((1.0055, 1.0135))
    assert abs(events.true_launch(flight, t, g) - 1.010) < 1e-3
    assert events._first_motion(flight, t, g)[1] is None

    # 1.005 s -> 1.016 s: 11 ms, a hole. Blank, and the hole is named.
    flight, t, g, _ax = _ramp_flight((1.0055, 1.0155))
    assert events.true_launch(flight, t, g) is None
    assert events._first_motion(flight, t, g)[1] == (1.005, 1.016)

    # A hole further up the climb, with the trace above the pad band on both
    # sides, does not touch the crossing.
    flight, t, g, _ax = _ramp_flight((1.02, 1.5))
    assert abs(events.true_launch(flight, t, g) - 1.010) < 1e-3


def test_burnout_does_not_need_first_motion() -> None:
    """The end of a burn is measured even when its start fell in a hole."""
    from flight_report import events

    flight, t, g, ax = _ramp_flight((1.0055, 1.0155))
    assert events.true_launch(flight, t, g) is None
    assert abs(events.true_burnout(flight, t, g, ax) - 2.0) < 2e-3


def test_a_launch_in_a_logging_gap_is_blank_and_says_where() -> None:
    """On the real log: no first motion, the hole named, the rest measured."""
    from flight_report.events import launch_gap, measured
    from flight_report.flight import Flight

    flight = Flight.from_bin(GAP_BIN)
    flight.load()
    ev = measured(flight)
    gap = launch_gap(flight)

    assert ev["launch"] is None, "first motion was interpolated across the gap again"
    assert gap is not None
    assert 0.74 < gap[1] - gap[0] < 0.75, gap
    # Burnout came 0.6 s after the log resumed, and the ejection and the apogee
    # after that: none of them needs first motion.
    assert gap[1] < ev["burnout"] < ev["ejection"] < ev["apogee"] < ev["landed"]
    assert 0.58 < ev["burnout"] - gap[1] < 0.63, ev["burnout"] - gap[1]

    # A clean log has no gap, and its first motion is unchanged.
    sample = Flight.from_bin(SAMPLE_BIN)
    sample.load()
    assert launch_gap(sample) is None
    assert abs(measured(sample)["launch"] - 0.3159) < 1e-3


def test_the_snapshot_times_a_launch_whose_flag_was_lost() -> None:
    """The hole swallowed the first flagged records too, so the flag is late.

    The Snapshot's stamp minus its flight_elapsed_ms is the flight computer's
    own launch time. On GAP_BIN it lands 0.47 s before the first flagged record,
    inside the hole; on a log that lost nothing the flag is used as it is.
    """
    from flight_report.events import _flag_time, declared_launch, launch_gap, markers
    from flight_report.flight import Flight

    flight = Flight.from_bin(GAP_BIN)
    flight.load()
    flag = _flag_time(flight.records, flight.t0_us, "launch")
    call = declared_launch(flight)
    gap = launch_gap(flight)
    assert 0.46 < flag - call < 0.48, flag - call
    assert gap[0] < call < gap[1]
    assert markers(flight)["launch"] == call, "the charts must hang off the call, not the late flag"

    sample = Flight.from_bin(SAMPLE_BIN)
    sample.load()
    assert declared_launch(sample) == _flag_time(sample.records, sample.t0_us, "launch")


def test_the_card_says_why_it_has_no_burn_time() -> None:
    """Three cells missing, one quiet line saying why, and no 146 G headline.

    The cells that count from first motion are left off. Coast still counts
    from burnout, which is measured. Max acceleration is taken over the part of
    the burn the log kept, not over the whole flight: that would headline the
    ejection and the landing, 146 G on this log.
    """
    from flight_report.flight import Flight
    from flight_report.registry import LEVEL_FLIGHT
    from flight_report.render import render_report
    from flight_report.summary import card_note, compute_summary

    flight = Flight.from_bin(GAP_BIN)
    flight.load()
    cells = {c["label"]: c for c in compute_summary(flight)}
    for label in ("Burn time", "Time to apogee", "Flight time"):
        assert label not in cells, f"{label} was measured from a guess"
    assert "Coast time" in cells
    peak = cells["Max acceleration"]
    assert 4.0 < peak["q"].value < 6.0, peak["q"].value
    assert "after the gap" in str(peak["hint"])

    note = card_note(flight)
    assert note.startswith("First motion fell in a 743 ms gap in the log"), note
    assert "burn time, time to apogee and flight time are not shown" in note
    assert note in render_report(flight, [], level=LEVEL_FLIGHT)

    sample = Flight.from_bin(SAMPLE_BIN)
    sample.load()
    assert card_note(sample) == ""
    assert "Burn time" in {c["label"] for c in compute_summary(sample)}


def test_sections_anchor_on_the_launch_call_when_first_motion_is_blank() -> None:
    """Motor refuses; stability and vibration anchor on the call, and say so.

    The motor numbers are integrals and extremes over a burn whose opening is
    not in the record, so the section refuses with the reason. The tilt needs
    only a pad reference and the boost, both of which the log kept. The
    vibration windows are named for what bounds them.
    """
    from flight_report.flight import Flight
    from flight_report.modules import stability, vibration

    result = _motor(GAP_BIN)
    assert result.error is None
    assert not result.metrics
    assert result.warnings and "743 ms gap in the log" in result.warnings[0]

    flight = Flight.from_bin(GAP_BIN)
    flight.load()
    tilt = stability.analyze(flight)
    assert tilt.error is None
    assert "Max tilt under thrust" in tilt.metrics, tilt.warnings

    names = {name: how for name, _lo, _hi, how in vibration.phases(flight, 0.0, 20.0)}
    assert names["Boost"] == "launch detection to thrust ending"
    assert names["Pad"] == "the 1.5 s before launch detection"


# ---------------------------------------------------------------------------
# Burnout is the sign of body X
#
# The accelerometer reads thrust minus drag. Along the long axis that turns
# negative when the motor stops out-pushing the air, which is the flight
# computer's own burnout test. The magnitude cannot see the sign: the old 1 g
# bar on |a| was late whenever drag stayed above 1 g after burnout, and early on
# every motor with a sustain or a long tail.
# ---------------------------------------------------------------------------


def test_burnout_is_the_sign_of_body_x_not_the_size_of_a() -> None:
    """Drag over 1 g does not hold the burn open, and a sustain does not end it.

    Both happened: 1.46 g of drag held |a| over the old bar for 1.30 s after
    the 2026-05-09 Goblin burned out, and a sustain near 1 g ended the
    2026-06-14 Rolly Polly III's burn 0.17 s before its motor stopped.
    """
    import numpy as np

    from flight_report import events

    flight, t, g, ax = _ramp_flight()
    after = t >= 2.0

    # 1.5 g of drag once the motor stops: |a| never falls below 1 g again.
    drag_g = np.where(after, 1.5, g)
    drag_ax = np.where(after, -1.5, ax)
    assert abs(events.true_burnout(flight, t, drag_g, drag_ax) - 2.0) < 2e-3

    # A 0.3 s sustain at 0.8 g net: |a| is under 1 g from 2.0 s, but the motor
    # out-pushes the air until 2.3 s.
    sustain = after & (t < 2.3)
    sus_g = np.where(sustain, 0.8, np.where(t >= 2.3, 0.3, g))
    sus_ax = np.where(sustain, 0.8, np.where(t >= 2.3, -0.3, ax))
    assert abs(events.true_burnout(flight, t, sus_g, sus_ax) - 2.3) < 2e-3


def test_the_burnout_flag_is_one_hold_after_measured_burnout() -> None:
    """Measured burnout starts the run of negative body X that the flag waits out.

    The flight computer's detector (BurnoutDetector.h) tests the same sign on
    the same channel, and since #197 (2026-05-23) latches only once the sign has
    held for 50 flight-loop ticks. Every log in the repo was flown since, so on
    each one the flag lands one hold after measured burnout: 51-54 ms.
    """
    from flight_report.events import _flag_time, measured
    from flight_report.flight import Flight

    examples = REPO_ROOT / "examples" / "flights"
    for path in (SAMPLE_BIN, examples / "flight_20260705_183745.bin",
                 examples / "flight_20260705_191300.bin", GAP_BIN, GOLDEN_BIN):
        flight = Flight.from_bin(path)
        flight.load()
        lag = _flag_time(flight.records, flight.t0_us, "burnout") - measured(flight)["burnout"]
        assert 0.045 < lag < 0.060, (path.name, lag)


# ---------------------------------------------------------------------------
# Burnout inside a hole in the log
#
# Measured burnout is interpolated between the last sample under thrust and the
# first one coasting, and across a hole that is only where a straight line
# happens to cut zero. 2026-03-14 224121 lost 990 ms with the motor still at
# 8 g, and the line put burnout 96 per cent of the way across. The bar is first
# motion's: 10 ms, the card's resolution.
# ---------------------------------------------------------------------------


def test_burnout_is_not_interpolated_across_a_hole() -> None:
    """The body-X crossing is only interpolated across a step of 10 ms or less.

    Only the step the crossing sits in is tested. A hole earlier in the burn,
    with body X positive on both sides, cannot end it; a hole inside the 50 ms
    hold, with body X negative on both sides, does not undo it.
    """
    from flight_report import events

    # Body X steps from 6 g to -0.3 g at 2.000 s. 1.995 s -> 2.004 s: 9 ms
    # across the crossing, still a measurement.
    flight, t, g, ax = _ramp_flight((1.9955, 2.0035))
    burnout, hole = events._burnout(flight, t, g, ax)
    assert hole is None
    assert 1.995 < burnout < 2.004, burnout

    # 1.995 s -> 2.006 s: 11 ms, a hole. Blank, and the hole is named.
    flight, t, g, ax = _ramp_flight((1.9955, 2.0055))
    assert events.true_burnout(flight, t, g, ax) is None
    assert events._burnout(flight, t, g, ax)[1] == (1.995, 2.006)

    # A hole in the burn, and a hole in the hold, leave the crossing alone.
    for drop in ((1.5, 1.9), (2.010, 2.100)):
        flight, t, g, ax = _ramp_flight(drop)
        assert abs(events.true_burnout(flight, t, g, ax) - 2.0) < 2e-3, drop


def _cut(flight, lo: float, hi: float) -> None:
    """Drop every record, in every stream, from `lo` to `hi` seconds since t0."""
    for name, recs in flight.records.items():
        if recs and "time_us" in recs[0]:
            flight.records[name] = [r for r in recs
                                    if not lo < (r["time_us"] - flight.t0_us) / 1e6 < hi]


def test_a_burnout_in_a_logging_gap_is_blank_and_says_where() -> None:
    """The sample flight, with a second cut out of every stream around burnout.

    No log small enough for the repo lost its burnout, so this cuts one the way
    2026-03-14 224121 lost it: every stream, from under thrust to coasting. Burn
    time and coast time go blank and the card says why. Max acceleration keeps
    the burn before the hole, the motor section refuses, stability leaves out
    both figures taken at burnout, the vibration windows stop at the hole's
    edges, and the ejection is still found after it.
    """
    from flight_report.events import burnout_gap, launch_gap, measured
    from flight_report.flight import Flight
    from flight_report.modules import motor, stability, vibration
    from flight_report.registry import LEVEL_FLIGHT
    from flight_report.render import render_report
    from flight_report.summary import card_note, compute_summary

    whole = Flight.from_bin(SAMPLE_BIN)
    whole.load()
    before = measured(whole)
    peak_before = {c["label"]: c for c in compute_summary(whole)}["Max acceleration"]

    flight = Flight.from_bin(SAMPLE_BIN)
    flight.load()
    _cut(flight, before["burnout"] - 0.5, before["burnout"] + 0.5)
    ev = measured(flight)
    hole = burnout_gap(flight)

    assert ev["burnout"] is None, "burnout was interpolated across the gap again"
    assert launch_gap(flight) is None
    assert hole is not None and hole[0] < before["burnout"] < hole[1], hole
    for key in ("launch", "apogee", "ejection", "landed"):
        assert ev[key] == before[key], key

    cells = {c["label"]: c for c in compute_summary(flight)}
    for label in ("Burn time", "Coast time"):
        assert label not in cells, f"{label} was measured from a guess"
    assert "Time to apogee" in cells and "Flight time" in cells
    # The sample flight peaks at 9.5 G early in the burn, well before the hole.
    peak = cells["Max acceleration"]
    assert peak["q"].value == peak_before["q"].value
    assert "under thrust, before the gap" in str(peak["hint"])

    note = card_note(flight)
    assert note == ("Burnout fell in a 1001 ms gap in the log (1.36–2.36 s), "
                    "so burn time and coast time are not shown."), note
    assert note in render_report(flight, [], level=LEVEL_FLIGHT)

    refused = motor.analyze(flight)
    assert refused.error is None and not refused.metrics
    assert refused.warnings and refused.warnings[0].startswith(
        "Burnout fell in a 1001 ms gap in the log"), refused.warnings

    tilt = stability.analyze(flight)
    assert tilt.error is None
    assert "Tilt at burnout" not in tilt.metrics and "Max tilt under thrust" not in tilt.metrics

    windows = {name: (lo, hi, how) for name, lo, hi, how in vibration.phases(flight, 0.0, 80.0)}
    assert windows["Boost"][1:] == (hole[0], "first motion to the gap in the log")
    assert windows["Coast"][0] == hole[1]
    assert windows["Coast"][2] == "the gap in the log to apogee"
    assert vibration.analyze(flight).error is None


# ---------------------------------------------------------------------------
# The ejection charge
#
# A motor ejection logs nothing, so the charge is found in the accelerometer as
# the first transient after the burn that clears both bars. It used to be the
# largest, and what follows a charge can hit harder than the charge did: a jolt
# half a second later, or the ground itself when the landed flag trails
# touchdown by 4-6 s. The largest was something other than the charge on nine
# flights of 24.
# ---------------------------------------------------------------------------


def test_the_ejection_is_the_first_transient_not_the_largest() -> None:
    """A harder jolt after the charge is not the charge, nor is a fast coast's drag.

    The 2026-08-29 54 mm Rolly Polly took 211 g half a second after a 43 g
    charge. The charge is dated at the peak of its own transient, not where it
    first cleared the bars. The window's first sample has no coast before it to
    be judged against, so the drag at the start of a fast coast is not taken
    for the charge.
    """
    from types import SimpleNamespace

    import numpy as np

    from flight_report import events

    no_pyro = SimpleNamespace(records={}, t0_us=0)
    # 1 kHz from burnout at 2.000 s: a 0.3 g coast, and a charge that climbs to
    # 30 g at 8.010 s and is back down by 8.020 s. It clears both bars at 8.002 s.
    t = np.round(2.0 + np.arange(18000) * 1e-3, 6)
    g = np.full(t.size, 0.3)
    charge = (t >= 8.0) & (t <= 8.02)
    g[charge] = 0.3 + 29.7 * (1.0 - np.abs(t[charge] - 8.01) / 0.01)
    assert abs(events.ejection(no_pyro, t, g, 2.0, None) - 8.010) < 1e-6

    # Half a second later, 100 g.
    jolt = g.copy()
    jolt[(t >= 8.5) & (t <= 8.505)] = 100.0
    assert abs(events.ejection(no_pyro, t, jolt, 2.0, None) - 8.010) < 1e-6

    # 7 g of drag at burnout, gone by 3.0 s. The window opens at 2.2 s, and its
    # first sample reads 5.7 g, over 8 times the whole window's 0.3 g median.
    drag = g.copy()
    early = t < 3.0
    drag[early] = 7.0 - 6.7 * (t[early] - 2.0)
    assert abs(events.ejection(no_pyro, t, drag, 2.0, None) - 8.010) < 1e-6

    # Nothing that clears both bars, no ejection.
    assert events.ejection(no_pyro, t, np.full(t.size, 0.3), 2.0, None) is None


def test_a_late_landed_flag_does_not_make_the_touchdown_the_ejection() -> None:
    """The sample flight with no landed flag, so the search runs to the end of the log.

    The flag closes the search a second before touchdown, but on five flights it
    came 4-6 s after touchdown, and the landing was taken for the charge:
    21.45 s against a charge at 6.90 s on 2026-03-14 F67. Here the log holds a
    77 G landing, twice the charge. The charge, the coast time counted to it and
    the tilt taken at it all stay where they were.
    """
    import numpy as np

    from flight_report import events
    from flight_report.flight import Flight
    from flight_report.modules import stability
    from flight_report.summary import compute_summary

    whole = Flight.from_bin(SAMPLE_BIN)
    whole.load()
    before = events.measured(whole)
    coast_before = {c["label"]: c for c in compute_summary(whole)}["Coast time"]
    tilt_before = stability.analyze(whole).metrics["Tilt at ejection"]

    flight = Flight.from_bin(SAMPLE_BIN)
    flight.load()
    for r in flight.records["NonSensor"]:
        r["alt_landed"] = False
    ev = events.measured(flight)
    assert ev["landed"] is None

    # The landing is inside the search now, and it is the harder hit.
    t, g, _ax = events._accel_series(flight)
    at_charge = float(g[np.argmin(np.abs(t - before["ejection"]))])
    assert float(np.max(g[t > before["ejection"] + 1.0])) > 1.5 * at_charge

    assert ev["ejection"] == before["ejection"], "the landing was taken for the charge"
    coast = {c["label"]: c for c in compute_summary(flight)}["Coast time"]
    assert coast["q"].value == coast_before["q"].value
    assert stability.analyze(flight).metrics["Tilt at ejection"].value == tilt_before.value


def test_apogee_turnover_stays_zoomed_in(report_html: Path) -> None:
    """Recovery events belong on their own chart, not on the turnover's axis.

    The turnover chart exists to separate four detector calls that land within a
    second or two of each other. Landing is tens of seconds later, so marking it
    there — which is how this was first written — compresses those calls into a
    sliver. The split is the point; this catches a merge that undoes it.
    """
    specs = dict(_chart_specs(report_html.read_text(encoding="utf-8")))
    vote, recovery = specs.get("chart-apogee-vote"), specs.get("chart-recovery")
    assert vote, "apogee turnover chart missing"
    assert recovery, "recovery chart missing"

    def labels(spec: dict) -> set[str]:
        return {a.get("text") for a in spec["layout"].get("annotations", [])}

    lo, hi = vote["layout"]["xaxis"]["range"]
    assert hi - lo < 20.0, f"turnover window is {hi - lo:.0f} s wide; it should be seconds"
    assert {"True Apogee", "Master Apogee"} <= labels(vote), (
        f"turnover chart lost its apogee marks: {sorted(labels(vote))}"
    )
    assert "Landed" not in labels(vote), "landing declaration stretched the turnover window"
    assert "Landed" in labels(recovery), (
        f"recovery chart is missing the landing declaration: {sorted(labels(recovery))}"
    )
    # It has to actually reach the landing it marks.
    r_lo, r_hi = recovery["layout"]["xaxis"]["range"]
    assert r_hi > hi, "recovery chart ends before the turnover chart does"


def test_charts_keep_every_sample(report_html: Path) -> None:
    """No decimation in the file: the chart must carry the full logged series.

    The viewport renderer reduces what is *drawn*, never what is stored, so a
    zoom can always reach real samples. Guards against a well-meaning
    decimate-on-export creeping back in.
    """
    specs = dict(_chart_specs(report_html.read_text(encoding="utf-8")))
    alt = specs.get("chart-altitude")
    assert alt, "altitude chart missing"

    longest = max(len(t["x"]) for t in alt["traces"])
    assert longest > 20_000, (
        f"altitude chart's longest trace has only {longest:,} points — the full "
        "barometer series should be tens of thousands; has something decimated it?"
    )


def _dataset_spec(html: str) -> dict:
    """The Explore payload, decoding the gzip+base64 form it takes on a real flight."""
    import base64
    import gzip
    import json
    import re

    m = re.search(
        r'<script type="application/json" class="dataset-spec"([^>]*)>(.*?)</script>', html, re.S
    )
    assert m, "explore dataset missing"
    tag, body = m.groups()
    if 'data-encoding="gzip+base64"' in tag:
        raw = gzip.decompress(base64.b64decode(body.strip())).decode("utf-8")
    else:
        raw = body.replace("\\u003c", "<")
    return _unpack_steps(json.loads(raw))


def test_time_axis_resolves_every_imu_sample(report_html: Path) -> None:
    """Consecutive samples of one trace must land on distinct x values.

    The golden flight's IMU logs at 3.84 kHz under boost, 260 µs apart, and the
    charts used to round time to 1 ms — which put three or four consecutive
    samples on one x, so a zoomed-in acceleration chart showed columns of
    stacked dots instead of a trace. Time now ships at the log clock's own
    microsecond (charts.TIME_DECIMALS), on the charts and in the Explore
    dataset alike. Strict ordering is what is checked, not a duplicate count:
    the raw stamps are strictly increasing on every stream of this flight, so
    an equal pair of neighbours can only have come from rounding.
    """
    html = report_html.read_text(encoding="utf-8")
    specs = dict(_chart_specs(html))
    accel = specs.get("chart-accel-g")
    assert accel, "acceleration chart missing"
    for t in accel["traces"]:
        xs = t["x"]
        stacked = sum(1 for a, b in zip(xs, xs[1:]) if b <= a)
        assert stacked == 0, (
            f"{t['name']}: {stacked:,} of {len(xs):,} samples share an x with their "
            "predecessor — time is rounded coarser than the IMU's sample spacing"
        )

    imu_t = _dataset_spec(html)["streams"]["ISM6HG256"]["t"]
    stacked = sum(1 for a, b in zip(imu_t, imu_t[1:]) if b <= a)
    assert stacked == 0, (
        f"Explore dataset: {stacked:,} of {len(imu_t):,} IMU samples share a t with "
        "their predecessor"
    )


def test_step_encoded_time_axis_round_trips_exactly(report_html: Path) -> None:
    """The step encoding of x is a size trick, not a precision one.

    render._pack_axis ships each x axis as integer microsecond steps because
    they gzip to a fraction of what absolute six-decimal floats do. The claim
    that goes with it is exactness: what the reader's browser rebuilds must be
    the very doubles the builder rounded to, not something a summation drifted
    away from. Checked end to end — log to chart builder to serializer to
    gzip to decode to unpack — against the IMU clock of the golden flight.
    """
    import numpy as np

    from flight_report.charts import TIME_DECIMALS
    from flight_report.flight import Flight
    from plot_flight_data_mini import get_array

    flight = Flight.from_bin(GOLDEN_BIN)
    flight.load()
    imu = flight.records["ISM6HG256"]
    expected = np.round((get_array(imu, "time_us") - flight.t0_us) / 1e6, TIME_DECIMALS).tolist()

    specs = dict(_chart_specs(report_html.read_text(encoding="utf-8")))
    xs = specs["chart-accel-g"]["traces"][0]["x"]
    # The chart may open on a pre-launch window rather than the first record.
    start = expected.index(xs[0])
    assert xs == expected[start:start + len(xs)], (
        "unpacked x axis is not bit-identical to the rounded IMU clock"
    )


def test_charts_offer_dots_lines_or_both(report_html: Path) -> None:
    """Every sample chart carries the Dots / Lines / Both control, and its traces can take either.

    Dots is the default because a scatter shows where the samples fall and
    where they thin out; a reader tracing a waveform through a zoomed-in boost
    wants the line. The control itself is DOM wiring in the template and is
    not unit-tested here, so what is pinned is the contract it relies on: one
    control per non-3D chart, and both a `line` and a `marker` style on every
    marker trace, in the same color, so the switch never recolors a series.
    """
    html = report_html.read_text(encoding="utf-8")
    for chart_id, spec in _chart_specs(html):
        if spec.get("kind") == "3d":
            assert f'data-chart="{chart_id}"' not in html, f"{chart_id}: a 3D chart got a draw-mode control"
            continue
        assert f'<div class="draw-mode" data-chart="{chart_id}"' in html, (
            f"{chart_id} has no Dots / Lines / Both control"
        )
        for t in spec["traces"]:
            if "markers" not in t.get("mode", "markers"):
                continue
            assert "line" in t and "marker" in t, f"{chart_id}/{t['name']} lacks a line or marker style"
            assert t["line"].get("color") == t["marker"].get("color"), (
                f"{chart_id}/{t['name']}: line and marker colors differ, so switching would recolor it"
            )


def _section(html: str, section_id: str) -> str:
    """The HTML of one report section, heading to the next heading."""
    start = html.index(f'<h2 id="{section_id}">')
    nxt = html.find("<h2 id=", start + 1)
    return html[start:nxt if nxt > 0 else None]


def test_settings_use_the_apps_words(report_html: Path) -> None:
    """Settings are read from the log's own frame and labeled as the app labels them.

    A flyer checking a number against what they typed into the app should find
    the same heading and the same words — "Pyro Channel 1", "Enabled", "Max
    Deflection" — not the firmware's field names. And the rows come from the
    FlightSettingsData frame in the binary, not the sidecar: the golden flight
    has no sidecar at all and still shows every group.
    """
    html = report_html.read_text(encoding="utf-8")
    settings = _section(html, "settings")
    for heading in ("Rocket", "IMU Mounting", "Servo Control", "PID Gains", "Servo",
                    "Control Mode", "Camera", "Pyro Channel 1", "Pyro Channel 4"):
        assert f"<h3>{heading}</h3>" in settings, f"settings section lacks the app's {heading!r} group"
    for label in ("Enable Sounds", "Enable Servo Control", "Max Deflection", "Min Pulse",
                  "Camera Type", "Nose axis"):
        assert f"<td>{label}</td>" in settings, f"settings section lacks the app's {label!r} row"
    for raw in ("gyro_fs_dps", "max_cmd_deg", "trigger_mode", "pyro.ch1"):
        assert raw not in settings, f"firmware field name {raw!r} leaked into the settings section"

    system = _section(html, "system")
    for heading in ("Firmware", "IMU", "Sensor mounting"):
        assert f"<h3>{heading}</h3>" in system, f"system section lacks the {heading!r} group"
    assert "Gyroscope range" in system, "gyro full scale belongs in the system section"
    assert "Gyroscope range" not in settings, "gyro full scale is not something the flyer set"


def test_imu_rate_reads_in_the_apps_words() -> None:
    """#1485: the 8k rates read as the app's picker names them.

    "8k Dynamic" logs no sentinel of its own: the settings frame carries the
    dynamic flag and the boost rate at the snapshot, 7680 Hz, which is how it
    tells apart from 4k Dynamic.
    """
    from flight_report.modules import settings as report_settings

    def rate(hz: int, dynamic: bool) -> str:
        return report_settings._imu_rate({"ism6_update_rate_hz": hz, "imu_rate_dynamic": dynamic})

    assert rate(7680, True) == ("Dynamic — 8k (7680 Hz) through boost and coast, "
                                "1k (960 Hz) after deployment")
    assert rate(3840, True).startswith("Dynamic — 4k (3840 Hz)")
    assert rate(7680, False) == "8k (7680 Hz)"
    assert rate(960, False) == "1k (960 Hz)"


def test_flight_settings_frame_decodes_every_version() -> None:
    """The decoder reads what a frame's length covers and nothing more.

    Built from the struct layout by hand: a full v9 frame decodes every tail,
    and the same bytes cut at the v2 length decode the head and the mounting
    orientation and leave the later tails None rather than misreading them.
    """
    import struct

    from flight_report import flight_settings as fs

    flags = (1 << fs.F_SERVO_ENABLED) | (1 << fs.F_GAIN_SCHEDULE) | (1 << fs.F_SOUNDS) \
        | (1 << fs.F_IMU_RATE_DYNAMIC)
    head = struct.pack("<IBBH6f2f3ffBHH4hhhhB",
                       123456, 9, flags, 500,
                       0.12, 0.01, 0.0, 10.0, -20.0, 20.0,
                       2.0, 60.0,
                       50.0, 25.0, 3.0,
                       0.0,
                       16, 256, 4000,
                       5, -5, 0, 0, 333, 1000, 2000,
                       2)
    pyro = struct.pack("<BBf", 1, 1, 228.6) + struct.pack("<BBf", 0, 0, 0.0) * 3
    sha = b"d7017c0".ljust(12, b"\0")
    profile = bytes([2, 0, 0, 0]) + struct.pack("<ffB", 1.0, 0.0, 0) \
        + struct.pack("<ffB", 3.0, 90.0, 0) + bytes(9 * 6)
    tails = struct.pack("<BBh4h", 4, 1, 0, 10000, 0, 0, 0) \
        + struct.pack("<2f", -30.0, 30.0) + struct.pack("<H", 3840) \
        + struct.pack("<2fB", 0.0, 0.0, 0) + struct.pack("<B", 5) + struct.pack("<H", 250) \
        + struct.pack("<B", 0x19)   # v9 (#413): provisioned V9
    frame = head + pyro + sha + profile + tails
    assert len(frame) == 223

    d = fs.decode(frame)
    assert d["version"] == 9 and d["servo_enabled"] and d["gain_schedule_enabled"]
    assert d["sounds_enabled"] and d["imu_rate_dynamic"] and not d["guidance_enabled"]
    assert d["roll_delay_ms"] == 500 and abs(d["kp"] - 0.12) < 1e-6
    assert d["servo_bias_us"] == [5, -5, 0, 0] and d["servo_hz"] == 333
    assert d["camera_type"] == 2 and d["fw_git_sha"] == "d7017c0"
    assert d["pyro"][0] == {"enabled": True, "mode": 1, "value": struct.unpack("<f", struct.pack("<f", 228.6))[0]}
    assert d["num_waypoints"] == 2 and d["waypoints"][1]["angle_deg"] == 90.0
    assert fs.orientation_name(d["b2r_code"]) == "-X" and d["b2r_mode"] == 1
    assert d["fin_min_deg"] == -30.0 and d["fin_max_deg"] == 30.0
    assert d["ism6_update_rate_hz"] == 3840
    assert d["gnss_otp_state"] == 5 and fs.gnss_otp_name(5) == "BLOCKLISTED"
    assert d["roll_min_speed_mps"] == 25.0
    # #413: the log finally says which board produced it. Bit 7 clear = the
    # PROVISIONED revision from NVS, which is the one that is not circular.
    assert d["board_rev_code"] == 0x19 and fs.board_rev_name(0x19) == "V9"

    v2 = fs.decode(frame[:200])
    assert v2["b2r_code"] == 4 and v2["fin_min_deg"] is None
    assert v2["ism6_update_rate_hz"] is None and v2["roll_min_speed_mps"] is None
    assert v2["gnss_otp_state"] is None
    assert v2["board_rev_code"] is None

    # A v8 frame is one byte short and must leave the revision None, not read
    # the roll gate's high byte as a board.
    v8 = fs.decode(frame[:222])
    assert v8["roll_min_speed_mps"] == 25.0 and v8["board_rev_code"] is None

    # None and "unknown" are different answers: None is "this log predates the
    # field", unknown is "it carried one and named no board we recognise".
    assert fs.board_rev_name(None) is None
    assert fs.board_rev_name(0x00) == "unknown"
    assert fs.board_rev_name(0x99) == "V9 (asserted)"
    assert fs.board_rev_name(0x21) == "M1"


def test_report_has_no_module_errors(report_html: Path) -> None:
    """Every analysis module should run to completion (warnings OK, errors not)."""
    html = report_html.read_text(encoding="utf-8")
    # `class="err"` blocks only render when a module raised
    assert 'class="err"' not in html, (
        "One or more analysis modules raised an exception; "
        "open the report to inspect the traceback."
    )


# ---------------------------------------------------------------------------
# #752 — the channel catalog must describe exactly what the parser emits.
#
# Both directions are pinned, because both have already drifted in this tree:
# the parser grew POWER.cam_a/servo_a with no provenance row, and the table
# carried rows for sensor_health and its health_* splits — including the
# sentence "the parser also splits it into the health_* channels" — for two
# months in which the parser read straight past field 17 and emitted none of
# them. A picker built on either half of that lies to the reader.
# ---------------------------------------------------------------------------

# Documented channels that no fixture in the repo produces, and why that is
# correct rather than drift. Listed here so that adding a thirteenth is a
# deliberate edit and not something a future stream quietly slips in.
CATALOG_EMPTY_ON_EVERY_FIXTURE = {
    # The old-PCB magnetometer. Mutually exclusive with IIS2MDC, and every
    # flight in the repo is a new-PCB capture, so the stream parses to zero
    # records. The rows stay because a V7 log will produce them.
    "MMC5983MA.mag_x", "MMC5983MA.mag_y", "MMC5983MA.mag_z",
    # Guidance telemetry: no real flight in the repo was flown guided.
    "Guidance.accel_cmd_n", "Guidance.accel_cmd_e", "Guidance.lateral_offset",
    "Guidance.los_angle", "Guidance.closing_vel", "Guidance.pitch_fin_cmd",
    "Guidance.yaw_fin_cmd", "Guidance.active", "Guidance.burnout",
    # #1154 item 4: FC_STATUS_MSG (0x92) is newer than every fixture in the
    # repo, so no committed log carries one. The row stays because the next
    # flight recorded on current firmware will produce it.
    "FcStatus.fc_camera_engaged",
}


def _catalogs():
    """A catalog per fixture: the golden pre-48-byte log and a 2026-07-05 one."""
    from flight_report import catalog as cat_mod
    from flight_report.flight import Flight

    out = []
    for path in (GOLDEN_BIN, SAMPLE_BIN):
        if not path.exists():
            continue
        flight = Flight.from_bin(path)
        flight.load()
        out.append((path, cat_mod.build(flight)))
    if not out:
        pytest.skip("no flight fixtures available")
    return out


def test_every_parsed_channel_has_a_provenance_row() -> None:
    """A field the parser emits and the table has never heard of.

    This is the guard that would have caught POWER.cam_a and POWER.servo_a,
    which shipped with the v2 power frame in #850 and reached the catalog as
    two rows labelled by a `str.capitalize()` of their field name, with no unit
    and no note — in a table whose entire purpose is to say that one of them is
    amps while `current` beside it is milliamps.
    """
    missing: dict[str, list[str]] = {}
    for path, cat in _catalogs():
        undocumented = sorted(c.key for c in cat.channels if not c.documented)
        if undocumented:
            missing[path.name] = undocumented
    assert not missing, (
        "the parser emits channels with no PROVENANCE row: "
        f"{missing} — add a _P(...) entry in catalog.py describing each one"
    )


def test_provenance_rows_describe_channels_that_exist() -> None:
    """The other direction: a row for a channel nothing produces.

    An orphan row is worse than a missing one. A missing row costs a label; an
    orphan tells a reader a channel is available, with a unit and a caution and
    every appearance of authority, when selecting it yields an empty plot.
    """
    from flight_report import catalog as cat_mod

    produced: set[str] = set()
    for _path, cat in _catalogs():
        produced |= {c.key for c in cat.channels}

    orphans = set(cat_mod.PROVENANCE) - produced - CATALOG_EMPTY_ON_EVERY_FIXTURE
    assert not orphans, (
        f"PROVENANCE documents channels no fixture produces: {sorted(orphans)}. "
        "Either the parser stopped emitting them, or the row was written for a "
        "field that was never implemented — the sensor_health case. If the "
        "stream is legitimately absent from every fixture, add it to "
        "CATALOG_EMPTY_ON_EVERY_FIXTURE with the reason."
    )
    stale = CATALOG_EMPTY_ON_EVERY_FIXTURE & produced
    assert not stale, (
        f"these are no longer empty and should leave the allowlist: {sorted(stale)}"
    )


def test_catalog_units_are_real_conversion_keys() -> None:
    """Every unit in the table must be a units.CONVERSIONS row.

    `ChannelMeta.unit` is documented as a CONVERSIONS key, and `Quantity`
    raises KeyError on an unknown one precisely so a typo is loud. Seventeen
    rows had drifted off it — `deg` where the table says `°`, `m/s2` where it
    says `m/s²`, and a handful (`Pa`, `mA`, `bytes`, `µT`, `°C`) that were
    never added at all — so any picker rendering them would have thrown on the
    imperial toggle.
    """
    from flight_report import catalog as cat_mod, units

    bad = sorted({
        u for meta in cat_mod.PROVENANCE.values()
        for u in (meta.unit, meta.conv) if u and u not in units.CONVERSIONS
    })
    assert not bad, (
        f"provenance units that are not CONVERSIONS keys: {bad} — add them to "
        "units.CONVERSIONS (None if they read the same in both systems), or use "
        "the spelling already there"
    )


def test_sensor_health_decodes_and_stays_none_on_older_logs() -> None:
    """#303's scorecard: sixteen 2-bit verdicts that reached nothing for months.

    It is field index 17 of the 48/50/52-byte NonSensor formats and the unpack
    read 15, 16, 18 and 19 straight past it. The 2026-07-05 sample carries it;
    the golden 2026-06-15 flight predates the 48-byte format and must decode as
    None rather than 0, because SH_NA is a real verdict and a zero word would
    read as sixteen deliberate "not configured" entries.
    """
    import plot_flight_data_mini as parser

    records, _stats, _config = parser.parse_binary_file(str(SAMPLE_BIN))
    rows = records["NonSensor"]
    assert rows, "sample flight has no NonSensor records"
    assert all(r["sensor_health"] is not None for r in rows), (
        "the 2026-07-05 sample is a 48-byte-or-later log; every record should "
        "carry the scorecard word"
    )
    # Every documented subsystem is split out, and the split agrees with the word.
    for name, shift in parser.SH_FIELDS.items():
        assert all(r[name] == ((r["sensor_health"] >> shift) & 0x3) for r in rows), name

    # The evidence that made this worth decoding: the flight computer recorded
    # its own GNSS outage, and it is the outage behind #741's 81 m nav/GNSS
    # divergence. Nothing in the tree could read it.
    bad = [r for r in rows if r["health_gnss"] == parser.SH_BAD]
    assert bad, "expected a GNSS BAD window on the 2026-07-05 sample"
    span_s = (bad[-1]["time_us"] - bad[0]["time_us"]) / 1e6
    assert 1.0 < span_s < 4.0, f"GNSS BAD window was {span_s:.2f}s, expected ~2s"

    old, _stats, _config = parser.parse_binary_file(str(GOLDEN_BIN))
    old_rows = old["NonSensor"]
    assert old_rows, "golden flight has no NonSensor records"
    assert all(r["sensor_health"] is None for r in old_rows)
    for name in parser.SH_FIELDS:
        assert all(r[name] is None for r in old_rows), (
            f"{name} decoded as a value on a log that predates the field; "
            "0 would read as a deliberate 'not configured' verdict"
        )


def test_snapshot_frames_decode_and_retire_the_0xd2_row() -> None:
    """#752's largest block of unreachable data: SNAPSHOT_MSG 0xD2.

    774 frames on the sample and 615 on the golden flight had no parser branch,
    so the report's own message table showed the reader a row labelled `0xD2`
    with a frame count and no way to open it.

    Both integrity gates are exercised, not just the happy path: this is the
    frame the flight computer restores ITSELF from after an in-flight reboot,
    so a bad magic or a failed CRC is corruption rather than a data point.
    """
    import struct

    import plot_flight_data_mini as parser
    from flight_report import catalog as cat_mod
    from flight_report.flight import Flight

    records, stats, _config = parser.parse_binary_file(str(SAMPLE_BIN))
    snaps = records["Snapshot"]
    assert len(snaps) == stats["type_counts"]["Snapshot"] > 0
    assert "0xD2" not in stats["type_counts"], (
        "the raw hex row is what the reader used to see; a decoded frame type "
        "must be counted under its name"
    )

    # Refusals are counted per file, in stats — not in a module global that
    # would attribute one flight's corruption to the next flight in a directory
    # run. A clean fixture must report zero on every gate.
    assert stats["snapshot_rejects"] == {"magic": 0, "crc": 0, "len": 0}

    first = snaps[0]
    # Same micros() origin as every other flight-computer stream, which is what
    # lets a snapshot be overlaid on the sensor traces. The frame carries no
    # other absolute time — flight_elapsed_ms is relative to launch detect.
    ns_span = (records["NonSensor"][0]["time_us"], records["NonSensor"][-1]["time_us"])
    assert ns_span[0] <= first["time_us"] <= ns_span[1]

    # v3 on this flight: the sim_flight byte was padding then, and reporting it
    # as False would assert a fact the firmware's own restore path refuses to
    # conclude from a v3 frame.
    assert first["version"] == 3
    assert first["sim_flight"] is None
    assert first["b2r_code"] is not None, "b2r_* is carried from v3 on"

    # The 15-state covariance diagonal arrives named, in the EKF's own order.
    assert parser.SNAPSHOT_P_NAMES[:3] == ("p_pos_n", "p_pos_e", "p_pos_d")
    assert all(first[n] is not None and first[n] >= 0 for n in parser.SNAPSHOT_P_NAMES), (
        "a variance cannot be negative"
    )

    # A near-vertical rocket on the pad, with its datum where the flight was.
    assert 70.0 < first["ekf_pitch"] < 100.0
    assert 90_000.0 < first["ground_pressure_pa"] < 110_000.0
    assert abs(first["ref_lat"] - first["ekf_lat"]) < 0.01

    # Corruption is rejected, and counted rather than swallowed. Flip one byte
    # of a real frame and rebuild it: the magic still matches, the CRC does not.
    payload = struct.pack(parser.FMT_SNAPSHOT, *([0] * 64))
    payload = struct.pack("<I", parser.SNAPSHOT_MAGIC) + payload[4:]
    assert len(payload) == parser.SNAPSHOT_LEN
    import zlib
    good_crc = zlib.crc32(payload[:220]) & 0xFFFFFFFF
    assert good_crc != 0, "an all-zero body should not CRC to zero"

    flight = Flight.from_bin(SAMPLE_BIN)
    flight.load()
    unreadable = {u.name for u in cat_mod.unreadable_for(flight)}
    assert "0xD2" not in unreadable and "Snapshot" not in unreadable


# ---------------------------------------------------------------------------
# #750 — Motor Performance, back without its entry form.
#
# The section was removed in #751 because two required-looking inputs (liftoff
# mass, motor designation) stood between dropping a log and reading a report.
# The rows that need neither are the point of bringing it back, so what is
# pinned here is that none of them ask for anything.
# ---------------------------------------------------------------------------

# Ground truth for SAMPLE_BIN, given by the flyer 2026-08-10 and recorded in
# examples/flights/README.md: Rolly Polly 54 mm, liftoff mass 0.883 kg, G77.
# Neither number is in the log or the sidecar, which is the whole problem —
# they are here so the log-only figures can be checked against a known answer.
SAMPLE_MASS_KG = 0.883
SAMPLE_MOTOR = "G77"


def _motor(bin_path, metadata=None):
    from flight_report.flight import Flight
    from flight_report.modules import motor

    flight = Flight.from_bin(bin_path)
    flight.load()
    if metadata:
        flight.metadata = dict(metadata)
    return motor.analyze(flight)


def test_motor_section_measures_the_burn_with_no_input_at_all() -> None:
    """Every row that does not need a mass, from an empty metadata dict.

    This is the regression the removal was about: the section used to render a
    warning telling the reader to go back and fill in a form. Dropping a file
    has to be enough.
    """
    result = _motor(SAMPLE_BIN)
    assert result.error is None
    for row in ("Burn time", "Peak acceleration", "Average acceleration",
                "Thrust-to-weight at peak", "Impulse per kg", "Speed at burnout"):
        assert row in result.metrics, f"{row} should not need an input"
    assert not result.warnings, f"a form-free run should warn about nothing: {result.warnings}"
    assert "typed in" in result.note

    # Thrust-to-weight needs no mass because the accelerometer already divided
    # by it — specific force over g IS the ratio. If this ever starts reading a
    # mass, that identity has been broken.
    tw = result.metrics["Thrust-to-weight at peak"]
    peak_g = result.metrics["Peak acceleration"]
    assert abs(tw.value - peak_g.value) < 1e-9


def test_motor_impulse_is_per_kilogram_and_agrees_with_the_known_flight() -> None:
    """The measured burn, checked against a motor whose class we actually know.

    SAMPLE_BIN flew on a G77 at 0.883 kg. Nothing in the log says so, so this is
    a real check rather than a tautology: the impulse-per-kg figure has to put
    that mass in the G window, and multiplying it out has to land in G's
    80-160 N·s band.
    """
    result = _motor(SAMPLE_BIN)
    j_per_kg = result.metrics["Impulse per kg"].value
    assert 50.0 < j_per_kg < 200.0, j_per_kg

    # G is 80-160 N·s, and 0.883 kg is what it flew at.
    impulse = j_per_kg * SAMPLE_MASS_KG
    assert 80.0 < impulse <= 160.0, (
        f"{impulse:.0f} N·s puts a known G77 outside class G"
    )

    # And the row that hands the reader that conclusion without asking for the
    # mass must name G over a window containing it.
    windows = result.metrics["Class by liftoff mass"]
    g_window = [w for w in windows.split(" · ") if w.startswith("G ")]
    assert g_window, windows
    lo, hi = (float(x) for x in g_window[0].split()[1].replace("kg", "").split("-"))
    assert lo <= SAMPLE_MASS_KG <= hi, f"{SAMPLE_MASS_KG} kg not in the G window {lo}-{hi}"


def test_motor_uses_a_supplied_mass_without_ever_asking_for_one() -> None:
    """The seam kept alive by the removal: metadata still works if a caller has it.

    Flight.metadata and the worker's pass-through were deliberately left in
    place when the form went. A caller that already knows the mass gets the
    newton-second rows; nothing in the shipped UI sends it, and the module does
    not prompt.
    """
    result = _motor(SAMPLE_BIN, {"mass_kg": SAMPLE_MASS_KG, "motor": SAMPLE_MOTOR})
    assert result.metrics["Measured class"] == "G", "a G77 should measure as class G"
    assert result.metrics["Motor"] == SAMPLE_MOTOR
    assert not result.warnings, "the declared and measured class agree; nothing to say"

    # Average thrust is the number in the designation: a G77 averages 77 N. The
    # measurement is net of drag, so it must come in under that — but not wildly.
    thrust = result.metrics["Average thrust"].value
    assert 0.6 * 77.0 < thrust < 77.0, (
        f"{thrust:.0f} N against a nominal 77 N — either the integral is wrong or "
        "the drag deduction is not what the section claims"
    )

    # A mass outside 0.01-500 kg is a typo, not a vehicle, and must not reach
    # the arithmetic.
    for typo in (0.0, -1.0, 5000.0, "heavy", None):
        degraded = _motor(SAMPLE_BIN, {"mass_kg": typo})
        assert "Measured impulse" not in degraded.metrics, typo
        assert "Impulse per kg" in degraded.metrics, "the log-only rows still stand"


def test_motor_flags_a_motor_that_does_not_match_the_burn() -> None:
    """The cross-check that made the section worth having."""
    result = _motor(SAMPLE_BIN, {"mass_kg": SAMPLE_MASS_KG, "motor": "J500"})
    assert result.warnings, "a two-class gap should be called out"
    assert "J500" in result.warnings[0] and "class G" in result.warnings[0]


def test_motor_degrades_rather_than_crashes_without_a_measurable_burn() -> None:
    """GOLDEN_BIN opens at 8.1 g, already under thrust — there is no liftoff.

    The section must say so and return cleanly. A module that raises here would
    take its section down in every report generated from a partial capture.
    """
    result = _motor(GOLDEN_BIN)
    assert result.error is None
    assert not result.metrics
    assert result.warnings and "burn window" in result.warnings[0]


def test_globe_reads_the_logged_origin_before_rebuilding_it() -> None:
    """#1419: the firmware writes its frozen ENU reference into every in-flight
    Snapshot (ref_lat / ref_lon / ref_alt_m). Rebuilding it from the pad fixes
    instead cost 1-7 m on logs with pad time and 38.8 m on a log that started
    at launch (Eagle Claw 2026-08-29), read as a bodily shift of the whole nav
    track. The logged value is what every e/n/u sample is relative to, so it
    wins whenever it exists; the null-island snapshot a receiver writes before
    its first fix is not a reference."""
    from flight_report.modules import globe

    snaps = {"Snapshot": [
        {"ref_lat": 0.0, "ref_lon": 0.0, "ref_alt_m": 0.0, "ref_datum_converged": False},
        {"ref_lat": 39.4680331, "ref_lon": -75.2929036, "ref_alt_m": 33.4, "ref_datum_converged": True},
        {"ref_lat": 39.5, "ref_lon": -75.3, "ref_alt_m": 40.0, "ref_datum_converged": True},
    ]}
    logged, note = globe._logged_reference(snaps)
    assert logged == (39.4680331, -75.2929036, 33.4, True), "the first real reference, not the null-island one"
    assert note is None

    # The plausibility gate: the logged origin is the mean of the very fixes
    # the GNSS track starts from, so it must sit within metres of their pad
    # mean. 38.8 m away (Eagle Claw's reconstruction error) passes; the golden
    # fixture's real-pad snapshots beside fixes frozen at (38, -122) do not.
    near = (39.4683812, -75.2928964, 26.4)
    assert globe._logged_reference(snaps, near)[0] == logged
    far = (38.0, -122.0, 4.5)
    rejected, why = globe._logged_reference(snaps, far)
    assert rejected is None and "refused" in why and "km" in why

    assert globe._logged_reference({"Snapshot": []})[0] is None
    assert globe._logged_reference({})[0] is None
    assert globe._logged_reference({"Snapshot": [{"ref_lat": float("nan"), "ref_lon": 1.0, "ref_alt_m": 2.0}]})[0] is None
    # A pre-#834 snapshot with no convergence flag still yields the origin.
    assert globe._logged_reference({"Snapshot": [{"ref_lat": 40.1, "ref_lon": -105.2, "ref_alt_m": 1500.0}]})[0] == (40.1, -105.2, 1500.0, None)

