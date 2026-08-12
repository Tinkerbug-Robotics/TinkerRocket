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

DETAILED_SECTIONS = [
    'id="kinematics"',
    'id="gaps"',
    'id="timestamps"',
    'id="launch_detection"',
    'id="pyro_apogee"',
    'id="sensor_noise"',
    'id="gnss_staleness"',
    'id="kinematic_checks"',
    'id="roll_pid"',
    'id="lora"',
    'id="log_buffer"',
]

# The consolidation is moving the report towards a single page, so sections keep
# arriving here from the detailed side. Settings, the log contents and the
# deployment write-up moved first; the detailed report keeps only what has not
# been rebuilt as interactive charts yet.
FLIGHT_SECTIONS = [
    'id="overview"',
    'id="globe"',
    'id="rocket_state"',
    'id="deployment"',
    'id="barometer"',
    'id="parser_stats"',
    'id="timing"',
    'id="settings"',
]


@pytest.fixture(scope="module")
def reports(tmp_path_factory: pytest.TempPathFactory) -> dict[str, Path]:
    """Run the suite once against the golden flight; yield both report levels."""
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

    out = {
        "flight": tmp_path / f"{GOLDEN_BIN.stem}_report.html",
        "detailed": tmp_path / f"{GOLDEN_BIN.stem}_report_detailed.html",
    }
    for level, path in out.items():
        assert path.exists(), f"{level} report not written to {path}"
        assert path.stat().st_size > 100_000, f"{level} report suspiciously small"
    return out


@pytest.fixture(scope="module")
def report_html(reports: dict[str, Path]) -> Path:
    """The flight-level report — the one a flyer opens."""
    return reports["flight"]


def test_detailed_report_renders_all_sections(reports: dict[str, Path]) -> None:
    html = reports["detailed"].read_text(encoding="utf-8")
    missing = [s for s in DETAILED_SECTIONS if s not in html]
    assert not missing, f"Missing sections: {missing}"


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


def test_report_has_inline_figures(reports: dict[str, Path]) -> None:
    """Static figures are the detailed report's idiom; the flight report has one.

    The per-sensor timing histogram is the exception, and a deliberate one: it is
    a distribution rather than a series, nothing zooms into it, and rebuilding it
    as a Plotly figure would cost more than it returns. Every other flight-level
    visual is an interactive chart, so the count is pinned rather than merely
    bounded — a second PNG appearing here should be a decision, not a drift.
    """
    eng = reports["detailed"].read_text(encoding="utf-8")
    fig_count = eng.count('<img src="data:image/png;base64,')
    assert fig_count >= 25, f"Expected ≥25 inline figures, got {fig_count}"

    flight = reports["flight"].read_text(encoding="utf-8")
    flight_figs = flight.count('<img src="data:image/png;base64,')
    assert flight_figs == 1, (
        f"Flight report should carry exactly one static PNG (per-sensor timing), "
        f"got {flight_figs}"
    )


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
        out.append((chart_id, json.loads(raw)))
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


def test_plotly_trajectory_lives_in_detailed(reports: dict[str, Path]) -> None:
    """The 3D Plotly path moved to the detailed report when the globe took over the flight report.

    It is kept there rather than deleted because it survives what the globe does
    not: a report opened with no network, and a flight with no GNSS fix at all.
    """
    flight = dict(_chart_specs(reports["flight"].read_text(encoding="utf-8")))
    eng = dict(_chart_specs(reports["detailed"].read_text(encoding="utf-8")))
    assert "chart-trajectory" not in flight, (
        "the flight report shows both a Cesium globe and a Plotly 3D path of the "
        "same trajectory"
    )
    assert "chart-trajectory" in eng, "the 3D trajectory chart vanished instead of moving"


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


def test_report_has_no_module_errors(report_html: Path) -> None:
    """Every analysis module should run to completion (warnings OK, errors not)."""
    html = report_html.read_text(encoding="utf-8")
    # `class="err"` blocks only render when a module raised
    assert 'class="err"' not in html, (
        "One or more analysis modules raised an exception; "
        "open the report to inspect the traceback."
    )
