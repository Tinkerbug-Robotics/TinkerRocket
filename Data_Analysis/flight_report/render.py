"""Render a per-flight HTML report from AnalysisResults."""

from __future__ import annotations

import base64
import gzip
import io
import json
from functools import lru_cache
from pathlib import Path
from typing import TYPE_CHECKING

import matplotlib.pyplot as plt
import numpy as np
from jinja2 import Environment, FileSystemLoader
from markupsafe import Markup

from .charts import TIME_DECIMALS

if TYPE_CHECKING:
    import matplotlib.figure
    from .flight import Flight
    from .registry import AnalysisResult


_TEMPLATE_DIR = Path(__file__).resolve().parent / "templates"
_VENDOR_DIR = Path(__file__).resolve().parent / "vendor"
_STATIC_DIR = Path(__file__).resolve().parent / "static"

# Charts smaller than this stay as plain JSON; see _chart_payload.
_GZIP_THRESHOLD_BYTES = 64 * 1024

# x axes ship as integer steps in units of 1/_STEP_DIV; see _pack_axis.
_STEP_DIV = 10 ** TIME_DECIMALS

_env = Environment(
    loader=FileSystemLoader(str(_TEMPLATE_DIR)),
    # Not select_autoescape(["html"]): it matches on extension, and the template
    # is report.html.j2, so escaping silently stayed off.
    autoescape=True,
    trim_blocks=True,
    lstrip_blocks=True,
)


def _fig_to_data_uri(fig: "matplotlib.figure.Figure", dpi: int = 110) -> str:
    """Encode a matplotlib Figure as a `data:image/png;base64,...` URI."""
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=dpi, bbox_inches="tight")
    buf.seek(0)
    b64 = base64.b64encode(buf.read()).decode("ascii")
    plt.close(fig)
    return f"data:image/png;base64,{b64}"


def _chart_payload(chart: dict) -> tuple[Markup, bool]:
    """Serialize a chart spec for embedding. Returns (text, is_gzipped).

    Full-resolution sample arrays are bulky as JSON text — a 154k-frame flight
    carries ~620k points, about 9 MB of digits and commas. Numeric JSON gzips
    roughly 5x, and browsers can inflate it themselves via DecompressionStream,
    so large charts ship compressed and are inflated at load. Small charts stay
    plain text: base64 costs 33% and gzip has a floor, so compressing them would
    make them bigger, and plain text keeps the payload greppable.

    Escaping `<` keeps a stray "</script>" in any label from ending the block
    early; the result is marked safe because it is JSON, not user HTML.
    """
    raw = json.dumps(chart, allow_nan=False, separators=(",", ":"))
    if len(raw) < _GZIP_THRESHOLD_BYTES:
        return Markup(raw.replace("<", "\\u003c")), False
    packed = base64.b64encode(gzip.compress(raw.encode("utf-8"), 6)).decode("ascii")
    return Markup(packed), True


def _pack_axis(values) -> list[int] | None:
    """`values` as integers in units of 1/_STEP_DIV: the first absolute, each
    next one the step from its predecessor. None when `values` is not a plain
    numeric list, so a text axis would ship as it is rather than fail the render.

    A size trick, not a precision one. Time is exact at six decimals
    (charts.TIME_DECIMALS), so microsecond integers lose nothing. But a stream
    logs at a near-constant rate, which makes consecutive steps near-identical
    short strings — "264,265,263,264" — that gzip folds to almost nothing,
    where absolute values are six fresh digits every sample and barely
    compress. On a 3.84 kHz V9 flight the chart payloads halved (15.4 MB to
    7.7 MB), to below what they weighed at 1 ms resolution. The template's
    readSpec rebuilds the absolute array on load, so nothing past the decode
    ever sees steps; tests/test_flight_report.py mirrors it.
    """
    arr = np.asarray(values)
    if arr.ndim != 1 or arr.size == 0 or arr.dtype.kind not in "iuf":
        return None
    q = np.rint(arr.astype(float) * _STEP_DIV).astype(np.int64)
    return np.diff(q, prepend=np.int64(0)).tolist()


def _pack_chart(chart: dict) -> dict:
    """The chart with each trace's x as steps under `xd`; see _pack_axis.

    3D charts go through untouched: their x is one of three equal axes the
    template plots straight through, and they carry only the flight itself.
    """
    if chart.get("kind") == "3d":
        return chart
    traces = []
    for t in chart.get("traces", []):
        steps = _pack_axis(t.get("x"))
        if steps is None:
            traces.append(t)
            continue
        traces.append({**{k: v for k, v in t.items() if k != "x"}, "xd": steps})
    return {**chart, "traces": traces, "stepDiv": _STEP_DIV}


def _pack_dataset(d: dict) -> dict:
    """The Explore payload with each stream's t as steps under `td`; see _pack_axis."""
    streams = {}
    for name, stream in (d.get("streams") or {}).items():
        steps = _pack_axis(stream.get("t"))
        if steps is None:
            streams[name] = stream
            continue
        streams[name] = {**{k: v for k, v in stream.items() if k != "t"}, "td": steps}
    return {**d, "streams": streams, "stepDiv": _STEP_DIV}


def _dataset_entry(d: dict) -> dict:
    """Panel descriptor for the template: payload for an owner, id for a ref.

    The X-Y panel points at the payload the time panel already embedded. It is
    ~3.9 MB gzipped on a real flight, and shipping it twice would double the
    document to save one dictionary lookup in the browser.
    """
    entry = {"id": d["id"], "panel": d.get("panel", "time"), "ref": bool(d.get("ref"))}
    if entry["ref"]:
        return entry
    payload, gz = _chart_payload(_pack_dataset({k: v for k, v in d.items() if k != "panel"}))
    entry["payload"] = payload
    entry["gzipped"] = gz
    return entry


def _flatten_settings(node, prefix: str = "") -> list[tuple[str, str]]:
    """Nested settings dict -> flat (dotted key, display value) pairs, sorted.

    The sidecar nests a few levels deep (`pyro.ch1.trigger_mode`), which as raw
    JSON is a wall of braces. Flattening to dotted keys makes it a table you can
    scan and diff against another flight.
    """
    rows: list[tuple[str, str]] = []
    if isinstance(node, dict):
        for k, v in sorted(node.items()):
            rows.extend(_flatten_settings(v, f"{prefix}.{k}" if prefix else str(k)))
    elif isinstance(node, list):
        if node and all(isinstance(v, (int, float, str, bool)) or v is None for v in node):
            rows.append((prefix, ", ".join("null" if v is None else str(v) for v in node)))
        else:
            for i, v in enumerate(node):
                rows.extend(_flatten_settings(v, f"{prefix}[{i}]"))
    elif isinstance(node, bool):
        rows.append((prefix, "yes" if node else "no"))
    elif node is None:
        rows.append((prefix, "—"))
    else:
        rows.append((prefix, str(node)))
    return rows


def _settings_groups(sidecar: dict) -> list[dict]:
    """Settings only, grouped by top-level key. Flight performance is excluded.

    The sidecar mixes configuration with results (apogee_time_s, max_altitude_m
    and friends). Those are the summary card's job; repeating them here as raw
    JSON invites the two to disagree.
    """
    settings = sidecar.get("settings")
    if not isinstance(settings, dict):
        return []

    groups = []
    scalars: list[tuple[str, str]] = []
    for key, value in sorted(settings.items()):
        if isinstance(value, (dict, list)):
            rows = _flatten_settings(value)
            if rows:
                groups.append({"name": key, "rows": rows})
        else:
            scalars.extend(_flatten_settings(value, key))
    if scalars:
        groups.insert(0, {"name": "general", "rows": scalars})
    return groups


@lru_cache(maxsize=8)
def _read_vendor(name: str) -> str:
    """A vendored asset, or '' if it isn't present (that feature then degrades).

    Cached per process: plotly alone is 1.6 MB and the CLI renders both levels
    from a single run, so re-reading per report is pure cost.
    """
    try:
        return (_VENDOR_DIR / name).read_text(encoding="utf-8")
    except OSError:
        return ""


@lru_cache(maxsize=8)
def _read_static(name: str) -> str:
    """Our own browser-side source, inlined so the report stays one file.

    Separate from `vendor/` because this is code we maintain and test — see
    tests/js/. Vendored bundles are third-party and are never edited.
    """
    return (_STATIC_DIR / name).read_text(encoding="utf-8")


def _plotly_source() -> str:
    """The vendored plotly.js, or '' if it isn't present (charts then degrade)."""
    return _read_vendor("plotly-gl3d.min.js")


def render_report(
    flight: "Flight",
    results: list["AnalysisResult"],
    level: str | None = None,
) -> str:
    """Render one report as an HTML string.

    `level` is `registry.LEVEL_FLIGHT` and only labels the
    output — pass the results you want in it. None renders an untitled report
    containing everything given.

    """
    from .cesium_bundle import cesium_css, cesium_source
    from .registry import LEVEL_FLIGHT
    from .summary import compute_summary

    template = _env.get_template("report.html.j2")

    sections = []
    for r in results:
        sections.append({
            "name": r.name,
            "title": r.title,
            "metrics": r.metrics,
            "metric_headers": r.metric_headers,
            "figures": [_fig_to_data_uri(f) for f in r.figures],
            "charts": [
                {**c, "payload": payload, "gzipped": gz}
                for c in r.charts
                for payload, gz in [_chart_payload(_pack_chart(c))]
            ],
            "maps": [
                {**m, "payload": payload, "gzipped": gz}
                for m in r.maps
                for payload, gz in [_chart_payload(m)]
            ],
            "globes": [
                {**g, "payload": payload, "gzipped": gz}
                for g in r.globes
                for payload, gz in [_chart_payload(g)]
            ],
            "datasets": [_dataset_entry(d) for d in r.datasets],
            "warnings": r.warnings,
            "text": r.text,
            "error": r.error,
        })

    has_charts = any(s["charts"] for s in sections)
    has_datasets = any(s["datasets"] for s in sections)
    has_maps = any(s["maps"] for s in sections)
    has_globe = any(s["globes"] for s in sections)
    is_flight = level == LEVEL_FLIGHT

    # cesium_source() raises if the vendored bundle stopped matching its file://
    # worker patch — but the globe module calls it first and drops the section on
    # failure, so by the time has_globe is true the call is a cache hit that
    # cannot raise. Keep that ordering if either side is refactored.
    cesium_js = Markup(cesium_source()) if has_globe else ""

    return template.render(
        flight=flight,
        date_str=flight.date.strftime("%Y-%m-%d %H:%M:%S") if flight.date else "unknown",
        duration_s=flight.duration_s,
        sidecar=flight.sidecar,
        settings_groups=_settings_groups(flight.sidecar or {}),
        stats=flight.stats,
        config=flight.config,
        sections=sections,
        level=level,
        # The flight report leads with the headline numbers; the detailed
        # report leads with parser/settings detail and doesn't repeat them.
        summary=compute_summary(flight) if is_flight else [],
        has_charts=has_charts,
        plotly_js=Markup(_plotly_source()) if has_charts else "",
        has_datasets=has_datasets,
        explore_js=Markup(_read_static("explore.js")) if has_datasets else "",
        has_maps=has_maps,
        leaflet_js=Markup(_read_vendor("leaflet.js")) if has_maps else "",
        leaflet_css=Markup(_read_vendor("leaflet.css")) if has_maps else "",
        has_globe=has_globe,
        cesium_js=cesium_js,
        cesium_css=Markup(cesium_css()) if has_globe else "",
        # Mutable so the template can record that it has already printed the
        # chart instructions; Jinja's own scoping would reset a plain flag on
        # each loop iteration.
        chart_hint_shown={"done": False},
    )


def write_report(
    flight: "Flight",
    results: list["AnalysisResult"],
    out_path: Path,
    level: str | None = None,
) -> Path:
    """Render and write to disk. Returns the written path."""
    html = render_report(flight, results, level=level)
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(html, encoding="utf-8")
    return out_path
