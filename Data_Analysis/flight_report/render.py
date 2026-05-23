"""Render a per-flight HTML report from AnalysisResults."""

from __future__ import annotations

import base64
import io
from pathlib import Path
from typing import TYPE_CHECKING

import matplotlib.pyplot as plt
from jinja2 import Environment, FileSystemLoader, select_autoescape

if TYPE_CHECKING:
    import matplotlib.figure
    from .flight import Flight
    from .registry import AnalysisResult


_TEMPLATE_DIR = Path(__file__).resolve().parent / "templates"

_env = Environment(
    loader=FileSystemLoader(str(_TEMPLATE_DIR)),
    autoescape=select_autoescape(["html"]),
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


def render_report(flight: "Flight", results: list["AnalysisResult"]) -> str:
    """Render the full HTML report as a string."""
    template = _env.get_template("report.html.j2")

    sections = []
    for r in results:
        sections.append({
            "name": r.name,
            "title": r.title,
            "metrics": r.metrics,
            "figures": [_fig_to_data_uri(f) for f in r.figures],
            "warnings": r.warnings,
            "text": r.text,
            "error": r.error,
        })

    return template.render(
        flight=flight,
        date_str=flight.date.strftime("%Y-%m-%d %H:%M:%S") if flight.date else "unknown",
        duration_s=flight.duration_s,
        sidecar=flight.sidecar,
        stats=flight.stats,
        config=flight.config,
        sections=sections,
    )


def write_report(flight: "Flight", results: list["AnalysisResult"], out_path: Path) -> Path:
    """Render and write to disk. Returns the written path."""
    html = render_report(flight, results)
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(html, encoding="utf-8")
    return out_path
