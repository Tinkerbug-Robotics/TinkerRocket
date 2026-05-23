"""Smoke test for the flight_report suite (issue #184).

Runs the full suite against the canonical golden flight checked into
`tests/test_data/`; fails if the report doesn't render or any required
section is missing.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
GOLDEN_BIN = REPO_ROOT / "tests" / "test_data" / "flight_20260520_173339.bin"

REQUIRED_SECTIONS = [
    'id="kinematics"',
    'id="gaps"',
    'id="timestamps"',
    'id="launch_detection"',
    'id="pyro_apogee"',
    'id="sensor_noise"',
    'id="gnss_staleness"',
    'id="kinematic_checks"',
    'id="settings"',
    'id="parser"',
]


@pytest.fixture
def report_html(tmp_path: Path) -> Path:
    """Run the suite against the golden flight; yield path to report.html."""
    if not GOLDEN_BIN.exists():
        pytest.skip(f"Golden flight missing: {GOLDEN_BIN}")

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

    report = tmp_path / f"{GOLDEN_BIN.stem}_report.html"
    assert report.exists(), f"Report not written to {report}"
    assert report.stat().st_size > 100_000, "Report too small — likely missing figures"
    return report


def test_report_renders_all_sections(report_html: Path) -> None:
    html = report_html.read_text(encoding="utf-8")
    missing = [s for s in REQUIRED_SECTIONS if s not in html]
    assert not missing, f"Missing sections: {missing}"


def test_report_has_inline_figures(report_html: Path) -> None:
    html = report_html.read_text(encoding="utf-8")
    fig_count = html.count('<img src="data:image/png;base64,')
    assert fig_count >= 25, f"Expected ≥25 inline figures, got {fig_count}"


def test_report_has_no_module_errors(report_html: Path) -> None:
    """Every analysis module should run to completion (warnings OK, errors not)."""
    html = report_html.read_text(encoding="utf-8")
    # `class="err"` blocks only render when a module raised
    assert 'class="err"' not in html, (
        "One or more analysis modules raised an exception; "
        "open the report to inspect the traceback."
    )
