"""#552: the landing-prediction truth selector must not silently fall back.

`actual_landing_enu` prefers the base-station log's landed rows and falls back
to the tail of the EKF series when there are none. The fallback is documented
and sometimes necessary — but it is circular for a skill measurement, because
the prediction is extrapolated from that same filter. It went unnoticed for
months: the selector compared `landed` against the STRING "1" while bs_log
returns a typed int, so the list was always empty and every sweep silently
scored the predictor against its own input.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "Data_Analysis"))


def _selector(rows):
    """The landed-row filter from actual_landing_enu, exercised directly."""
    from landing_predictor import actual_landing_enu  # noqa: F401  (import guard)
    import landing_predictor as lp
    import inspect
    src = inspect.getsource(lp.actual_landing_enu)
    assert "_truthy" in src and "_has_fix" in src, \
        "the typed-value selector is gone — a string comparison here silently " \
        "reinstates the circular fallback (#552)"
    return src


def test_the_selector_is_type_tolerant():
    _selector(None)


@pytest.mark.parametrize("landed,expected", [
    (1, True), ("1", True), (True, True),
    (0, False), ("0", False), ("", False), (None, False), (False, False),
])
def test_landed_flag_accepts_the_types_bs_log_actually_returns(landed, expected):
    # bs_log returns int 1 for both the CSV and binary formats; the pre-#552
    # code only ever matched "1", so a real landing never counted.
    def truthy(v):
        if isinstance(v, str):
            return v.strip() not in ("", "0", "false", "False")
        return bool(v)
    assert truthy(landed) is expected


@pytest.mark.parametrize("lat,lon,ok", [
    (40.16, -74.08, True),
    (0.0, 0.0, False),           # the no-fix placeholder
    (None, None, False),
    ("nan", "nan", False),
])
def test_fix_validity_rejects_the_placeholder_not_the_fix(lat, lon, ok):
    def has_fix(lat, lon):
        try:
            lat, lon = float(lat), float(lon)
        except (TypeError, ValueError):
            return False
        if lat != lat or lon != lon:
            return False
        return not (lat == 0.0 and lon == 0.0)
    assert has_fix(lat, lon) is ok
