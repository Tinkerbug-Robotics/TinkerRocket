"""#1763: the offline copies of the apogee and landing votes count a GNSS record as new GPS only with a usable fix,
as main.cpp does (flight_computer/main/gnss_fix_policy.h).

- Data_Analysis/sim_kinematic_checks.py's gnss_fix_usable() is the policy's test (its truth table here is the one
  tests_cpp/test_gnss_fix_policy.cpp pins on the C++);
- the two Python replays that drive it (replay_kinematic_checks.py and the flight report's kinematic_checks module)
  set new_gps from it on every record, never to True.
The C++ replay (tools/raw_replay/flight_votes.h) is tested in tests_cpp/test_raw_replay_votes.cpp
(ANoFixRecordIsNoVoter).
"""
import os
import re
import sys

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
DA = os.path.join(ROOT, "Data_Analysis")
sys.path.insert(0, DA)

from sim_kinematic_checks import GNSS_MIN_SATS, gnss_fix_usable  # noqa: E402

MIRRORS = [
    os.path.join(DA, "replay_kinematic_checks.py"),
    os.path.join(DA, "flight_report", "modules", "kinematic_checks.py"),
]


def code(path):
    with open(path) as fh:
        return "\n".join(line.split("#", 1)[0] for line in fh.read().splitlines())


def test_the_python_test_is_the_policys():
    assert GNSS_MIN_SATS == 4   # config::GNSS_MIN_SATS
    assert gnss_fix_usable(3, 4)
    assert gnss_fix_usable(3, 25)
    assert gnss_fix_usable(4, 12)
    assert not gnss_fix_usable(3, 3)
    assert not gnss_fix_usable(2, 12)
    assert not gnss_fix_usable(1, 12)
    assert not gnss_fix_usable(0, 0)
    assert not gnss_fix_usable(0, 7)


def test_the_replays_count_a_record_only_with_a_usable_fix():
    for path in MIRRORS:
        src = code(path)
        sets = [s.strip() for s in re.findall(r"^\s*new_gps\s*=(?!=)\s*([^\n]+)", src, re.M)]
        sets = [v for v in sets if not v.endswith(",")]   # statements, not a call's new_gps=new_gps,
        assert "True" not in sets, f"{os.path.basename(path)} counts every record as new GPS (#1763)"
        assert 'gnss_fix_usable(r["fix_mode"], r["num_sats"])' in sets, (path, sets)
        assert set(sets) <= {"False", 'gnss_fix_usable(r["fix_mode"], r["num_sats"])'}, (path, sets)
