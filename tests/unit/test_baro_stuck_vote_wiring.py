"""#1764: main.cpp hands the apogee vote GnssAscentGate's latched verdict on the barometer.

The latch (GnssAscentGate::baroVotes) is host-tested in tests_cpp/test_gnss_ascent_gate.cpp, and the vote with it in
tests_cpp/test_baro_stuck_vote.cpp (the real TR_KinematicChecks and GnssAscentGate in a sealed-port flight). main.cpp
is not host-built, so these tests read the source:
- both calls of the checks (the raw-filter image's and every other image's copy) pass baroVotes() as baro_may_vote,
  and still pass the raw baro_healthy, which the landing tests and the Layer-2 backstop read;
- the replay harness (tools/raw_replay/flight_votes.h) passes the same.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")
FLIGHT_VOTES = os.path.join(ROOT, "tools", "raw_replay", "flight_votes.h")


def code(path):
    with open(path) as fh:
        return "\n".join(line.split("//", 1)[0] for line in fh.read().splitlines())


def flat(text):
    return re.sub(r"\s+", " ", text)


def call_args_at(src, i, name):
    """The argument list of the call to name starting at index i, split at its top-level commas."""
    j = i + len(name) + 1
    depth, start, args = 1, j, []
    while depth:
        c = src[j]
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        if (c == "," and depth == 1) or depth == 0:
            args.append(flat(src[start:j]).strip())
            start = j + 1
        j += 1
    return args


def calls(src, prefix):
    return [call_args_at(src, m.start() + len(prefix), "kinematicChecks")
            for m in re.finditer(re.escape(prefix) + r"kinematicChecks\(", src)]


def test_both_calls_of_the_checks_take_the_latched_verdict():
    found = calls(code(MAIN_CPP), "kinematics.")
    assert len(found) == 2, len(found)
    for args in found:
        assert len(args) == 19, args
        assert args[13] == "baro_healthy", args[13]                                   # landing, backstop
        assert args[17] == "&inertial_vote", args[17]
        assert args[18] == "GnssAscentGate::baroVotes(gnss_admit_state)", args[18]    # baro_may_vote


def test_the_replay_takes_it_too():
    found = calls(code(FLIGHT_VOTES), "kin_.")
    assert len(found) == 1, len(found)
    args = found[0]
    assert len(args) == 19, args
    assert args[13] == "baro_healthy", args[13]
    assert args[18] == "GnssAscentGate::baroVotes(gate_)", args[18]
