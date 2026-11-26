"""#1763: main.cpp counts a GNSS record as new GPS for the apogee and landing votes only with a usable fix.

The policy (flight_computer/main/gnss_fix_policy.h) and its effect on the real vote above the barometer's range are
host-tested in tests_cpp/test_gnss_fix_policy.cpp. main.cpp is not host-built, so these tests read the source:
- the kinematic checks' new_gps (gps_new_for_kc) is set from gnss_fix::usable() of the record just read, on every
  record, for every receiver: no `= true`, no preprocessor branch around it;
- both call sites of the checks pass that flag with the same record's velocity, and it is cleared once after them;
- GnssAscentGate admits GNSS on the same test, so the record that admits it and the records that vote meet one bar.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")

USABLE = "gnss_fix::usable(gnss_latest_si.fix_mode, gnss_latest_si.num_sats, config::GNSS_MIN_SATS)"


def read():
    with open(MAIN_CPP) as fh:
        return fh.read()


def code(text):
    """The source without // comments, whitespace runs collapsed to one space."""
    return re.sub(r"[ \t]*\n[ \t]*", "\n", "\n".join(line.split("//", 1)[0] for line in text.splitlines()))


def flat(text):
    return re.sub(r"\s+", " ", text)


def pp_stack(src, pos):
    """The preprocessor conditions open at pos: each #if/#ifdef/#ifndef line, '!'-prefixed in its #else/#elif."""
    stack = []
    for line in src[:pos].splitlines():
        t = line.strip()
        if re.match(r"#\s*if", t):
            stack.append(t)
        elif re.match(r"#\s*(else|elif)", t):
            stack[-1] = "!" + stack[-1].lstrip("!")
        elif re.match(r"#\s*endif", t):
            stack.pop()
    return stack


def receiver_branches(src, pos):
    """The open conditions at pos that name one receiver's image (or the one image's choice)."""
    return [c for c in pp_stack(src, pos)
            if re.search(r"TR_RAW_NAV_(SDR|MOSAIC|PX1105R|AUTO)|TR_GNSS_(SDR|DRIVER|AUTO)|RAW_IS_|RAW_ON", c)]


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


def test_the_policy_is_included_for_every_image():
    src = code(read())
    i = src.index('#include "gnss_fix_policy.h"')
    assert receiver_branches(src, i) == []


def test_a_record_is_new_gps_only_with_a_usable_fix():
    src = code(read())
    sets = [m for m in re.finditer(r"(?<!bool )\bgps_new_for_kc\s*=(?!=)\s*([^;]+);", src)]   # not the declaration
    values = [flat(m.group(1)).strip() for m in sets]
    # one set from the record, one reset after the checks read it; never a bare true
    assert "true" not in values, "a record is counted as new GPS with no fix test (#1763)"
    assert sorted(values) == sorted([flat(USABLE), "false"]), values
    m = next(m for m in sets if flat(m.group(1)).strip() == flat(USABLE))
    # inside the read of a new record, right after it is converted: the test reads the record just read
    read_at = src.rindex("if (sensor_collector.getGNSSData(gnss_data))", 0, m.start())
    block = src[read_at:m.start()]
    assert "sensor_converter.convertGNSSData(gnss_data, gnss_latest_si);" in block
    assert block.count("{") - block.count("}") == 1, "the set is not inside the new-record block"
    assert receiver_branches(src, m.start()) == [], receiver_branches(src, m.start())


def test_the_flag_is_cleared_once_the_checks_have_read_it():
    """The reset comes after both calls of the checks, at the level of the MAG_CALIBRATION skip around them: a
    reset before them (or inside a branch that skips them) would throw every fix away before it voted."""
    src = code(read())
    resets = [m.start() for m in re.finditer(r"(?<!bool )\bgps_new_for_kc\s*=\s*false\s*;", src)]
    assert len(resets) == 1, resets
    reset = resets[0]
    last_call = max(m.start() for m in re.finditer(r"kinematics\.kinematicChecks\(", src))
    assert reset > last_call, "the flag is cleared before the checks read it"
    skip = src.rindex("if (rocket_state != MAG_CALIBRATION)", 0, last_call)
    between = src[skip:reset]
    assert between.count("{") == between.count("}"), "the reset is not at the level of the checks' block"
    assert re.search(r"\bbmp_new_for_kf\s*=\s*false;\s*$", src[:reset]), "the reset is not beside the barometer's"
    assert receiver_branches(src, reset) == []


def test_both_calls_of_the_checks_vote_with_that_flag_and_the_same_records_velocity():
    src = code(read())
    calls = [m.start() for m in re.finditer(r"kinematics\.kinematicChecks\(", src)]
    assert len(calls) == 2, len(calls)
    for i in calls:
        args = call_args_at(src, i + len("kinematics."), "kinematicChecks")
        assert args[7] == "gps_new_for_kc", args[7]                       # new_gps
        assert args[11] == "(float)gnss_latest_si.vel_u", args[11]       # gps_vel_u


def test_gnss_is_admitted_on_the_same_test_it_votes_on():
    src = code(read())
    m = re.search(r"\bga\.gnss_fix_ok\s*=\s*([^;]+);", src)
    assert m, "GnssAscentGate's gnss_fix_ok not found"
    rhs = flat(m.group(1))
    assert flat(USABLE) in rhs, rhs
    assert "fix_mode >=" not in rhs and "num_sats >=" not in rhs, "a second copy of the test (#1763)"
