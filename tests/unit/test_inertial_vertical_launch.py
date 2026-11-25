"""#1752: the IMU's own vertical estimate (InertialVertical) must be launched on every flight computer image.

APOGEE_VOTE_BY_SENSOR (config.h, every board) drops the navigation filter's velocity and pitch apogee votes in favour of
the accelerometer's and gyro's, which read InertialVertical. It flies only once enterInflight() calls launch(); until
then healthy() is false and neither sensor votes. The call used to sit inside `#if TR_RAW_NAV_P4`, so every image
without that macro (the Beetle V1, the V9 and V10 auto images, the legacy boards) flew with two of the four voters
missing and nothing in the log to say so. main.cpp is not host-built, so these tests read the source: one launch()
call, in enterInflight(), under no preprocessor condition the function itself is not under, and the IMU update and
the vote's inputs likewise built on every image.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
FC_MAIN = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main")
MAIN_CPP = os.path.join(FC_MAIN, "main.cpp")
CONFIG_H = os.path.join(FC_MAIN, "config.h")


def read(path):
    with open(path) as fh:
        return fh.read()


def conditions_at(lines, index):
    """The stack of open preprocessor conditions just before lines[index], outermost first."""
    stack = []
    for line in lines[:index]:
        s = line.strip()
        if re.match(r"#\s*if(n?def)?\b", s):
            stack.append(s)
        elif re.match(r"#\s*(else|elif)\b", s):
            stack[-1] = stack[-1] + " / " + s
        elif re.match(r"#\s*endif\b", s):
            stack.pop()
    return stack


def line_of(lines, pattern, start=0):
    for i in range(start, len(lines)):
        if re.search(pattern, lines[i]):
            return i
    raise AssertionError("no line matches %r" % pattern)


def test_one_launch_call_in_enter_inflight_under_no_condition_of_its_own():
    lines = read(MAIN_CPP).splitlines()
    calls = [i for i, l in enumerate(lines) if "inertial_vert.launch(" in l and not l.strip().startswith("//")]
    assert len(calls) == 1, "expected one inertial_vert.launch() call, found %d" % len(calls)
    fn = line_of(lines, r"^static void enterInflight\(")
    nxt = line_of(lines, r"^(static )?[A-Za-z_][\w:<>,\s\*&]*\s[\w:]+\([^;]*$", fn + 1)
    assert fn < calls[0] < nxt, "inertial_vert.launch() is not in enterInflight()"
    assert conditions_at(lines, calls[0]) == conditions_at(lines, fn), (
        "inertial_vert.launch() sits under %s: an image without it flies with no accelerometer or gyro apogee vote"
        % conditions_at(lines, calls[0])[len(conditions_at(lines, fn)):])


def test_the_imu_update_and_the_vote_inputs_are_under_no_condition_the_launch_is_not():
    lines = read(MAIN_CPP).splitlines()
    launch = conditions_at(lines, line_of(lines, r"^\s*inertial_vert\.launch\("))
    for pattern in (r"^\s*inertial_vert\.update\(", r"inertial_vote\.in_use\s*=", r"inertial_vote\.healthy\s*="):
        at = conditions_at(lines, line_of(lines, pattern))
        extra = [c for c in at if c not in launch]
        assert not extra, "%s is under %s" % (pattern, extra)


def test_the_vote_by_sensor_is_set_for_every_board():
    lines = read(CONFIG_H).splitlines()
    i = line_of(lines, r"static constexpr bool\s+APOGEE_VOTE_BY_SENSOR\s*=\s*true;")
    assert not [c for c in conditions_at(lines, i) if "TR_BOARD" in c or "TR_RAW_NAV" in c], (
        "APOGEE_VOTE_BY_SENSOR is set per board: check every board still launches InertialVertical")
