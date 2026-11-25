"""#1755: main.cpp's wiring of the boost thrust-axis check (thrust_axis_policy.h).

An AUTO-mounted board can fly a wrong board-to-rocket frame. The thrust-axis check measures the real thrust direction
in the first 150 ms; ThrustAxisPolicy (host-tested in tests_cpp/test_orientation_sway.cpp) decides whether it is
upward thrust that disproves the frame. main.cpp then has to act on that verdict in four places: burnout reads the
measured axis (held, boundedly, while the check measures), the IMU's pitch apogee voter stands down, roll control
stands down, and every launch clears the verdict. After a reboot the verdict comes back from the OC's snapshot, and
every reader of the roll rate gets it about the real axis. main.cpp is not host-built, so these tests read the source.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")


def read():
    with open(MAIN_CPP) as fh:
        return fh.read()


def call_args(src, name):
    """The text of the first call `name(...)` (balanced parentheses), arguments only."""
    i = src.index(name + "(")
    j = i + len(name) + 1
    depth = 1
    while depth:
        c = src[j]
        depth += (c == "(") - (c == ")")
        j += 1
    return src[i + len(name) + 1:j - 1]


def statement(src, start_pattern):
    """From the first match of start_pattern to the next ';' at nesting depth 0."""
    m = re.search(start_pattern, src)
    assert m, "no match for %r" % start_pattern
    j, depth = m.start(), 0
    while True:
        c = src[j]
        depth += (c in "([{") - (c in ")]}")
        if c == ";" and depth <= 0:
            return src[m.start():j + 1]
        j += 1


def test_the_verdict_comes_from_the_policy_with_the_imus_attitude_and_the_mounting():
    src = read()
    assert '#include "thrust_axis_policy.h"' in src
    args = call_args(src, "ThrustAxisPolicy::judge")
    for needed in ("inertial_vert.healthy()", "inertial_vert.upOf(", "b2r_active_mode == ORIENT_MODE_MANUAL"):
        assert needed in args, "judge() is not given %s" % needed
    # The frame is disproved only on the policy's say-so: the boost verdict, or
    # the snapshot's after a reboot.
    sets = [m.start() for m in re.finditer(r"orient_frame_disproved\s*=\s*true", src)]
    assert len(sets) == 2
    guards = [src[i - 700:i] for i in sets]
    assert any("if (v.disproved)" in g for g in guards)
    assert any("if (ThrustAxisPolicy::restoredVerdict(snap.b2r_mode" in g for g in guards)


def test_burnout_reads_the_policys_input():
    src = read()
    args = call_args(src, "tr::burnoutAxialInput")
    assert "ThrustAxisPolicy::holdBurnout(" in args
    hold = call_args(args, "ThrustAxisPolicy::holdBurnout")
    for needed in ("ORIENT_MODE_MANUAL", "orient_thrust_check.running()", "t_since_launch_ms"):
        assert needed in hold, "holdBurnout() is not given %s" % needed
    assert "orient_frame_disproved ? orient_burnout_axis : nullptr" in args
    step = call_args(src, "tr::burnoutDetectStep")
    assert "burnout_axial" in step and "low_g_acc_x" not in step


def test_the_pitch_voter_and_roll_control_stand_down_on_a_disproved_frame():
    src = read()
    pitch = statement(src, r"inertial_vote\.pitch_may_vote\s*=")
    assert "!orient_frame_disproved" in pitch
    m = re.search(r"if \(reboot_recovery \|\| t_since_launch_ms < roll_delay_ms \|\|[^{]*\{", src)
    assert m, "the roll-control neutral hold moved"
    assert "orient_frame_disproved" in m.group(0)


def test_every_launch_clears_the_verdict_before_the_check_starts():
    src = read()
    i = src.index("orient_thrust_check.start(")
    before = src[i - 300:i]
    assert "orient_frame_disproved = false;" in before
    assert "orient_thrust_mismatch = false;" in before


def test_the_verdict_rides_the_snapshot_and_the_restore_applies_the_mode_alone():
    src = read()
    build = statement(src, r"snap\.b2r_mode\s*=")
    assert "ThrustAxisPolicy::packSnapshotMode(" in build
    assert "orient_frame_disproved" in build and "orient_burnout_axis" in build
    # The restore applies the low nibble only: the verdict bits are not a mode.
    m = re.search(r"applyBoardToRocketRotation\(R, snap\.b2r_code,([^;]*);", src)
    assert m, "the restore's orientation apply moved"
    assert "snap.b2r_mode" not in m.group(1)
    assert "ThrustAxisPolicy::snapshotMode(snap.b2r_mode)" in src


def test_every_reader_of_the_roll_rate_gets_it_about_the_real_axis():
    src = read()
    assign = statement(src, r"roll_rate_dps\s*=\s*ThrustAxisPolicy::rollRateDps")
    assert "orient_frame_disproved" in assign and "orient_burnout_axis" in assign
    # And nothing else assigns it from gyro_x directly.
    assert not re.search(r"roll_rate_dps\s*=\s*\(float\)ism6_latest_si\.gyro_x", src)


def test_a_sim_reset_clears_the_verdict():
    src = read()
    i = src.index("static void resetFlightStateForSim(")
    body = src[i:src.index("\n}\n", i)]
    assert "orient_frame_disproved = false;" in body
    assert "orient_thrust_mismatch = false;" in body


def test_a_new_frame_starts_without_a_verdict_and_the_restore_sets_it_after_the_frame():
    src = read()
    i = src.index("static void applyBoardToRocketRotation(")
    body = src[i:src.index("\n}\n", i)]
    assert "orient_frame_disproved = false;" in body
    # The restore applies the frame first, then the snapshot's verdict.
    apply_at = src.index("applyBoardToRocketRotation(R, snap.b2r_code")
    verdict_at = src.index("ThrustAxisPolicy::restoredVerdict(snap.b2r_mode")
    assert apply_at < verdict_at
