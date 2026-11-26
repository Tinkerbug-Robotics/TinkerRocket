"""#1756, #1762: main.cpp's wiring of the navigation filter's start (flight_computer/main/nav_start_policy.h).

The policy is host-tested in tests_cpp/test_nav_start_policy.cpp (with a model of the loop's pass order and the
review's three routes) and, for the gnss-sdr-dev board, tests_cpp/test_sdr_nav_start_policy.cpp. main.cpp is not
host-built, so these tests read the source:
- #1762: every receiver takes the rule (it used to be the board's alone; an answering receiver that never fixed left
  the filter unstarted for the flight), with GNSS-absent mode starting at once as before;
- a degraded start gives way to a usable fix on the pad unarmed, and holds the receiver's fixes, raw epochs and pad
  datum out of the filter while it stands;
- #1756: "no usable fix" is none for a while (receiverWait), stamped where the policy assumes.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")


def read():
    with open(MAIN_CPP) as fh:
        return fh.read()


def code(text):
    return "\n".join(line.split("//", 1)[0] for line in text.splitlines())


def call_args(src, name):
    i = src.index(name + "(")
    j = i + len(name) + 1
    depth = 1
    while depth:
        c = src[j]
        depth += (c == "(") - (c == ")")
        j += 1
    return src[i + len(name) + 1:j - 1]


def body(src, signature):
    i = src.index(signature)
    while True:
        eol = src.index("\n", i)
        if src.startswith("\n{\n", eol):
            j = eol + 3
            return src[j:src.index("\n}\n", j)]
        i = src.index(signature, eol)


def test_every_receiver_takes_the_start_rule():
    src = code(read())
    assert '#include "nav_start_policy.h"' in src
    # one degraded start, for every image and receiver: no #if around it, no per-receiver branch
    assert src.count("nav_start::degradedStartDue(") == 1
    args = call_args(src, "nav_start::degradedStartDue")
    flat = " ".join(args.split())
    assert flat.startswith("normal_init || nav_start::receiverWait(nav_last_usable_fix_ms, nav_reinit_ms, millis()),")
    assert flat.endswith("rocket_state, gnss_absent_mode"), "a receiver that failed bring-up must still start at once"
    assert "!gnss_started" not in src[src.index("const bool normal_init = have_ref_pos && gnss_gate3_init;"):
                                      src.index("nav_start::degradedStartDue(")]
    i = src.index("const bool degraded_init = nav_start::degradedStartDue(")
    block = src[src.rindex("else if (!ekf_initialized)", 0, i):i]
    assert "#if" not in block, "the start rule sits behind a preprocessor branch"


def test_a_late_usable_fix_restarts_a_degraded_start_on_the_pad():
    src = code(read())
    m = re.search(r"if \(ekf_initialized && gnss_absent_flight &&\s*"
                  r"nav_start::restartOnFix\(true, have_ref_pos && gnss_gate3_init, rocket_state\)\)\s*\{"
                  r"\s*ekf_initialized = false;\s*gnss_absent_flight = false;", src)
    assert m, "the restart on a late fix is not every receiver's"
    assert m.start() < src.index("if (rocket_state == MAG_CALIBRATION)", m.start())


def test_a_degraded_start_holds_gnss_out_of_the_filter():
    src = code(read())
    # the classic filter: a fix with time 0, skipped like the degraded seed's
    assert re.search(r"ekf\.update\(use_ahrs_acc, ekf_imu, gnss_absent_flight \? kNoGnssFix : ekf_gnss, ekf_mag\);", src)
    assert "static const EkfGNSSDataLLA kNoGnssFix{};" in src
    # the raw filter: every raw receiver's epochs held (not the board's alone)
    task = body(src, "static void rawGnssMeasTask(void*)")
    h = task.index("if (raw_gnss_held)")
    assert "RAW_IS_SDR" not in task[task.rindex("\n", 0, h):h]
    assert "raw_gnss_held_n++;" in task[h:h + 200] and "continue;" in task[h:h + 200]
    assert task.index("while (sensor_collector.getGNSSRawEpoch(epoch))") < h
    assert "raw_gnss_held = gnss_absent_flight;" in src
    # the pad datum and the pad average take no fix while a degraded start stands armed
    assert "const bool degraded_armed = gnss_absent_flight && rocket_state != READY;" in src
    assert "const bool ref_take = (!ref_from_nav || ref_nav_now) && !degraded_armed;" in src
    assert "const bool ref_take = !degraded_armed;" in src
    assert "if (gnss_gate3_init && !degraded_armed &&" in src


def test_the_last_usable_fix_is_the_one_the_normal_start_takes():
    src = code(read())
    m = re.search(r"if \(have_ref_pos && gnss_gate3_init\)\s*\{[^}]*nav_last_usable_fix_ms\s*=", src)
    assert m, "nav_last_usable_fix_ms is not stamped on the normal start's own condition"
    # after the pad average (which may set have_ref_pos on this very fix), before the restart and the start read it
    assert src.index("have_ref_pos = true;", src.index("const bool gnss_gate3_init =")) < m.start()
    assert m.start() < src.index("nav_start::restartOnFix(") < src.index("nav_start::degradedStartDue(")


def test_a_reinit_of_a_filter_on_fixes_starts_its_grace():
    src = code(read())
    stamp = re.search(r"if \(nav_filter_on_fixes\)\s*\{[^}]*nav_filter_on_fixes = false;[^}]*nav_reinit_ms\s*=", src)
    assert stamp, "the re-init is not stamped"
    unstarted = src.index("else if (!ekf_initialized)", src.index("if (rocket_state == MAG_CALIBRATION)"))
    assert unstarted < stamp.start() < src.index("nav_start::degradedStartDue(")
    on = re.search(r"nav_filter_on_fixes = !degraded_init;", src)
    assert on, "a start does not record whether it flies on fixes"
    assert src.rfind("ekf_initialized = true;", 0, on.start()) > src.index("nav_start::degradedStartDue(")


def test_a_sim_reset_clears_the_receivers_timestamps():
    fn = code(body(read(), "static void resetFlightStateForSim("))
    for name in ("nav_last_usable_fix_ms = 0;", "nav_filter_on_fixes    = false;", "nav_reinit_ms          = 0;"):
        assert name in fn, name


def test_a_mag_cal_exit_starts_the_grace_again():
    fn = code(body(read(), "static void magCalBackToReady()"))
    assert re.search(r"if \(nav_filter_on_fixes \|\| nav_reinit_ms != 0\)\s*\{[^}]*nav_reinit_ms\s*=", fn)
