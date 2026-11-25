"""#1756: main.cpp's wiring of the gnss-sdr-dev board's degraded start (sdr_nav_start_policy.h).

The policy is host-tested in tests_cpp/test_sdr_nav_start_policy.cpp, including a model of the loop's pass order. What
broke was main.cpp's input to it: "no usable fix" was "no usable fix on THIS 1 kHz pass", so any re-init on the armed
pad started degraded between two of the board's fixes and stood for the flight. main.cpp is not host-built, so these
tests read the source: the degraded start is given boardWait(), the last usable fix and the re-init are stamped where
the policy assumes, and a sim's reset clears them.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")


def read():
    with open(MAIN_CPP) as fh:
        return fh.read()


def call_args(src, name):
    i = src.index(name + "(")
    j = i + len(name) + 1
    depth = 1
    while depth:
        c = src[j]
        depth += (c == "(") - (c == ")")
        j += 1
    return src[i + len(name) + 1:j - 1]


def test_the_degraded_start_waits_for_the_board():
    args = call_args(read(), "sdr_nav_start::degradedStartDue")
    first = args.split(",")[0]
    assert "normal_init" in first and "sdr_nav_start::boardWait(" in args
    wait = call_args(args, "sdr_nav_start::boardWait")
    assert "sdr_last_usable_fix_ms" in wait and "sdr_reinit_ms" in wait


def test_the_last_usable_fix_is_the_one_the_normal_start_takes():
    src = read()
    m = re.search(r"if \(have_ref_pos && gnss_gate3_init\)\s*\{[^}]*sdr_last_usable_fix_ms\s*=", src)
    assert m, "sdr_last_usable_fix_ms is not stamped on the normal start's own condition"
    # stamped after the pad average (which may set have_ref_pos on this very fix) and before the late board's
    # restart and the filter-start decision read it
    assert src.index("have_ref_pos = true;", src.index("const bool gnss_gate3_init =")) < m.start()
    assert m.start() < src.index("sdr_nav_start::restartOnFix(") < src.index("sdr_nav_start::degradedStartDue(")


def test_a_reinit_of_a_filter_on_the_board_starts_its_grace():
    src = read()
    stamp = re.search(r"if \(sdr_filter_on_board\)\s*\{[^}]*sdr_filter_on_board = false;[^}]*sdr_reinit_ms\s*=", src)
    assert stamp, "the re-init is not stamped"
    # inside the unstarted filter's block (past MAG_CALIBRATION's skip): above it, it would consume
    # sdr_filter_on_board on the pass after every start and silently disable the grace
    unstarted = src.index("else if (!ekf_initialized)", src.index("if (rocket_state == MAG_CALIBRATION)"))
    assert unstarted < stamp.start() < src.index("sdr_nav_start::degradedStartDue(")
    on = re.search(r"sdr_filter_on_board = RAW_NAV_AND\(RAW_IS_SDR\) !degraded_init;", src)
    assert on, "a start does not record whether it flies on the board"
    assert src.rfind("ekf_initialized = true;", 0, on.start()) > src.index("sdr_nav_start::degradedStartDue(")


def test_a_sim_reset_clears_the_boards_timestamps():
    src = read()
    i = src.index("static void resetFlightStateForSim(")
    body = src[i:src.index("\n}\n", i)]
    for name in ("sdr_last_usable_fix_ms = 0;", "sdr_filter_on_board    = false;", "sdr_reinit_ms          = 0;"):
        assert name in body, name


def test_a_mag_cal_exit_starts_the_grace_again():
    src = read()
    i = src.index("static void magCalBackToReady()")
    body = src[i:src.index("\n}\n", i)]
    m = re.search(r"if \(sdr_filter_on_board \|\| sdr_reinit_ms != 0\)\s*\{[^}]*sdr_reinit_ms\s*=", body)
    assert m, "magCalBackToReady does not restart the board's grace"
