"""#1777: main.cpp's wiring of the restored flight clock (restore_clock_policy.h).

The snapshot keeps the flight's times as elapsed from launch. The restore has to put them back on this boot's clock
counted from the boot's start (RestoreClock::launchMs), not from the restore's own moment, so the setup before the
restore stays in the flight's clock; apogee and burnout follow from the launch time, and the time-after-apogee trigger
reads them through the same test the host tests step (tests_cpp/test_restore_clock.cpp). The launch time wraps below
0, so every reader of the three times has to take a difference or a signed difference. main.cpp is not host-built, so
these tests read the source.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")


def code():
    """main.cpp without its comments, so a comment cannot satisfy or break a check."""
    with open(MAIN_CPP) as fh:
        src = fh.read()
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)


def block_at(src, j):
    """The balanced {...} block opening at src[j]."""
    depth, k = 0, j
    while True:
        c = src[k]
        depth += (c == "{") - (c == "}")
        k += 1
        if depth == 0:
            return src[j:k]


def restore_block(src):
    """The `if (valid) { ... }` block of setup_fc that restores a flight from the snapshot."""
    blocks = [block_at(src, m.end() - 1) for m in re.finditer(r"if \(valid\)\s*\{", src)]
    restoring = [b for b in blocks if "INFLIGHT REBOOT RECOVERY" in b]
    assert len(restoring) == 1, "one block restores the flight"
    return restoring[0]


def test_the_header_is_included():
    with open(MAIN_CPP) as fh:
        assert '#include "restore_clock_policy.h"' in fh.read()


def test_the_launch_is_rebased_on_this_boots_start():
    block = restore_block(code())
    assert "launch_time_millis = RestoreClock::launchMs(snap.flight_elapsed_ms);" in block
    assert not re.search(r"launch_time_millis\s*=\s*now_ms\s*-", block), "not on the restore's moment"
    assert "now_ms - snap." not in block


def test_apogee_and_burnout_follow_from_the_rebased_launch():
    block = restore_block(code())
    launch = block.index("launch_time_millis = RestoreClock::launchMs(")
    apogee = block.index("pyro_apogee_time_ms = RestoreClock::eventMs(launch_time_millis, snap.apogee_elapsed_ms);")
    burnout = block.index("burnout_time_ms = RestoreClock::eventMs(launch_time_millis, snap.burnout_elapsed_ms);")
    assert launch < apogee and launch < burnout, "from this boot's launch time, set first"
    assert re.search(r"if \(snap\.pyro_apogee_detected\)\s*\{\s*pyro_apogee_detected = true;\s*"
                     r"pyro_apogee_time_ms = RestoreClock::eventMs", block)
    assert re.search(r"if \(snap\.burnout_detected\)\s*\{\s*burnout_detected = true;\s*"
                     r"burnout_time_ms = RestoreClock::eventMs", block)


def test_the_restore_logs_the_setup_it_counts():
    block = restore_block(code())
    m = re.search(r'ESP_LOGW\(TAG, "\[RECOVERY\] flight clock from this boot\'s start[^;]*;', block, flags=re.S)
    assert m, "the bench reads the boot's time to the restore from the log"
    assert "(unsigned long)now_ms" in m.group(0)


def test_the_snapshot_keeps_the_times_as_elapsed_from_launch():
    src = code()
    assert "snap.flight_elapsed_ms  = now_ms - launch_time_millis;" in src
    assert "? (pyro_apogee_time_ms - launch_time_millis) : 0;" in src
    assert "? (burnout_time_ms - launch_time_millis) : 0;" in src


def test_the_time_after_apogee_trigger_is_the_host_tested_one():
    src = code()
    m = re.search(r"if \(apogee_usable && mode == PYRO_TRIGGER_TIME_AFTER_APOGEE\)\s*\{(.*?)\}", src, flags=re.S)
    assert m, "the trigger's branch"
    assert m.group(1).strip() == \
        "if (RestoreClock::timeAfterApogeeDue(now_ms, pyro_apogee_time_ms, val)) should_fire = true;"
    assert "elapsed_s >= val" not in src, "no second copy of the test"


def writes(src, name):
    """What NAME is set to, past its declaration."""
    return [src[m.end():src.find(";", m.end())].strip()
            for m in re.finditer(r"(?<![\w.])%s\s*=(?!=)" % name, src)
            if not re.search(r"static uint32_t\s*$", src[max(0, m.start() - 40):m.start()])]


def test_nothing_else_re_times_the_flight():
    src = code()
    assert sorted(writes(src, "launch_time_millis")) == sorted([
        "RestoreClock::launchMs(snap.flight_elapsed_ms)",   # the restore
        "now_ms",                                          # launch
    ])
    assert sorted(writes(src, "pyro_apogee_time_ms")) == sorted([
        "now_ms",                                                              # the live apogee
        "RestoreClock::eventMs(launch_time_millis, snap.apogee_elapsed_ms)",   # the restore
        "0",                                                                   # the reset
    ])
    assert sorted(writes(src, "burnout_time_ms")) == sorted([
        "RestoreClock::eventMs(launch_time_millis, snap.burnout_elapsed_ms)",  # the restore
        "0", "0",                                                              # the resets
        "now_ms", "now_ms",                                                    # the live detector, #1776
    ])


# A reader of the three times: a difference, the restore's own helpers, the SDR phase
# message's signed sinceMs, or the LANDED release's moment an answer counts from.
READER_SHAPES = [
    r"-\s*$",                                           # x - NAME
    r"^\s*-",                                           # NAME - x
    r"RestoreClock::eventMs\($",
    r"RestoreClock::timeAfterApogeeDue\(now_ms,\s*$",
    r"\bf\.(launch|apogee|burnout)_ms = $",             # sdr_phase::sinceMs (signed)
    r"pwr_hold::answerFromMs\($",                       # then ocAliveSinceLaunch, signed
]


def test_every_reader_takes_a_difference():
    """The launch time wraps below 0 on a restore: an ordered compare against it would be wrong."""
    src = code()
    for name in ("launch_time_millis", "pyro_apogee_time_ms", "burnout_time_ms"):
        for m in re.finditer(r"\b%s\b" % name, src):
            before = src[max(0, m.start() - 160):m.start()]
            after = src[m.end():m.end() + 40]
            line = src[src.rfind("\n", 0, m.start()) + 1:src.find("\n", m.end())].strip()
            if re.match(r"\s*=(?!=)", after) or re.search(r"static uint32_t\s*$", before):
                continue   # a write, checked above, or the declaration
            ok = any(re.search(p, before) for p in READER_SHAPES if not p.startswith("^")) or \
                any(re.match(p, after) for p in READER_SHAPES if p.startswith("^"))
            assert ok, "%s read without a difference: %s" % (name, line)


def test_at_landing_a_restored_flight_counts_an_oc_answer_only_from_the_restore():
    src = code()
    assert re.search(r"const uint32_t answer_from_ms =\s*pwr_hold::answerFromMs\(launch_time_millis, "
                     r"recovery_gate_active,\s*recovery_restore_ms\);", src)
    calls = re.findall(r"pwr_hold::ocAliveSinceLaunch\(([^;]*)\);", src)
    assert [re.sub(r"\s+", " ", c) for c in calls] == ["out_ready, out_ready_last_ms, answer_from_ms"]
    assert writes(src, "recovery_restore_ms") == ["now_ms"], "the restore alone sets it"
    assert "recovery_restore_ms   = now_ms;" in restore_block(src), "stamped at the restore"
