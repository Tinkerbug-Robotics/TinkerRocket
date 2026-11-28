"""#1779: the wiring of the LANDED snapshot clear on the FC, and of its END_FLIGHT backstop on the OC.

The FC's LANDED clear retires the flight the OC would otherwise serve to the next boot for 30 min. It was sent once,
from the one-shot LANDED actions, into an I2S queue that can be full at LANDED entry; END_FLIGHT, beside it, was
retried. Now the FC tries the clear every 20 ms until it is queued, queues END_FLIGHT behind it and repeats the clear
every second while LANDED (landed_sends_policy.h), and the OC, given
END_FLIGHT while its cache still reads INFLIGHT, marks the cache and its RTC copy LANDED (snapshot_end_flight_policy.h).
Both are host-tested in tests_cpp/test_landed_clear.cpp. Neither main.cpp is host-built, so these tests read the
source.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
FC_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")
OC_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "out_computer", "main", "main.cpp")


def raw(path):
    with open(path) as fh:
        return fh.read()


def code(path):
    """The source without its comments, each whitespace run collapsed to one space."""
    src = re.sub(r"/\*.*?\*/", "", raw(path), flags=re.S)
    src = re.sub(r"//[^\n]*", "", src)
    return re.sub(r"\s+", " ", src)


def block_at(src, j):
    """The balanced {...} block opening at src[j]."""
    depth, k = 0, j
    while True:
        c = src[k]
        depth += (c == "{") - (c == "}")
        k += 1
        if depth == 0:
            return src[j:k]


def landed_case(src):
    i = src.index("case LANDED: {")
    return block_at(src, i + len("case LANDED: "))


SERVICE = ("LandedSends::service( landed_sends, now_ms, [] { return clearFlightSnapshot(); }, "
           "[] { return enqueueI2STx(END_FLIGHT, nullptr, 0); });")


# --------------------------------------------------------------------------- the FC


def test_the_fc_includes_the_policy():
    assert '#include "landed_sends_policy.h"' in raw(FC_CPP)


def test_the_clear_says_whether_it_went_into_the_queue():
    src = code(FC_CPP)
    head = "static bool clearFlightSnapshot() {"
    assert head in src, "clearFlightSnapshot returns bool"
    body = block_at(src, src.index(head) + len(head) - 1)
    assert "return enqueueI2STx(SNAPSHOT_MSG," in body
    assert "(void)enqueueI2STx(SNAPSHOT_MSG" not in body


def test_landed_retries_the_clear_then_end_flight_on_every_pass():
    src = code(FC_CPP)
    case = landed_case(src)
    once = block_at(case, case.index("if (!landed_actions_done) {") + len("if (!landed_actions_done) "))
    assert "clearFlightSnapshot" not in once, "not in the one-shot actions"
    assert case.count(SERVICE) == 1
    assert case.index(SERVICE) > case.index(once) + len(once) - 1, "after the one-shot block, on every pass"
    assert case.rstrip().endswith(SERVICE + " break; }"), "the case's last step"


def test_end_flight_is_sent_from_landed_alone():
    """The OC's backstop rests on it: END_FLIGHT comes only from LANDED, after the clear."""
    src = code(FC_CPP)
    assert src.count("enqueueI2STx(END_FLIGHT") == 1
    assert "enqueueI2STx(END_FLIGHT" in landed_case(src)
    assert "end_flight_sent" not in src, "one latch for both"


def test_the_latches_start_over_with_the_one_shot_latch():
    src = code(FC_CPP)
    resets = [m.start() for m in re.finditer(r"landed_sends = LandedSends::State\{\};", src)]
    one_shot = [m.start() for m in re.finditer(r"(?<!static bool )landed_actions_done = false;", src)]
    assert len(resets) == len(one_shot) == 3, "the restore, a sim's reset and a launch"
    for r in resets:
        assert any(abs(r - o) < 80 for o in one_shot), "beside landed_actions_done = false"


# --------------------------------------------------------------------------- the OC


def test_the_oc_includes_the_policy():
    assert '#include "snapshot_end_flight_policy.h"' in raw(OC_CPP)


def end_flight_branch(src):
    m = re.search(r"else if \(type == END_FLIGHT\) \{", src)
    assert m, "the END_FLIGHT branch"
    return block_at(src, m.end() - 1)


def test_end_flight_marks_a_cache_still_inflight_and_its_rtc_copy():
    src = code(OC_CPP)
    body = end_flight_branch(src)
    lock = "portENTER_CRITICAL(&snapshot_cache_mux);"
    mark = "const bool retired = snapshot_cache_valid && SnapshotEndFlight::markLanded(snapshot_cache, kSnapFrameLen);"
    unlock = "portEXIT_CRITICAL(&snapshot_cache_mux);"
    rtc = "const bool rtc_retired = SnapshotEndFlight::markLanded(snapshot_rtc);"
    for s in (lock, mark, unlock, rtc):
        assert s in body, s
    assert body.index(lock) < body.index(mark) < body.index(unlock) < body.index(rtc), "the cache under its lock"
    assert body.index(rtc) < body.index("logger.endLogging();"), "before the log is ended"
    assert "flightlogEndFlight();" in body


def test_end_flight_is_handled_where_the_snapshots_are_stored():
    """The branch shares processFrame with the snapshot store. processFrame also runs for frames from the I2C ingress,
    but END_FLIGHT and the snapshots both come over I2S, so both are handled on the I2S parser task, and the RTC copy's
    writes stay on that one task."""
    src = code(OC_CPP)
    fns = [m.start() for m in re.finditer(r"\bstatic [\w:<>*& ]+\b(\w+)\([^;{)]*\)\s*\{", src)]
    def owner(pos):
        return max(f for f in fns if f < pos)
    end_at = src.index("else if (type == END_FLIGHT) {")
    store_at = src.index("SnapshotRtcCopy::store(snapshot_rtc, frame, esp_rtc_get_time_us());")
    assert owner(end_at) == owner(store_at)
    assert src.count("SnapshotEndFlight::markLanded(") == 2
