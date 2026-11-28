"""#1717: the OC's main.cpp wiring of its boot re-seed after a reset of its own.

The FC asks the OC for its flight snapshot at boot. After an OC reset too, the OC's RAM cache is empty, and the #846
re-seed looked only for a flight_recovered_* index entry that the deferred scan (since #1307 on its own task, reading
every page of the flight) had not created yet. So the FC got the flight token's launch-edge frame or nothing.

Now, in order: the cache's RTC copy (snapshot_rtc_copy.h) seeds it at setup on a boot resuming a flight; the quick
path (TR_FlightLog::newestOrphanTail, snapshot_reseed_policy.h) finds the cut-off flight's tail on NAND; the #846
indexed path runs only when no cut-off flight is newer; and the deferred scan's task starts after all of that. The
policies are host-tested in tests_cpp (test_snapshot_reseed.cpp, test_orphan_tail.cpp); main.cpp is not host-built,
so these tests read the source.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "out_computer", "main", "main.cpp")


def code():
    """main.cpp without its comments, so a comment cannot satisfy or break a check."""
    with open(MAIN_CPP) as fh:
        src = fh.read()
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)


def function(src, signature):
    """The body of the function whose definition starts with `signature`."""
    i = src.index(signature)
    j = src.index("{", i)
    depth, k = 0, j
    while True:
        depth += (src[k] == "{") - (src[k] == "}")
        k += 1
        if depth == 0:
            return src[j:k]


def test_the_headers_are_included():
    with open(MAIN_CPP) as fh:
        raw = fh.read()
    for h in ('#include "snapshot_rtc_copy.h"', '#include "snapshot_reseed_policy.h"', "#include <esp_rtc_time.h>"):
        assert h in raw


def test_the_rtc_copy_lives_in_rtc_memory_and_every_cache_write_stores_it():
    src = code()
    assert re.search(r"RTC_NOINIT_ATTR static SnapshotRtcCopy::Copy<kSnapFrameLen> snapshot_rtc;", src)
    stores = re.findall(r"SnapshotRtcCopy::store\(([^;]*)\);", src)
    assert stores.count("snapshot_rtc, frame, esp_rtc_get_time_us()") == 1, "the parser's"
    assert stores.count("snapshot_rtc, best_f, esp_rtc_get_time_us()") == 2, \
        "and each NAND seed, so a second reset before a new frame keeps it"
    assert len(stores) == 3
    # right after the parser task's cache write, in the same block
    m = re.search(r"if \(frame_len == kSnapFrameLen\)\s*\{(.*?)\n                \}", src, flags=re.S)
    assert m
    block = m.group(1)
    assert block.index("memcpy(snapshot_cache, frame, kSnapFrameLen);") < block.index("SnapshotRtcCopy::store(")


def test_setup_seeds_from_the_rtc_copy_once_after_the_rail_decision():
    src = code()
    calls = [m.start() for m in re.finditer(r"\bseedSnapshotCacheFromRtcCopy\(\);", src)]
    assert len(calls) == 1
    assert src.index("boot_token_restore = true;") < calls[0], "after the token restore is decided"
    assert src.index("nvs_err = nvs_flash_init();") < calls[0]
    setup = function(src, "static void setup_oc()")
    assert "seedSnapshotCacheFromRtcCopy();" in setup, "in setup, before loop_oc brings up the parser and the logger"


def test_the_rtc_seed_serves_only_a_recovery_boot_and_retires_the_copy_otherwise():
    body = function(code(), "static void seedSnapshotCacheFromRtcCopy()")
    assert "const bool recovery_context = boot_rail_restored || boot_token_restore;" in body
    assert "SnapshotRtcCopy::ageMs(snapshot_rtc, esp_rtc_get_time_us(), age_ms)" in body
    assert "aged && snapshotServable(snapshot_rtc.frame, age_ms)" in body
    assert re.search(r"snapshot_rtc_verdict = SnapshotRtcCopy::verdict\(recovery_context, aged, age_ms,\s*"
                     r"kSnapshotServeTtlMs, servable\);", body)
    assert "if (snapshot_rtc_verdict == SnapshotRtcCopy::Verdict::Seed)" in body
    assert "snapshot_cache_ms = millis() - age_ms;" in body, "the cache's TTL runs from when it was stored"
    assert "snapshot_rtc_seeded = true;" in body
    seed_at = body.index("snapshot_rtc_seeded = true;")
    assert body.index("return;", seed_at) < body.index("SnapshotRtcCopy::retire(snapshot_rtc);"), \
        "a seeded copy is kept; every other path retires it"


def test_the_quick_path_runs_before_the_indexed_path_and_settles_nothing_early():
    body = function(code(), "static bool initLoggingSubsystem()")
    quick = body.index("flightlog.newestOrphanTail(")
    indexed = body.index('strncmp(e.filename, "flight_recovered_", 17) == 0')
    settled = body.index("snapshot_stores_settled = logger.isMramEnabled() || flightlog.isInitialized();")
    assert quick < indexed < settled
    assert "const bool newer_in_cache = snapshot_rtc_seeded || snapshot_cache_valid;" in body
    assert re.search(r"const bool rtc_says_over =\s*snapshot_rtc_verdict == SnapshotRtcCopy::Verdict::Over;", body)
    assert "const bool want_tail = !cold_boot && !newer_in_cache && !rtc_says_over;" in body
    assert re.search(r"if \(!orphan_newest && best_idx >= 0 && best_id == newest_id\)", body), \
        "the indexed path only when no cut-off flight is newer"


def test_the_quick_paths_tail_must_reach_its_last_page_and_be_servable():
    body = function(code(), "static bool initLoggingSubsystem()")
    m = re.search(r"const bool tail_servable = (.*?);", body, flags=re.S)
    assert m
    cond = re.sub(r"\s+", " ", m.group(1))
    for term in ("want_tail", "scan.fs.found", "ot.end_fed", "ot.gaps == 0", "!scan.overflow",
                 "snapshotServable(last_f, 0)", "snapshotServable(best_f, 0)"):
        assert term in cond, term
    assert "scan.fs.order_off = offsetof(FlightSnapshotData, flight_elapsed_ms);" in re.sub(r" +", " ", body)


def test_each_orphan_action_does_what_the_policy_says():
    body = function(code(), "static bool initLoggingSubsystem()")
    assert re.search(r"SnapshotReseed::orphanAction\(\s*SnapshotReseed::sameOrphan\(orphan_mark, found\), cold_boot,\s*"
                     r"newer_in_cache, rtc_says_over, tail_servable\)", body)
    def case(name):
        i = body.index("case SnapshotReseed::OrphanAction::%s:" % name)
        return body[i:body.index("break;", i)]
    assert "markNow(" in case("Retire")
    assert "markNow(" in case("Decline")
    for name in ("MarkOnServe", "Seed"):
        c = case(name)
        assert "orphan_seed_pending_mark = true;" in c and "orphan_seed_id = found;" in c
        assert "markNow(" not in c, "marked only once the FC is served"
    seed = case("Seed")
    assert "memcpy(snapshot_cache, best_f, kSnapFrameLen);" in seed
    assert "SnapshotRtcCopy::store(snapshot_rtc, best_f, esp_rtc_get_time_us());" in seed
    assert "snapshot_cache" not in case("MarkOnServe"), "the newer cached frame is left alone"
    marknow = body[body.index("auto markNow"):body.index("switch (SnapshotReseed::orphanAction(")]
    for key in ('"or_id", found.flight_id', '"or_blk", found.start_block', '"or_seq", found.last_seq'):
        assert key in marknow


def test_the_indexed_path_honors_the_orphan_marker_and_follows_its_policy():
    body = function(code(), "static bool initLoggingSubsystem()")
    assert re.search(r"SnapshotReseed::indexedAlreadyDone\(\s*done_id, done_by, orphan_mark, e\.flight_id, "
                     r"e\.final_bytes,\s*e\.start_block, e\.n_blocks\)", body)
    assert re.search(r"SnapshotReseed::indexedAction\(already, cold_boot, newer_in_cache,\s*rtc_says_over\)", body)
    def case(name):
        i = body.index("case SnapshotReseed::IndexedAction::%s:" % name)
        return body[i:body.index("break;", i)]
    assert 'rp.putUInt("done_id", e.flight_id);' in case("Retire")
    assert 'rp.putUInt("done_id", e.flight_id);' in case("Decline")
    mos = case("MarkOnServe")
    assert "snapshot_seed_pending_mark = true;" in mos and "memcpy(snapshot_cache" not in mos
    scan = case("Scan")
    assert "tr_flightlog::tailScanForFrame(" in scan
    assert "tailScanForFrame(" not in "".join(case(n) for n in ("Nothing", "Retire", "MarkOnServe", "Decline"))


def test_the_deferred_scan_starts_after_the_re_seed():
    body = function(code(), "static bool initLoggingSubsystem()")
    creates = [m.start() for m in re.finditer(r'"fl_recovery"', body)]
    assert len(creates) == 1
    assert body.index("flightlog.newestOrphanTail(") < creates[0]
    assert body.index('strncmp(e.filename, "flight_recovered_", 17) == 0') < creates[0]
    assert creates[0] < body.index("snapshot_stores_settled = logger.isMramEnabled()")
    assert "start_recovery_task = flightlog.recoveryDeferred();" in body
    assert re.search(r"if \(start_recovery_task\)\s*\{\s*xTaskCreatePinnedToCore\(", body)


def test_the_orphan_marker_is_written_once_the_fc_is_served():
    src = code()
    m = re.search(r"if \(orphan_seed_pending_mark && snapshot_served\)\s*\{(.*?)\n    \}", src, flags=re.S)
    assert m
    block = m.group(1)
    assert "orphan_seed_pending_mark = false;" in block
    for key in ('"or_id", orphan_seed_id.flight_id', '"or_blk", orphan_seed_id.start_block',
                '"or_seq", orphan_seed_id.last_seq'):
        assert key in block
