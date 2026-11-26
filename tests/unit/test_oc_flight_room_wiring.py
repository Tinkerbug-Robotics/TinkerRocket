"""#1758: the OC's wiring of the flight log's room (TR_FlightLog::Config, flightRoom(), storage_health_policy.h).

TR_FlightLog's room target, eviction plan and short last extension are host-tested in
tests_cpp/test_tr_flightlog_core.cpp with the OC's configuration, and the verdict's room rule in
tests_cpp/test_storage_health_policy.cpp. main.cpp is not host-built, so these tests read the source: the OC sets
the room target the owner chose (a fifth of the flight region, 2026-11-25), keeps auto-eviction on, and feeds the
storage verdict the room and free count the next arm will leave, not the raw free count.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "out_computer", "main", "main.cpp")


def read():
    with open(MAIN_CPP) as fh:
        return fh.read()


def body(src, signature):
    i = src.index(signature)
    j = src.index("\n{\n", i) + 3
    return src[j:src.index("\n}\n", j)]


def code(text):
    """text without // comments, so a comment cannot satisfy or trip a check."""
    return "\n".join(line.split("//", 1)[0] for line in text.splitlines())


def test_the_arm_clears_a_fifth_of_the_region():
    src = code(read())
    assert re.search(r"fl_cfg\.auto_evict_oldest\s*=\s*true;", src)
    m = re.search(r"fl_cfg\.auto_evict_target_room_blocks\s*=\s*static_cast<uint16_t>\(\s*"
                  r"static_cast<uint32_t>\(fl_cfg\.flight_region_end - fl_cfg\.flight_region_start\)\s*/\s*5u\);",
                  src)
    assert m, "the room target is not a fifth of the flight region"
    # set before begin(), which copies the config
    assert m.start() < src.index("flightlog.begin(", m.start())


def test_the_storage_verdict_reads_the_planned_room():
    fn = code(body(read(), "static SensorHealthState ocStorageHealth()"))
    assert re.search(r"=\s*flightlog\.flightRoom\(\);", fn), "the verdict does not plan the arm's room"
    assert re.search(r"in\.region_free_blocks\s*=\s*room\.region_free_blocks;", fn)
    assert re.search(r"in\.flight_room_blocks\s*=\s*room\.room_blocks;", fn)
    assert re.search(r"in\.room_target_blocks\s*=\s*flightlog\.config\(\)\.auto_evict_target_room_blocks;", fn)
    assert "regionFreeBlocks()" not in fn, "the raw free count reads BAD in READY although the arm will make room"
    assert re.search(r"in\.index_used\s*=\s*flightlog\.index\(\)\.size\(\)\s*-\s*room\.evictions;", fn), \
        "a full index reads BAD in READY although the arm frees a slot"
