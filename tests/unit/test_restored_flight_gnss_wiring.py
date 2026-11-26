"""#1765: on the one image an FC reset in flight no longer waits out the GNSS search before the flight resumes.

The bring-up's restored-flight plan is host-tested in tests_cpp/test_gnss_detect.cpp (GnssBringUpRestored: the u-blox's
probes for 3 s, its driver only if one answered, no search and no second look, a flight found late). These tests read
the source that wires it, which is not host-built:
- main.cpp asks the OC for a flight in progress before the GNSS bring-up whatever the receiver last boot was, and on a
  flight in progress defers the bring-up (TR_GNSSAuto::setBeginWait(false));
- when that ask had no answer, the bring-up asks again (fcFlightFoundLate) before the u-blox driver's search for
  nothing proven and before the second look;
- TR_GNSSAuto::begin starts the hinted raw receiver's driver as before, and otherwise hands the bring-up the restored
  flight.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
IDF = os.path.join(ROOT, "tinkerrocket-idf")
MAIN_CPP = os.path.join(IDF, "projects", "flight_computer", "main", "main.cpp")
AUTO = os.path.join(IDF, "components", "TR_Sensor_Collector", "gnss_auto_driver.cpp")
COLLECTOR = os.path.join(IDF, "components", "TR_Sensor_Collector", "TR_Sensor_Collector.cpp")


def code(path):
    with open(path) as fh:
        return "\n".join(line.split("//", 1)[0] for line in fh.read().splitlines())


def flat(text):
    return re.sub(r"\s+", " ", text)


def pp_stack(src, pos):
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


def test_the_one_image_asks_before_the_bring_up_whatever_the_hint():
    src = code(MAIN_CPP)
    asks = [m.start() for m in re.finditer(r"const OcSnapshotAnswer raw_boot_ask\s*=", src)]
    assert len(asks) == 1, "one early ask, for every image that has one"
    i = asks[0]
    stmt = flat(src[i:src.index(";", i)])
    assert stmt == "const OcSnapshotAnswer raw_boot_ask = fcAskOcForFlightSnapshot()", stmt
    # on the one image too: no #if TR_RAW_NAV_AUTO branch of its own around it
    assert not any("TR_RAW_NAV_AUTO" in c for c in pp_stack(src, i)), pp_stack(src, i)
    assert "gnssTypeGivesRaw(raw_boot_hint) ?" not in flat(src), "the ask is not gated on the hint (#1765)"
    begin = src.index("sensor_collector.begin(")
    assert i < begin, "the ask comes before the GNSS bring-up"


def test_a_flight_in_progress_defers_the_bring_up_for_every_hint():
    src = code(MAIN_CPP)
    i = src.index("const OcSnapshotAnswer raw_boot_ask")
    m = re.search(r"if \(raw_boot_ask\.valid\)\s*\{\s*sensor_collector_hw\.deferGNSSBringUpThisBoot\(\);", src[i:])
    assert m, "deferGNSSBringUpThisBoot() first in the valid branch, unconditional"


def test_the_auto_driver_hands_the_bring_up_the_restored_flight():
    src = code(AUTO)
    body = src[src.index("bool TR_GNSSAuto::begin("):src.index("uint8_t TR_GNSSAuto::updateRateHz()")]
    hinted = body.index("beginHintedRaw(rememberedType()")
    start = body.index("up.start(")
    assert hinted < start, "the raw hints' own start comes first"
    assert re.search(r"const bool restored_flight = !begin_wait_;", body)
    args = flat(body[start:body.index(";", start)])
    assert args.endswith("kLookForPx1105r, restored_flight)"), args


def test_the_bring_up_asks_again_before_a_search_or_a_second_look():
    src = code(AUTO)
    body = src[src.index("bool TR_GNSSAuto::begin("):src.index("uint8_t TR_GNSSAuto::updateRateHz()")]
    # right before the u-blox driver, for nothing proven
    assert re.search(r"if \(up\.proven\(\) == GNSS_TYPE_NONE && flightFoundLate\(up\)\) break;\s*"
                     r"const bool ok = ublox_\.begin\(", body)
    # right before the second look's probes
    assert re.search(r"const bool second = up\.step\(\) == gnss_detect::BringUp::Step::SECOND_LOOK;\s*"
                     r"if \(second && flightFoundLate\(up\)\) break;\s*runPlan\(", body)
    helper = src[src.index("bool TR_GNSSAuto::flightFoundLate("):]
    helper = helper[:helper.index("\n}\n")]
    assert "up.restoredFlight() || flight_check_ == nullptr || !flight_check_()" in flat(helper)
    assert "up.flightFoundLate();" in helper
    # ends only where the bring-up did, and leaves the UART closed (GNSS-absent)
    assert re.search(r"if \(up\.step\(\) != gnss_detect::BringUp::Step::DONE\) return false;\s*uartClose\(\);", helper)


def test_main_hands_the_bring_up_the_late_ask():
    src = code(MAIN_CPP)
    i = src.index("const OcSnapshotAnswer raw_boot_ask")
    begin = src.index("sensor_collector.begin(")
    seg = src[i:begin]
    assert "s_boot_ask_definitive = raw_boot_ask_definitive;" in seg
    assert "sensor_collector_hw.setGnssFlightCheck(fcFlightFoundLate);" in seg
    fn = src[src.index("static bool fcFlightFoundLate()"):]
    fn = flat(fn[:fn.index("\n}\n")])
    assert "if (s_boot_ask_definitive) return false;" in fn, "a definitive early answer asks nothing more"
    assert "fcAskOcForFlightSnapshot(1)" in fn, "one attempt per late ask"
    assert re.search(r"if \(a\.valid \|\| a\.answered_no_flight\) \{ s_boot_ask_definitive = true; s_late_ask = a; \}", fn)
    assert "return a.valid;" in fn
    # the restore takes the late answer as it takes the early one
    assert re.search(r"raw_boot_ask_definitive \? raw_boot_ask : s_boot_ask_definitive \? s_late_ask : "
                     r"fcAskOcForFlightSnapshot\(\)", flat(src))
    col = code(COLLECTOR)
    assert re.search(r"gnss_receiver\.setFlightCheck\(gnss_flight_check_\);[\s\S]{0,200}?gnss_receiver\.begin\(", col)
