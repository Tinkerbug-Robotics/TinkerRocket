"""#1757: the OC's wiring of the fault-current trip to OC_ARM_EN (pyro_fault_trip_policy.h).

The policy, pollShunt() included, is host-tested in tests_cpp/test_pyro_fault_trip_policy.cpp, with a model of
main.cpp's poll task, loop_oc and the INA_ALERT ISR. What broke was main.cpp's glue: it dropped the pin on every
over-limit reading before asking the policy, so after a stuck reading's release every 1 ms poll dropped the pin and
every loop pass raised it again. main.cpp is not host-built, so these tests read the source: the poll goes through
pollShunt(), and the trip paths drop the pin only on the policy's drop_pin.
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


def enclosing_function(src, pos):
    """The name of the function whose body holds pos (bodies open with '{' at column 0)."""
    brace = src.rfind("\n{\n", 0, pos)
    start = brace
    while True:   # back to the declaration's first line, the last one starting at column 0
        start = src.rfind("\n", 0, start)
        if src[start + 1] not in " \t\n":
            break
    decl = src[start + 1:brace]
    return re.search(r"(\w+)\s*\(", decl).group(1)


def test_the_poll_goes_through_the_policy():
    task = body(read(), "static void pyroFaultTask(void*)")
    assert re.search(r"pyroFaultApplyLocked\(\s*pollShunt\(\s*pyro_fault,\s*pyro_fault_reading,\s*pyro_fault_alert_edge,",
                     task), "the poll does not apply pollShunt()'s verdict"
    assert task.count("pyroFaultApplyLocked(") == 1
    # nothing else in the poll may touch the pin or the trip state
    for banned in ("gpio_set_level", "arm_consent_pin_level", "onOverLimit(", "pyroFaultTripLocked(", "overLimit(",
                   "notePeak(", "noteUnderLimit("):
        assert banned not in task, banned


def test_the_trip_paths_drop_the_pin_only_on_the_policys_word():
    src = read()
    apply = body(src, "static void pyroFaultApplyLocked(PyroFaultTrip::OverLimit r)")
    m = re.search(r"if \(r\.drop_pin && arm_consent_pin_level\)\s*\{\s*gpio_set_level\(\(gpio_num_t\)kArmConsentGpio, 0\);",
                  apply)
    assert m, "pyroFaultApplyLocked does not gate the drop on drop_pin"
    assert apply.count("gpio_set_level") == 1
    trip = body(src, "static void pyroFaultTripLocked(")
    assert "overLimitSeen(" in trip and "pyroFaultApplyLocked(" in trip
    assert "gpio_set_level" not in trip
    # and only the policy's verdicts reach it: the poll's and pyroFaultTripLocked's
    calls = [m.start() for m in re.finditer(r"pyroFaultApplyLocked\(", src)]
    callers = sorted(enclosing_function(src, c) for c in calls
                     if not src.startswith("static void pyroFaultApplyLocked(", c - len("static void ")))
    assert callers == ["pyroFaultTask", "pyroFaultTripLocked"], callers


def test_only_the_policy_records_an_over_limit():
    # A bare onOverLimit() in main.cpp would be a trip path that decides the pin on its own.
    assert "onOverLimit(" not in read()


def test_the_consent_pin_has_no_other_writer():
    # The pin's writers: the boot-time LOW in setup_oc, the ISR, pyroFaultApplyLocked and armConsentApplyPin.
    src = read()
    writes = [m.start() for m in re.finditer(r"gpio_(?:ll_)?set_level\([^;]*(?:kArmConsentGpio|ARM_CONSENT_PIN)", src)]
    owners = sorted({enclosing_function(src, w) for w in writes})
    assert owners == sorted({"pyroFaultAlertIsr", "pyroFaultApplyLocked", "armConsentApplyPin", "setup_oc"}), owners
