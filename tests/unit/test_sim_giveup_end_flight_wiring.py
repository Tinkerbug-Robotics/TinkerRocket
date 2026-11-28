"""#1780: the FC's wiring of the END_FLIGHT a sim given up after launch owes the OC.

The OC opens a flight's log on the FC's NSF_LAUNCH rising edge and ends it on END_FLIGHT (from LANDED) or on a user
Stop (on the OC). A sim given up by the 90 s landing hold drops the FC straight to READY, so the log stayed open and
the next real flight was appended to it. sim_flight::endsOcLog and serviceOcLogEnd (host-tested in
tests_cpp/test_sim_flight_policy.cpp) decide when END_FLIGHT is owed and send it until it is queued. main.cpp has to
decide on the give-up edge before the reset drops the launch, read the launch as the OC does, send on every pass, and
drop what is still owed at the next launch. main.cpp is not host-built, so these tests read the source.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")


def code():
    """main.cpp without its comments, each whitespace run collapsed to one space."""
    with open(MAIN_CPP) as fh:
        src = fh.read()
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
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


def function_body(src, head):
    i = src.index(head)
    return block_at(src, src.index("{", i + len(head) - 1))


def edge_block(src):
    """The block that classifies the sim's edge and acts on it."""
    i = src.index("const bool curr_sim_active = sensor_collector.isSimActive();")
    j = src.rindex("{", 0, i)
    return block_at(src, j)


def test_the_launch_is_read_as_the_oc_reads_it():
    src = code()
    body = function_body(src, "static inline bool nsfLaunchReported()")
    assert body.strip() == "{ return kinematics.launch_flag || (rocket_state == INFLIGHT); }"
    assert "if (nsfLaunchReported()) non_sensor_data.flags |= NSF_LAUNCH;" in src, "the same predicate"


def test_the_give_up_owes_end_flight_before_the_reset_drops_the_launch():
    block = edge_block(code())
    assert ("const sim_flight::Edge sim_edge = sim_flight::classify(prev_sim_active, curr_sim_active, "
            "rocket_state == LANDED);") in block
    assert "switch (sim_edge)" in block
    case = block[block.index("case sim_flight::Edge::EndedEarly:"):block.index("case sim_flight::Edge::EndedLanded:")]
    owe = "if (sim_flight::endsOcLog(sim_edge, nsfLaunchReported())) { sim_oc_log_end_owed = true;"
    assert owe in case
    assert case.index(owe) < case.index('resetFlightStateForSim("ended early");'), "decided before the reset"


def test_the_owed_end_flight_is_sent_on_every_pass_after_the_edge():
    block = edge_block(code())
    service = ("sim_flight::serviceOcLogEnd(sim_oc_log_end_owed, "
               "[] { return enqueueI2STx(END_FLIGHT, nullptr, 0); });")
    assert block.count(service) == 1
    switch = block_at(block, block.index("{", block.index("switch (sim_edge)")))
    assert service not in switch, "every pass, not only on the edge"
    assert block.index("prev_sim_active = curr_sim_active;") < block.index(service)


def test_the_reset_sends_its_clear_ahead_of_end_flight_and_keeps_what_is_owed():
    """The OC's #1779 backstop reads END_FLIGHT as the end of a flight whose clear went first."""
    src = code()
    reset = function_body(src, "static void resetFlightStateForSim(const char* edge)")
    assert "clearFlightSnapshot();" in reset
    assert "sim_oc_log_end_owed" not in reset, "a Start or Stop right after still lets it go out"


def test_what_is_owed_is_dropped_at_the_next_launch_and_set_nowhere_else():
    src = code()
    writes = re.findall(r"(?<![\w.])sim_oc_log_end_owed = (\w+);", src)
    assert sorted(writes) == ["false", "false", "true"], writes   # the declaration, the launch, the give-up
    launch = function_body(src, "static void enterInflight(uint32_t now_ms, const char* from_state)")
    assert "sim_oc_log_end_owed = false;" in launch
