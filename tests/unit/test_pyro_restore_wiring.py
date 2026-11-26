"""#1760: the FC's wiring of the pyro restore policy (flight_computer/main/pyro_restore_policy.h).

The policy is host-tested in tests_cpp/test_pyro_restore_policy.cpp, with a model of a fire, the 10 Hz snapshots and
the snapshot on the pass a channel leaves Idle. What broke was main.cpp: the snapshot recorded a channel only once it
was Done, and the restore set every other channel back to Idle, so a fire whose own short reset the FC repeated on
every boot. main.cpp is not host-built, so these tests read the source.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")


def read():
    with open(MAIN_CPP) as fh:
        return fh.read()


def body(src, signature):
    # the definition, not a forward declaration: the line that starts with the
    # signature and is followed by its opening brace
    i = src.index(signature)
    while True:
        eol = src.index("\n", i)
        if src.startswith("\n{\n", eol):
            j = eol + 3
            return src[j:src.index("\n}\n", j)]
        i = src.index(signature, eol)


def code(text):
    return "\n".join(line.split("//", 1)[0] for line in text.splitlines())


def test_the_snapshot_records_each_channel_through_the_policy():
    fn = code(body(read(), "static void buildFlightSnapshot("))
    for n in range(4):
        assert re.search(r"snap\.pyro%d_fired\s*=\s*PyroRestorePolicy::snapshotValue\(pyroPhase\(pyro_ch\[%d\]\.state\)\);"
                         % (n + 1, n), fn), "pyro%d_fired" % (n + 1)
    assert "PyroChState::Done) ? 1 : 0" not in fn


def test_every_channel_state_maps_to_a_phase():
    fn = code(body(read(), "static PyroRestorePolicy::Phase pyroPhase(PyroChState s)"))
    for state, phase in (("ArmSettle", "Settling"), ("Firing", "Firing"), ("Done", "Done")):
        assert re.search(r"case PyroChState::%s:\s*return PyroRestorePolicy::Phase::%s;" % (state, phase), fn), state
    assert "return PyroRestorePolicy::Phase::Idle;" in fn


def test_a_channel_leaving_idle_sends_a_snapshot_outside_the_lock():
    fn = code(body(read(), "static void servicePyroChannels(uint32_t now_ms)"))
    # the one place a channel leaves Idle marks it
    starts = [m.start() for m in re.finditer(r"ch\.state\s*=\s*PyroChState::ArmSettle;", fn)]
    assert len(starts) == 1, "a channel leaves Idle somewhere that does not mark it"
    mark = fn.index("left_idle[i]      = true;")
    assert starts[0] < mark < fn.index("}", starts[0])
    owe = re.search(r"if \(PyroRestorePolicy::edgeSnapshotDue\(left_idle\)\) pyro_edge_snapshot_owed = true;", fn)
    assert owe, "a channel leaving Idle does not owe the OC a snapshot"
    send = re.search(r"if \(pyro_edge_snapshot_owed && saveFlightSnapshot\(now_ms\)\) \{\s*pyro_edge_snapshot_owed = false;", fn)
    assert send, "the owed snapshot is not sent, or not retried until the queue takes it"
    assert owe.start() < send.start()
    # after the pyro spinlock is released (saveFlightSnapshot takes it again)
    assert fn.rindex("portEXIT_CRITICAL(&pyro_spinlock);", 0, owe.start()) > fn.rindex(
        "portENTER_CRITICAL(&pyro_spinlock);", 0, owe.start())


def test_the_snapshot_reports_whether_the_queue_took_it():
    fn = code(body(read(), "static bool saveFlightSnapshot(uint32_t now_ms)"))
    assert re.search(r"return enqueueI2STx\(SNAPSHOT_MSG,", fn)


def test_the_restore_keeps_a_started_or_fired_channel_off():
    src = code(read())
    i = src.index("const uint8_t fired_snap[4] = { snap.pyro1_fired, snap.pyro2_fired,")
    block = src[i:src.index("pyro_last_fire_ms      = now_ms;", i)]
    assert re.search(r"spent\[i\]\s*=\s*PyroRestorePolicy::restoredSpent\(fired_snap\[i\]\);", block)
    assert re.search(r"pyro_ch\[i\]\.state\s*=\s*spent\[i\] \? PyroChState::Done : PyroChState::Idle;", block)
    assert "fired_snap[i] ? PyroChState::Done" not in block
    assert re.search(r"pyro_fired_this_flight = spent\[0\] \|\| spent\[1\] \|\| spent\[2\] \|\| spent\[3\];", block)


def test_the_pulse_waits_for_the_frame_that_records_it():
    fn = code(body(read(), "static void servicePyroChannels(uint32_t now_ms)"))
    gate = re.search(r"if \(ch\.state == PyroChState::ArmSettle &&\s*PyroRestorePolicy::pulseMayStart\(now_ms - ch\.phase_start_ms,\s*"
                     r"config::PYRO_ARM_SETTLE_MS,\s*ch\.edge_on_wire, now_ms - ch\.edge_wire_ms,\s*"
                     r"i2s_stream\.txRingUs\(\) / 1000u \+ 1u\)\) \{", fn)
    assert gate, "ArmSettle -> Firing does not wait for the frame"
    assert fn.count("ch.state          = PyroChState::Firing;") == 1, "another way into Firing"
    assert gate.start() < fn.index("ch.state          = PyroChState::Firing;")
    seen = re.search(r"\(int32_t\)\(i2s_snap_written\.load\(\) - ch\.edge_seq\) >= 0\) \{\s*ch\.edge_on_wire = true;\s*"
                     r"ch\.edge_wire_ms = now_ms;", fn)
    assert seen and seen.start() < gate.start(), "the frame being written is not noticed before the gate"
    # the frame is recorded against every channel still settling without one
    tail = fn[fn.index("pyro_edge_snapshot_owed = false;"):]
    assert re.search(r"const uint32_t seq = i2s_snap_queued\.load\(\);", tail)
    assert re.search(r"pyro_ch\[i\]\.state == PyroChState::ArmSettle && pyro_ch\[i\]\.edge_seq == 0\)\s*"
                     r"pyro_ch\[i\]\.edge_seq = seq;", tail)
    # a channel leaving Idle starts with no frame
    start = fn.index("ch.state          = PyroChState::ArmSettle;")
    assert re.search(r"ch\.edge_seq\s*= 0;\s*ch\.edge_on_wire\s*= false;", fn[start:start + 300])


def test_both_ends_of_the_queue_count_snapshots():
    src = code(read())
    enq = body(src, "static inline bool enqueueI2STx(uint8_t type, const uint8_t *payload, size_t len)")
    assert re.search(r"if \(type == SNAPSHOT_MSG\) i2s_snap_queued\.fetch_add\(1\);", enq)
    sender = body(src, "static void i2sSenderTask(void *)")
    assert re.search(r"err = i2sSendWithStats\(msg\.type, msg\.payload, msg\.len\);\s*"
                     r"if \(msg\.type == SNAPSHOT_MSG\) i2s_snap_written\.fetch_add\(1\);", sender)


def test_a_new_flight_owes_no_old_frame():
    src = code(read())
    assert "pyro_edge_snapshot_owed = false;" in body(src, "static void enterInflight(uint32_t now_ms, const char* from_state)")
    assert "pyro_edge_snapshot_owed = false;" in body(src, "static void resetFlightStateForSim(const char* edge)")

