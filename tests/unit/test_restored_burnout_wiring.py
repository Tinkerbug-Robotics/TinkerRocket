"""#1776: main.cpp's wiring of the restored-burnout rule (RestoredBurnout.h).

A flight restored from the flight token's launch-edge frame comes back with burnout unset, and a section hanging nose-up
under the drogue never gives the live detector its negative axial specific force, so the apogee vote never ran.
RestoredBurnout (host-tested with the real kinematic checks in tests_cpp/test_restored_burnout.cpp) decides when this
boot's own evidence shows the vehicle past thrust. main.cpp has to step it where the recovery arm gate is stepped, on
the gate's own inputs, only while burnout is unset, and latch burnout and its time when it says so; a restore starts
it fresh. main.cpp is not host-built, so these tests read the source.
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


def gate_block(src):
    """The `if (recovery_gate_active) { ... }` block that steps the recovery arm gate."""
    blocks = [block_at(src, m.end() - 1) for m in re.finditer(r"if \(recovery_gate_active\)\s*\{", src)]
    stepping = [b for b in blocks if "RecoveryArmGate::step(recovery_arm_state, r);" in b]
    assert len(stepping) == 1, "one block steps the gate"
    return stepping[0]


def test_the_header_is_included():
    with open(MAIN_CPP) as fh:
        assert "#include <RestoredBurnout.h>" in fh.read()


def test_the_rule_steps_inside_the_recovery_gate_block_after_the_gate_on_its_inputs():
    src = code()
    block = gate_block(src)
    calls = re.findall(r"RestoredBurnout::step\(([^;]*)\);", src)
    assert len(calls) == 1, "one step of the rule"
    args = [a.strip() for a in calls[0].split(",")]
    assert args == ["restored_burnout_state", "!burnout_detected", "recovery_arm_state", "r"]
    assert "RestoredBurnout::step(" in block, "stepped where the gate is, on a restored flight only"
    assert block.index("RecoveryArmGate::step(recovery_arm_state, r);") < block.index("RestoredBurnout::step("), \
        "the rule reads the gate's verdict after this tick's step"


def test_its_evidence_latches_burnout_and_stamps_the_time():
    src = code()
    m = re.search(r"if \(rb != RestoredBurnout::Evidence::None\)\s*\{(.*?)\}", src, flags=re.S)
    assert m, "the latch is taken on the rule's evidence"
    body = m.group(1)
    assert "burnout_detected = true;" in body
    assert "burnout_time_ms  = now_ms;" in body or "burnout_time_ms = now_ms;" in body


def test_burnout_is_set_true_only_by_the_snapshot_the_live_detector_and_the_rule():
    src = code()
    sets = [m.start() for m in re.finditer(r"\bburnout_detected\s*=\s*true\s*;", src)]
    assert len(sets) == 2, "the snapshot's restore and the rule (the live detector latches by reference)"
    contexts = [src[max(0, i - 400):i] for i in sets]
    assert any("if (snap.burnout_detected)" in c for c in contexts)
    assert any("RestoredBurnout::Evidence::None" in c for c in contexts)
    assert "tr::burnoutDetectStep(burnout_detected, burnout_neg_count," in src


def test_a_restore_starts_the_rule_fresh_with_the_gate():
    src = code()
    i = src.index("RecoveryArmGate::reset(recovery_arm_state, now_ms);\n            recovery_gate_active  = true;")
    tail = src[i:i + 600]
    assert "RestoredBurnout::reset(restored_burnout_state);" in tail
    assert src.count("RestoredBurnout::reset(") == 1
