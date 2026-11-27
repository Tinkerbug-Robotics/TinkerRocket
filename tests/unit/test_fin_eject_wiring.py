"""#1768: once the eject occurs, main.cpp cuts the fins' pulse train and servo rail for the rest of the flight.

The policy (flight_computer/main/fin_eject_policy.h) and a flight with the real servo controller are host-tested in
tests_cpp/test_fin_eject_policy.cpp. main.cpp is not host-built, so these tests read the source:
- the INFLIGHT case steps the policy once a pass, on the deployment detected after the apogee call, a deployment
  charge fired or a cut restored from the snapshot, after the pass's pyro service and before the servo block,
  unconditionally, and drives its cut with the servo controller and the rail switch;
- the servo block's branch chain puts the cut first, ahead of the hold and of every roll law and guidance: it holds
  guidance and commands no fin; nothing else in the INFLIGHT case commands the fins;
- roll control past apogee is untouched: nothing in the servo block reads the apogee call;
- the charge flag is set where a charge starts its fire, restored from the spent channels, cleared only at launch;
- the rail switch writes SERVO_ACT only, cuts it only where the board switches the supply, is cut only through the
  policy, and is raised at launch and a sim's reset, the outputs unlocked only after it;
- the snapshot carries the cut, and the restore reads it back;
- the policy's state is reset at launch, a sim's reset and a restore, and nothing else writes it;
- LANDED never re-drives fins the eject cut;
- no preprocessor branch keeps any of it out of a build.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")

EJECTED = ("const bool ejected = (deployment_detected && pyro_apogee_detected) || pyro_charge_fired || "
           "fin_cut_restored;")
STEP = "FinEjectPolicy::step(fin_eject, ejected, now_ms, config::SERVO_RAIL_CUT_DELAY_MS)"
DRIVE = "FinEjectPolicy::drive(fins, servo_control, servoRailSet);"
HOLD_IF = ("reboot_recovery || t_since_launch_ms < roll_delay_ms || !roll_speed_gate_open || "
           "orient_frame_disproved")
# Every call that moves the fins or changes whether they are driven.
FIN_DRIVES = ("servo_control.controlAngle(", "servo_control.controlWithGainSchedule(", "servo_control.control(",
              "servo_control.controlToSetpoint(", "servo_control.setServoAngles(", "servo_control.setPulseChannel(",
              "servo_control.beginNeutralSettle(", "servo_control.serviceNeutralSettle(",
              "servo_control.beginWiggle(", "servo_control.stowControl(", "servo_control.idle(",
              "servo_control.lockOutputs(", "servoRailSet(",
              "gpio_set_level((gpio_num_t)config::SERVO_ACT_PIN")


def read(path=MAIN_CPP):
    with open(path) as fh:
        return fh.read()


def code(text):
    """The source without comments (string and character literals kept whole), each whitespace run collapsed
    to one space."""
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if text.startswith("//", i):
            i = text.find("\n", i)
            i = n if i < 0 else i
        elif text.startswith("/*", i):
            i = text.index("*/", i + 2) + 2
            out.append(" ")
        elif c in "\"'":
            j = i + 1
            while text[j] != c:
                j += 2 if text[j] == "\\" else 1
            out.append(text[i:j + 1])
            i = j + 1
        else:
            out.append(c)
            i += 1
    return re.sub(r"\s+", " ", "".join(out))


def match_close(src, i):
    """Index just past the bracket that closes the one at src[i]."""
    pairs = {"(": ")", "{": "}", "[": "]"}
    stack = [pairs[src[i]]]
    j = i + 1
    while stack:
        c = src[j]
        if c in pairs:
            stack.append(pairs[c])
        elif c in ")}]":
            assert c == stack.pop(), f"unbalanced at {j}"
        j += 1
    return j


def if_chain(src, i):
    """The if/else-if/else chain starting at the `if` at src[i]: a list of (condition or None, body), and the
    index just past the chain."""
    chain = []
    while True:
        assert src.startswith("if", i), src[i:i + 40]
        po = src.index("(", i)
        pc = match_close(src, po)
        cond = src[po + 1:pc - 1].strip()
        bo = pc + (1 if src[pc] == " " else 0)
        assert src[bo] == "{", f"if without braces: {src[i:bo + 20]}"
        bc = match_close(src, bo)
        chain.append((cond, src[bo + 1:bc - 1]))
        k = bc + (1 if src[bc:bc + 1] == " " else 0)
        if not src.startswith("else", k):
            return chain, bc
        k += 4
        k += 1 if src[k] == " " else 0
        if src.startswith("if", k):
            i = k
            continue
        assert src[k] == "{"
        ec = match_close(src, k)
        chain.append((None, src[k + 1:ec - 1]))
        return chain, ec


def block_at(src, i):
    """The braced block of the statement starting at src[i] (an `if`), and the index just past it."""
    bo = match_close(src, src.index("(", i))
    bo += 1 if src[bo] == " " else 0
    assert src[bo] == "{"
    bc = match_close(src, bo)
    return src[bo + 1:bc - 1], bc


def inflight_case(src):
    a = src.index("case INFLIGHT:")
    return a, src.index("case LANDED:", a)


def function_body(src, signature):
    i = src.index(signature)
    bo = src.index("{", i)
    return src[bo:match_close(src, bo)]


def pp_stack(raw, pos):
    stack = []
    for line in raw[:pos].splitlines():
        t = line.strip()
        if re.match(r"#\s*if", t):
            stack.append(t)
        elif re.match(r"#\s*(else|elif)", t):
            stack[-1] = "!" + stack[-1].lstrip("!")
        elif re.match(r"#\s*endif", t):
            stack.pop()
    return stack


def servo_block(src):
    """The INFLIGHT servo block: its start and end."""
    a, b = inflight_case(src)
    i = src.index("if (servo_enabled) {", src.index(STEP))
    assert a < i < b
    _, end = block_at(src, i)
    return i, end


def test_the_policy_is_stepped_once_a_pass_before_the_servo_block():
    src = code(read())
    assert '#include "fin_eject_policy.h"' in read()
    assert src.count("FinEjectPolicy::step(") == 1 and src.count(STEP) == 1 and src.count(EJECTED) == 1
    assert src.count("FinEjectPolicy::drive(") == 1
    a, b = inflight_case(src)
    at = src.index(STEP)
    assert a < at < b, "stepped in case INFLIGHT"
    assert src.index("servicePyroChannels(now_ms);", a) < at, "the pass that fires the charge cuts on it"
    drive = src.index(DRIVE)
    sb, _ = servo_block(src)
    assert at < drive < sb, "stepped and driven before the servo block"
    # The drive is the statement right after the step, unconditional, and both sit in the INFLIGHT case's own
    # block: no `if (servo_enabled)` (or anything else) gates the cut.
    assert (EJECTED + " const FinEjectPolicy::Fins fins = " + STEP + "; " + DRIVE) in src
    depth, k = 0, at
    while True:
        k -= 1
        if src[k] == "}":
            depth += 1
        elif src[k] == "{":
            if depth == 0:
                break
            depth -= 1
    assert src[:k].rstrip().endswith("case INFLIGHT:"), src[max(0, k - 80):k + 1]


def test_the_cut_comes_first_in_the_servo_block_and_commands_no_fin():
    src = code(read())
    sb, sb_end = servo_block(src)
    i = src.index("if (FinEjectPolicy::cut(fin_eject))", sb)
    assert i < sb_end
    chain, chain_end = if_chain(src, i)
    conds = [c for c, _ in chain]
    assert conds == ["FinEjectPolicy::cut(fin_eject)", HOLD_IF, "ism6_fresh", None], conds
    cut, hold, fresh, stale = (body for _, body in chain)
    assert "guidance_active = false;" in cut
    assert "guidance_inhibit_flags = GuidanceGatePolicy::GF_HELD;" in cut
    for d in FIN_DRIVES + ("guidance.", "guidance_active = true", "control_mixer."):
        assert d not in cut, d
    assert "servo_control.stowControl();" in hold and "servo_control.resetPID();" in hold
    assert "servo_control.controlWithGainSchedule(" in fresh
    assert "servo_control.stowControl();" in stale

    # Nothing else in the INFLIGHT case commands the fins: not in the servo block before the chain (the speed
    # gate), not after it (the landing transition), not anywhere else in the pass but the policy's drive.
    a, b = inflight_case(src)
    outside = src[a:i] + src[chain_end:b]
    assert outside.count(DRIVE) == 1
    outside = outside.replace(DRIVE, "")
    for d in FIN_DRIVES:
        assert d not in outside, d


def test_control_past_apogee_is_untouched():
    """Roll control and guidance run to the eject: the servo block does not read the apogee call."""
    src = code(read())
    sb, sb_end = servo_block(src)
    block = src[sb:sb_end]
    assert "pyro_apogee_detected" not in block
    assert "apogee_flag" not in block


def test_the_charge_flag_is_set_at_the_fire_restored_and_cleared_only_at_launch():
    src = code(read())
    assert src.count("static bool pyro_charge_fired = false;") == 1
    body = function_body(src, "static void servicePyroChannels(uint32_t now_ms)")
    fired = body.index("if (just_fired[i]) {")
    first, _ = block_at(body, fired)
    assert first.startswith(" pyro_charge_fired = true;"), "on the pulse's first pass"
    assert "just_fired[i] = true;" in body
    restore = src.index("pyro_charge_fired = pyro_fired_this_flight;")
    assert src.rindex("pyro_fired_this_flight = spent[0] || spent[1] || spent[2] || spent[3];", 0, restore) > \
        restore - 200, "set from the spent channels, right after them"
    launch = function_body(src, "static void enterInflight(uint32_t now_ms, const char* from_state)")
    assert launch.count("pyro_charge_fired = false;") == 1
    writes = re.findall(r"(?<![.\w])pyro_charge_fired = ([^;]+);", src.replace(
        "static bool pyro_charge_fired = false;", ""))
    assert sorted(writes) == ["false", "pyro_fired_this_flight", "true"], writes


def test_the_rail_is_cut_only_through_the_policy_and_raised_at_launch_and_a_sims_reset():
    src = code(read())
    rail = function_body(src, "static void servoRailSet(bool on)")
    assert ("if (config::SERVO_ACT_PIN >= 0 && (on || config::SERVO_ACT_SWITCHES_SUPPLY)) "
            "gpio_set_level((gpio_num_t)config::SERVO_ACT_PIN, on ? 1 : 0);") in rail
    calls = re.findall(r"servoRailSet\(([^)]*)\)", src.replace("static void servoRailSet(bool on)", ""))
    assert sorted(calls) == ["true", "true"], calls   # the drive takes it by name
    for sig in ("static void enterInflight(uint32_t now_ms, const char* from_state)",
                "static void resetFlightStateForSim(const char* edge)"):
        body = function_body(src, sig)
        assert "FinEjectPolicy::reset(fin_eject); servoRailSet(true); servo_control.lockOutputs(false);" in body, sig
    # The outputs are locked only through the policy, and unlocked only at those two raises.
    assert "lockOutputs(true)" not in src
    assert src.count("servo_control.lockOutputs(false);") == 2
    # The rail waits out the pulse under way: two PWM periods at the slowest servo rate.
    assert ("static_assert(config::SERVO_RAIL_CUT_DELAY_MS * (uint32_t)TR_ServoControl::kMinServoHz >= 2u * 1000u,"
            in src)
    # SERVO_ACT is written only by the boot's raise and by servoRailSet.
    sets = [m.start() for m in re.finditer(r"gpio_set_level\(\(gpio_num_t\)config::SERVO_ACT_PIN", src)]
    assert len(sets) == 2


def test_the_snapshot_carries_the_cut_and_the_restore_reads_it():
    src = code(read())
    assert ("snap.servo_enabled = (uint8_t)((servo_enabled ? SNAP_SERVO_ENABLED : 0u) | "
            "(FinEjectPolicy::cut(fin_eject) ? SNAP_SERVO_FINS_CUT : 0u));") in src
    assert "servo_enabled = (snap.servo_enabled & SNAP_SERVO_ENABLED) != 0;" in src
    restore = src.index("fin_cut_restored = (snap.servo_enabled & SNAP_SERVO_FINS_CUT) != 0;")
    rebase = src.index("launch_time_millis = now_ms - snap.flight_elapsed_ms;")
    assert rebase < restore and not re.search(r"\bstatic \w+ \w+\(", src[rebase:restore])
    assert src.count("static bool fin_cut_restored = false;") == 1
    launch = function_body(src, "static void enterInflight(uint32_t now_ms, const char* from_state)")
    assert "fin_cut_restored = false;" in launch
    writes = re.findall(r"(?<![.\w])fin_cut_restored = ([^;]+);", src.replace("static bool fin_cut_restored = false;", ""))
    assert sorted(writes) == ["(snap.servo_enabled & SNAP_SERVO_FINS_CUT) != 0", "false"], writes


def test_the_state_is_reset_at_launch_a_sims_reset_and_a_restore_and_nowhere_else():
    src = code(read())
    assert src.count("static FinEjectPolicy::State fin_eject;") == 1
    assert src.count("FinEjectPolicy::reset(fin_eject);") == 3
    restore = src.index("FinEjectPolicy::reset(fin_eject); if (snap.pyro_apogee_detected) {")
    rebase = src.index("launch_time_millis = now_ms - snap.flight_elapsed_ms;")
    assert rebase < restore and not re.search(r"\bstatic \w+ \w+\(", src[rebase:restore])
    uses = [m.start() for m in re.finditer(r"\bfin_eject\b", src)]
    allowed = {src.index("static FinEjectPolicy::State fin_eject;") + len("static FinEjectPolicy::State "),
               src.index(STEP) + len("FinEjectPolicy::step(")}
    allowed |= {m.start() + len("FinEjectPolicy::reset(") for m in re.finditer(r"FinEjectPolicy::reset\(fin_eject\)", src)}
    allowed |= {m.start() + len("FinEjectPolicy::cut(") for m in re.finditer(r"FinEjectPolicy::cut\(fin_eject\)", src)}
    assert set(uses) == allowed, "fin_eject: declared, stepped, reset, read -- nothing else"


def test_landed_never_redrives_fins_the_eject_cut():
    # #1770 moved the rule into ServoPadPolicy::relaxAtLanding (host-tested there): the cut relaxes.
    src = code(read())
    assert ("if (ServoPadPolicy::relaxAtLanding(servo_enabled, config::SERVO_RELAX_ON_PAD, "
            "FinEjectPolicy::cut(fin_eject))) servo_control.idle(); else servo_control.stowControl();") in src


def test_no_preprocessor_branch_keeps_it_out_of_a_build():
    raw = read()
    for needle in ("FinEjectPolicy::step(", "FinEjectPolicy::drive(", "static FinEjectPolicy::State fin_eject;",
                   "static void servoRailSet(bool on)", "static bool          pyro_charge_fired",
                   "static bool fin_cut_restored = false;", '#include "fin_eject_policy.h"'):
        assert raw.count(needle) == 1, needle
        stack = pp_stack(raw, raw.index(needle))
        assert not [c for c in stack if re.search(r"TR_RAW_NAV|TR_GNSS|TR_BENCH|TR_BOARD|RAW_", c)], (needle, stack)
    for m in re.finditer(r"FinEjectPolicy::reset\(fin_eject\);|pyro_charge_fired = ", raw):
        stack = pp_stack(raw, m.start())
        assert not [c for c in stack if re.search(r"TR_RAW_NAV|TR_GNSS|TR_BENCH|TR_BOARD|RAW_", c)], stack


def test_only_boards_that_switch_the_servo_supply_cut_the_rail():
    """The V9 and V10 boards' U28 switches the supply; the V7's Q9 the return; the V8's is unconfirmed."""
    board = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "board")
    want = {"board_v10.h": "true", "board_v9.h": "true", "board_m2.h": "false",
            os.path.join("legacy", "board_v5.h"): "false", os.path.join("legacy", "board_v7.h"): "false",
            os.path.join("legacy", "board_v8.h"): "false"}
    for name, value in want.items():
        text = read(os.path.join(board, name))
        got = re.findall(r"static constexpr bool SERVO_ACT_SWITCHES_SUPPLY = (true|false);", text)
        assert got == [value], (name, got)
    assert sorted(os.listdir(board)) == ["board_m2.h", "board_v10.h", "board_v9.h", "legacy"]
    assert sorted(os.listdir(os.path.join(board, "legacy"))) == ["board_v5.h", "board_v7.h", "board_v8.h"]
