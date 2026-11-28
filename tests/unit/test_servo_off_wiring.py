"""#1770: with servo control off, main.cpp still brings the fin servos back after a servo test.

The rules (flight_computer/main/servo_pad_policy.h) and a pad session and flight with the real servo controller are
host-tested in tests_cpp/test_servo_pad_policy.cpp. main.cpp is not host-built, so these tests read the source:
- the neutral settle is serviced once a pass, unconditionally;
- SERVO_TEST_STOP settles by ServoPadPolicy::stopSettles (a test running, or on the pad), and always ends the test;
- READY and PRELAUNCH relax by ServoPadPolicy::relaxOnPad, and LANDED by relaxAtLanding;
- the #363 launch failsafe and the mag-cal start stow by stowOnCancel, read before they clear the test flags, and
  arm the wake window;
- the servo replay's stop arms the wake window; servo control's disable cancels the boot sweep and arms it;
- the host test's wake window is config.h's;
- no preprocessor branch keeps any of it out of a build.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_DIR = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main")
MAIN_CPP = os.path.join(MAIN_DIR, "main.cpp")
CONFIG_H = os.path.join(MAIN_DIR, "config.h")
HOST_TEST = os.path.join(ROOT, "tests_cpp", "test_servo_pad_policy.cpp")

WAKE = "servo_pad_wake_until_ms = now_ms + config::SERVO_PAD_TRIM_WAKE_MS;"
RELAX = ("if (ServoPadPolicy::relaxOnPad(servo_enabled, config::SERVO_RELAX_ON_PAD, servo_control.isWiggling(), "
         "servo_control.isNeutralSettling(), now_ms, servo_pad_wake_until_ms)) servo_control.idle();")
LANDED = ("if (ServoPadPolicy::relaxAtLanding(servo_enabled, config::SERVO_RELAX_ON_PAD, "
          "FinEjectPolicy::cut(fin_eject))) servo_control.idle(); else servo_control.stowControl();")


def read(path=MAIN_CPP):
    with open(path) as fh:
        return fh.read()


def code(text):
    """The source without comments (string and character literals kept whole), each whitespace run collapsed
    to one space, none just inside a parenthesis (so a call's line breaks do not matter)."""
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
    text = re.sub(r"\s+", " ", "".join(out))
    return re.sub(r"\s+\)", ")", re.sub(r"\(\s+", "(", text))


def match_close(src, i):
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


def block_at(src, i):
    """The braced block of the `if`/`else if` statement starting at src[i]."""
    bo = match_close(src, src.index("(", i))
    bo += 1 if src[bo] == " " else 0
    assert src[bo] == "{", src[i:bo + 20]
    return src[bo + 1:match_close(src, bo) - 1]


def case_body(src, label, next_label):
    a = src.index(label)
    return src[a:src.index(next_label, a)]


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


def test_the_settle_is_serviced_once_a_pass_unconditionally():
    src = code(read())
    call = "servo_control.serviceNeutralSettle(now_ms);"
    assert src.count("serviceNeutralSettle(") == 1 and src.count(call) == 1
    i = src.index(call)
    assert src[:i].rstrip().endswith((";", "}", "{")), src[i - 80:i]
    # Before the state switch, inside the state-machine branch (no test mode running).
    assert i < src.index("switch (rocket_state)", i)
    assert src.rindex("if (post_flight_lockout) rocket_state = LANDED;", 0, i) > i - 200


def test_the_servo_test_stop_settles_by_the_policy():
    src = code(read())
    i = src.index("else if (out_pending_command == SERVO_TEST_STOP)")
    body = block_at(src, i + len("else "))
    assert "const bool on_pad = rocket_state == READY || rocket_state == PRELAUNCH;" in body
    j = body.index("if (ServoPadPolicy::stopSettles(servo_test_active, on_pad)) {")
    settle = block_at(body, j)
    assert "servo_control.beginNeutralSettle(now_ms);" in settle and WAKE in settle
    assert body.count("beginNeutralSettle(") == 1
    assert body.rstrip().endswith("servo_test_active = false;"), "the test ends after the check"
    assert body.count("servo_test_active = false;") == 1, "and not before it"


def test_ready_and_prelaunch_relax_by_the_policy():
    src = code(read())
    assert src.count(RELAX) == 2
    ready = case_body(src, "case READY:", "case PRELAUNCH:")
    prelaunch = case_body(src, "case PRELAUNCH:", "case INFLIGHT:")
    assert ready.count(RELAX) == 1 and prelaunch.count(RELAX) == 1
    assert "servo_enabled && config::SERVO_RELAX_ON_PAD" not in src, "the old servo-on-only relax is gone"


def test_landed_relaxes_by_the_policy_whatever_servo_en_says():
    src = code(read())
    assert src.count(LANDED) == 1
    i = src.index(LANDED)
    # at the top of the one-shot LANDED actions, not under `if (servo_enabled)`
    start = src.index("if (!landed_actions_done) {", src.index("case LANDED:"))
    depth = 0
    for c in src[start:i]:
        depth += (c == "{") - (c == "}")
    assert depth == 1, "not under `if (servo_enabled)`"


STOW_RULE = "ServoPadPolicy::stowOnCancel(servo_enabled, servo_test_active, servo_replay_active);"
CLEAR = "ground_test_active = false; servo_test_active = false; servo_replay_active = false;"


def test_the_launch_failsafe_stows_by_the_policy_and_arms_the_wake():
    src = code(read())
    i = src.index("if (TestModeGatePolicy::launchFailsafeShouldCancel(kinematics.launch_flag, "
                  "ground_test_active || servo_test_active || servo_replay_active, post_flight_lockout))")
    body = block_at(src, i)
    assert ("const bool stow = " + STOW_RULE + " " + CLEAR) in body, "the rule is read before the flags clear"
    assert body.rstrip().endswith("if (stow) { servo_control.stowControl(); " + WAKE + " }"), body
    assert body.count("stowControl(") == 1


def test_the_mag_cal_start_stows_by_the_policy_and_arms_the_wake():
    src = code(read())
    i = src.index("const bool mag_cal_stow = " + STOW_RULE)
    assert src.index(CLEAR, i) == i + len("const bool mag_cal_stow = " + STOW_RULE) + 1
    j = src.index("if (mag_cal_stow) {", i)
    body = block_at(src, j)
    assert body.strip().startswith("servo_control.stowControl(); " + WAKE)
    assert "rocket_state = MAG_CALIBRATION;" in src[i:j]
    assert "if (servo_enabled) { servo_control.stowControl(); ESP_LOGI(TAG, \"[MAGCAL]" not in src


def test_servo_controls_disable_cancels_the_sweep_and_arms_the_wake():
    src = code(read())
    i = src.index("else if (out_pending_command == SERVO_CTRL_DISABLE)")
    body = block_at(src, i + len("else "))
    assert body.strip().startswith("servo_enabled = false; servo_control.cancelWiggle(); "
                                   "servo_control.stowControl(); " + WAKE)


def test_the_replay_stop_arms_the_wake():
    src = code(read())
    i = src.index("else if (out_pending_command == SERVO_REPLAY_STOP)")
    body = block_at(src, i + len("else "))
    assert "servo_control.stowControl(); servo_control.resetPID(); " + WAKE in body


def test_the_host_tests_wake_window_is_configs():
    cfg = re.search(r"static constexpr uint32_t SERVO_PAD_TRIM_WAKE_MS = (\d+);", read(CONFIG_H))
    host = re.search(r"constexpr uint32_t kWakeMs = (\d+);\s*// config::SERVO_PAD_TRIM_WAKE_MS", read(HOST_TEST))
    assert cfg and host and cfg.group(1) == host.group(1)


def test_no_preprocessor_branch_keeps_it_out_of_a_build():
    """Every site sits outside any #if: compiled into every flight build."""
    raw = read()
    for needle in ('#include "servo_pad_policy.h"', "servo_control.serviceNeutralSettle(now_ms);",
                   "ServoPadPolicy::relaxAtLanding(", "else if (out_pending_command == SERVO_TEST_STOP)",
                   "ServoPadPolicy::stopSettles(", "const bool stow = ServoPadPolicy::stowOnCancel(",
                   "const bool mag_cal_stow", "else if (out_pending_command == SERVO_CTRL_DISABLE)",
                   "else if (out_pending_command == SERVO_REPLAY_STOP)"):
        assert raw.count(needle) == 1, needle
        assert pp_stack(raw, raw.index(needle)) == [], needle
    for m in re.finditer(r"ServoPadPolicy::relaxOnPad\(", raw):
        assert pp_stack(raw, m.start()) == []
