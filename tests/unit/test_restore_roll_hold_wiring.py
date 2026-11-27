"""#1769: after an FC reboot in flight, main.cpp holds the fins until the restored filter's velocity is trusted.

The rules (flight_computer/main/restore_control_policy.h), a restored flight with the real servo controller and nav
adapter, and a reboot in the boost through the real raw filter are host-tested in
tests_cpp/test_restore_control_policy.cpp and tests_cpp/test_raw_gnss_shadow.cpp. main.cpp is not host-built, so these
tests read the source:
- the restore arms the hold with the 500 ms settle the host test uses, and latches whether it brought a filter back;
- INFLIGHT releases it only by RestoreControlPolicy::holdFins on the filter's velocity variance against the EKF row's
  EKF_VEL_VAR_OK, before the roll chain reads it, whatever servo_en says; nothing else releases it in flight;
- roll control's filter gate is filterUsableForControl: never a filter started in flight after a filterless restore;
- the speed gate does not open on a speed the hold has not found trusted, and its warning does not blame it for the
  hold;
- angle mode is off on a restored flight;
- both filters' restores start outside the trusted velocity (the static checks), and the host tests' constants are
  config.h's;
- no preprocessor branch keeps any of it out of a build.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_DIR = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main")
MAIN_CPP = os.path.join(MAIN_DIR, "main.cpp")
CONFIG_H = os.path.join(MAIN_DIR, "config.h")
POLICY_TEST = os.path.join(ROOT, "tests_cpp", "test_restore_control_policy.cpp")
SHADOW_TEST = os.path.join(ROOT, "tests_cpp", "test_raw_gnss_shadow.cpp")

RELEASE = ("if (reboot_recovery) { float var_vel[3]; ekf.getCovVel(var_vel); "
           "const bool vel_trusted = RestoreControlPolicy::velocityTrusted(ekf.isHealthy(), var_vel, "
           "config::EKF_VEL_VAR_OK); "
           "if (!RestoreControlPolicy::holdFins(now_ms, servo_settle_end_ms, restore_had_filter, vel_trusted)) { "
           "reboot_recovery = false;")
ARM = ("reboot_recovery = true; reboot_recovery_telem = true; restore_hold_noted = false; "
       "restore_had_filter = ekf_initialized; servo_settle_end_ms = now_ms + 500;")
CHAIN_HOLD = "else if (reboot_recovery || t_since_launch_ms < roll_delay_ms || !roll_speed_gate_open || orient_frame_disproved)"
GATE = ("else if (ism6_fresh && !reboot_recovery && RestoreControlPolicy::filterUsableForControl(ekf_initialized, true, "
        "reboot_recovery_telem, restore_had_filter))")
GATE_WARN = ("if (!roll_speed_gate_open && !roll_speed_gate_warned && !reboot_recovery && "
             "t_since_launch_ms >= config::ROLL_CONTROL_SPEED_GATE_WARN_MS)")
CTRL = ("const bool ekf_ctrl_healthy = RestoreControlPolicy::filterUsableForControl(ekf_initialized, ekf.isHealthy(), "
        "reboot_recovery_telem, restore_had_filter);")
ANGLE = ("const bool angle_mode_active = RestoreControlPolicy::angleModeAllowed(use_angle_control, "
         "ekf_ctrl_healthy, reboot_recovery_telem);")


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


def function_body(src, signature):
    i = src.index(signature)
    j = src.index("{", i)
    depth, k = 0, j
    while True:
        if src[k] == "{":
            depth += 1
        elif src[k] == "}":
            depth -= 1
            if depth == 0:
                return src[j + 1:k]
        k += 1


def cfg(name, kind=r"(?:float|double|uint32_t)"):
    m = re.search(r"static constexpr " + kind + r"\s+" + name + r"\s*=\s*([0-9.]+)f?;", read(CONFIG_H))
    assert m, name
    return float(m.group(1))


def test_the_restore_arms_the_hold_with_the_settle():
    src = code(read())
    setup = function_body(src, "static void setup_fc()")
    assert setup.count(ARM) == 1
    assert src.count("reboot_recovery = true;") == 1, "armed only by the restore"
    assert src.count("reboot_recovery_telem = true;") == 1
    # the restore's filter is latched there, and only there
    assert len(re.findall(r"\brestore_had_filter\s*[-+*/&|^]?=(?!=)", src)) == 2   # the declaration and the arm
    assert setup.index("ekf_initialized = snap.ekf_initialized != 0;") < setup.index(ARM)
    m = re.search(r"constexpr uint32_t kSettleMs = (\d+);\s*// main.cpp: servo_settle_end_ms = now_ms \+ 500",
                  read(POLICY_TEST))
    assert m and m.group(1) == "500"


def test_inflight_releases_it_only_on_a_trusted_velocity():
    src = code(read())
    assert src.count(RELEASE) == 1
    i = src.index(RELEASE)
    case = src.rindex("case INFLIGHT:", 0, i)
    assert src.index("case LANDED:", case) > i, "in INFLIGHT"
    assert "now_ms >= servo_settle_end_ms" not in src, "the time-only release is gone"
    # before the roll chain reads it, on the same pass
    assert i < src.index(CHAIN_HOLD, i)
    # in the INFLIGHT case's own block: no `if (servo_enabled)` (or anything else) gates it
    depth, k = 0, i
    while True:
        k -= 1
        if src[k] == "}":
            depth += 1
        elif src[k] == "{":
            if depth == 0:
                break
            depth -= 1
    assert src[:k].rstrip().endswith("case INFLIGHT:"), src[max(0, k - 80):k + 1]
    # in flight nothing else releases it: the release, the sim reset and LANDED's one-shot (and the declaration),
    # however the assignment is spelled
    assert src.count("static bool reboot_recovery = false;") == 1
    assert len(re.findall(r"\breboot_recovery\s*[-+*/&|^]?=(?!=)", src)) == 5
    assert src.count("reboot_recovery = false;") == 4
    # and nothing else reads the settle's end: its declaration, the arm, the release and the hold's log
    assert len(re.findall(r"\bservo_settle_end_ms\b", src)) == 4
    assert function_body(src, "static void resetFlightStateForSim(").count("reboot_recovery = false;") == 1
    loop = function_body(src, "static void loop_fc()")
    assert loop.count("reboot_recovery = false;") == 2
    landed = loop[loop.index("case LANDED:"):]
    assert "reboot_recovery = false; reboot_recovery_telem = false;" in landed


def test_the_chain_still_holds_the_fins_on_the_hold():
    src = code(read())
    i = src.index(CHAIN_HOLD)
    j = src.index("{", i)
    body = src[j + 1:src.index("}", j)]
    assert body.strip().startswith("servo_control.stowControl(); servo_control.resetPID();")


def test_roll_control_never_runs_on_a_filter_started_after_a_filterless_restore():
    src = code(read())
    assert src.count(CTRL) == 1
    assert "const bool ekf_ctrl_healthy = ekf_initialized && ekf.isHealthy();" not in src
    i = src.index(CTRL)
    assert src.index(CHAIN_HOLD) < i < src.index("else if (gain_sched_enabled && ekf_ctrl_healthy)", i)


def test_the_speed_gate_waits_for_the_hold():
    src = code(read())
    assert src.count(GATE) == 1
    assert "else if (ekf_initialized && ism6_fresh) {" not in src
    assert src.count(GATE_WARN) == 1
    assert src.index(GATE) < src.index(GATE_WARN) < src.index(CHAIN_HOLD)


def test_angle_mode_is_off_on_a_restored_flight():
    src = code(read())
    assert src.count(ANGLE) == 1
    assert "use_angle_control && ekf_ctrl_healthy;" not in src
    assert src.index(ANGLE) < src.index("if (angle_mode_active && seg.mode == ROLL_SEG_ANGLE)")


def test_both_restores_start_outside_the_trusted_velocity():
    src = code(read())
    assert ("static_assert(config::RAW_RESTORE_VEL_SIGMA_MPS * config::RAW_RESTORE_VEL_SIGMA_MPS > "
            "4.0 * config::EKF_VEL_VAR_OK,") in src
    gins = "for (int i = 3; i < 6; i++) ekf.inflateCovDiag(i, 25.0f);"
    assert src.count(gins) == 1
    assert src.index("static_assert(25.0f > 4.0f * config::EKF_VEL_VAR_OK,") > src.index(gins)


def test_the_host_tests_constants_are_configs():
    policy, shadow = read(POLICY_TEST), read(SHADOW_TEST)

    def const(text, name, comment):
        m = re.search(r"constexpr (?:float|double|uint32_t) " + name + r" = ([0-9.]+)f?;\s*// " + re.escape(comment),
                      text)
        assert m, name
        return float(m.group(1))

    assert const(policy, "kVarOk", "config::EKF_VEL_VAR_OK") == cfg("EKF_VEL_VAR_OK")
    assert const(policy, "kVRef", "config::GAIN_SCHEDULE_V_REF") == cfg("GAIN_SCHEDULE_V_REF")
    assert const(policy, "kVMin", "config::GAIN_SCHEDULE_V_MIN") == cfg("GAIN_SCHEDULE_V_MIN")
    assert const(policy, "kRestoreVelSigma", "config::RAW_RESTORE_VEL_SIGMA_MPS") == cfg("RAW_RESTORE_VEL_SIGMA_MPS")
    assert const(shadow, "kRestoreVarOk", "config::EKF_VEL_VAR_OK") == cfg("EKF_VEL_VAR_OK")
    for name, key in (("kRestorePosSigmaM", "RAW_RESTORE_POS_SIGMA_M"), ("kRestoreVelSigmaMps", "RAW_RESTORE_VEL_SIGMA_MPS"),
                      ("kRestoreAttSigmaDeg", "RAW_RESTORE_ATT_SIGMA_DEG")):
        assert const(shadow, name, "config::" + key) == cfg(key), name
    # the shadow test's schedule is the controller's: V_REF over the floored speed, squared, capped at 3
    assert "const double r = 50.0 / std::max(std::fabs(v), 25.0);" in shadow
    assert "return std::min(r * r, 3.0);" in shadow
    assert "GAIN_SCHEDULE_SCALE_CAP = 3.0f;" in read(os.path.join(ROOT, "tinkerrocket-idf", "components",
                                                                   "TR_ServoControl_ledc_mult",
                                                                   "TR_ServoControl_ledc_mult.h"))


def test_no_preprocessor_branch_keeps_it_out_of_a_build():
    """Every site sits outside any #if: compiled into every flight build."""
    raw = read()
    for m in re.finditer(r"RestoreControlPolicy::filterUsableForControl\(", raw):
        assert pp_stack(raw, m.start()) == []
    assert len(re.findall(r"RestoreControlPolicy::filterUsableForControl\(", raw)) == 2
    for needle in ('#include "restore_control_policy.h"', "static bool     restore_hold_noted",
                   "restore_hold_noted = false;\n            restore_had_filter = ekf_initialized;",
                   "static bool     restore_had_filter",
                   "RestoreControlPolicy::velocityTrusted(", "RestoreControlPolicy::holdFins(",
                   "else if (ism6_fresh && !reboot_recovery &&",
                   "RestoreControlPolicy::angleModeAllowed(", "static_assert(25.0f > 4.0f * config::EKF_VEL_VAR_OK,"):
        assert raw.count(needle) == 1, needle
        assert pp_stack(raw, raw.index(needle)) == [], needle
    # the raw filter's own check sits with its restore sigmas, in the raw images
    i = raw.index("static_assert(config::RAW_RESTORE_VEL_SIGMA_MPS")
    assert pp_stack(raw, i) == ["#if TR_RAW_GNSS_SHADOW", "#if TR_RAW_NAV_P4"]
