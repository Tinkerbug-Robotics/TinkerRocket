"""#1774: the OC believes NSF_SIM_ACTIVE only once ten frames in a row carry it.

The run (out_computer/main/sim_flag_run.h) and both policies that read the flag are host-tested in
tests_cpp/test_sim_flag_run.cpp, test_arm_consent_policy.cpp and test_landed_lockout_policy.cpp, the issue's
CRC-escaping frame among them. The OC's main.cpp is not host-built, so these tests read the source:
- each policy runs the frame's bit through SimFlagRun::step first and decides on its result alone;
- processFrame feeds each policy once per NonSensorData frame, the frame's own bit, and nothing else steps or
  resets them (a second call per frame would fill the run twice as fast);
- processFrame's time_us filter drops a re-delivered NonSensorData frame (the I2S DMA replays of #69) before
  either step, so one corrupted frame delivered again cannot fill the run;
- arm consent's verdict reaches serviceArmConsent's decide() only from that step;
- ten frames is 20 ms at the FC's NonSensorData rate, far inside the sim's pad phase.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
OC = os.path.join(ROOT, "tinkerrocket-idf", "projects", "out_computer", "main")
MAIN = os.path.join(OC, "main.cpp")
RUN = os.path.join(OC, "sim_flag_run.h")
CONSENT = os.path.join(OC, "arm_consent_policy.h")
LOCKOUT = os.path.join(OC, "landed_lockout_policy.h")
FC_CONFIG = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "config.h")
SIM_H = os.path.join(ROOT, "tinkerrocket-idf", "components", "TR_Sensor_Collector_Sim", "TR_Sensor_Collector_Sim.h")

FLAG = "nsFlagSet(latest_non_sensor.flags, NSF_SIM_ACTIVE)"


def read(path):
    with open(path) as fh:
        return fh.read()


def code(text):
    """The source without // and /* */ comments, whitespace collapsed to one space, none just inside a parenthesis."""
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    text = "\n".join(line.split("//", 1)[0] for line in text.splitlines())
    text = re.sub(r"\s+", " ", text)
    return re.sub(r"\s+\)", ")", re.sub(r"\(\s+", "(", text))


def body(src, signature):
    """The braced body of the function whose signature starts at `signature`."""
    i = src.index("{", src.index(signature))
    depth = 0
    for k in range(i, len(src)):
        depth += {"{": 1, "}": -1}.get(src[k], 0)
        if depth == 0:
            return src[i:k + 1]
    raise AssertionError("unbalanced")


def k_frames():
    m = re.search(r"inline constexpr uint8_t kFrames = (\d+);", code(read(RUN)))
    assert m
    return int(m.group(1))


def test_the_run_needs_the_flag_on_every_frame_and_restarts_without_it():
    run = body(code(read(RUN)), "inline bool step(Run& r, bool sim_active)")
    assert run == ("{ if (!sim_active) { r.n = 0; return false; } if (r.n < kFrames) r.n++; "
                   "return r.n >= kFrames; }"), run


def test_arm_consent_decides_on_the_run_alone():
    src = code(read(CONSENT))
    assert '#include "sim_flag_run.h"' in src
    assert "SimFlagRun::Run flag;" in body(src, "struct SimFlightLatch")
    step = body(src, "inline bool step(SimFlightLatch& s, bool state_inflight, bool sim_active)")
    assert step.startswith("{ const bool sim_flag = SimFlagRun::step(s.flag, sim_active);"), step
    assert step.count("sim_active") == 1, "the raw bit is read only by the run"
    assert "else if (!s.prev_inflight) s.sim = sim_flag;" in step
    assert "else if (sim_flag) s.sim = true;" in step


def test_the_landed_lockout_decides_on_the_run_alone():
    src = code(read(LOCKOUT))
    assert '#include "sim_flag_run.h"' in src
    assert "SimFlagRun::Run flag;" in body(src, "struct State")
    step = body(src, "inline bool step(State& s, RocketState state, bool sim_flag_bit)")
    assert step.startswith("{ const bool sim_active = SimFlagRun::step(s.flag, sim_flag_bit);"), step
    assert step.count("sim_flag_bit") == 1, "the raw bit is read only by the run"
    assert step.count("sim_active") == 4, step   # the run's result: flight, landed, (a)


def test_process_frame_feeds_each_policy_once_per_frame_with_its_own_bit():
    src = code(read(MAIN))
    consent = ("arm_consent_inflight_is_sim = ArmConsentPolicy::step(arm_consent_sim_latch, "
               "(RocketState)latest_non_sensor.rocket_state == INFLIGHT, " + FLAG + ");")
    lockout = "LandedLockout::step(oc_landed_lockout, latest_rocket_state, " + FLAG + ");"
    assert src.count(consent) == 1
    assert src.count(lockout) == 1
    assert src.count("ArmConsentPolicy::step(") == 1
    assert src.count("LandedLockout::step(") == 1
    # both inside the NonSensorData branch, after this frame is copied in
    ns = src.index("else if (type == NON_SENSOR_MSG) {")
    copy = src.index("memcpy(&latest_non_sensor, payload, sizeof(NonSensorData));", ns)
    end = src.index("else if (type ==", copy)
    assert copy < src.index(consent) < end
    assert copy < src.index(lockout) < end
    # the consent step sees this frame's state before it is published
    assert src.index(consent) < src.index("latest_rocket_state = (RocketState)latest_non_sensor.rocket_state;", ns)


def test_a_re_delivered_frame_is_dropped_before_either_step():
    src = code(read(MAIN))
    pf = src.index("static void processFrame(const uint8_t* frame, size_t frame_len,")
    dedup = src.index("case NON_SENSOR_MSG: prev = &prev_time_ns; prev_cb = &prev_cb_ns; break;", pf)
    drop = src.index("if (time_us != 0 && time_us <= *prev) {", dedup)
    assert "return;" in src[drop:src.index("}", src.index("return;", drop)) + 1]
    assert src.index("return;", drop) < src.index("*prev = time_us;", drop)
    ns = src.index("else if (type == NON_SENSOR_MSG) {", pf)
    assert pf < dedup < drop < ns
    assert drop < src.index("ArmConsentPolicy::step(") and drop < src.index("LandedLockout::step(")


def test_nothing_else_steps_or_resets_the_two_policies_state():
    src = code(read(MAIN))
    assert src.count("arm_consent_sim_latch") == 2, "declared, then stepped"
    assert "static ArmConsentPolicy::SimFlightLatch arm_consent_sim_latch;" in src
    assert not re.search(r"\boc_landed_lockout\s*=[^=]", src)
    assert not re.search(r"\boc_landed_lockout\.(flag|prev|flight_was_sim|landed_was_sim|locked)\s*=[^=]", src)
    # the verdict is written only by the step, and read by the decision
    assert "static volatile bool arm_consent_inflight_is_sim = false;" in src
    writes = re.findall(r"(?<!bool )\barm_consent_inflight_is_sim\s*=[^=]", src)   # not the declaration
    assert len(writes) == 1, writes
    assert "const bool is_sim = arm_consent_inflight_is_sim;" in src
    assert "ArmConsentPolicy::decide(test_open, hold.refuse, is_sim);" in src


def test_ten_frames_are_20_ms_at_the_fcs_rate_far_inside_the_sims_pad():
    k = k_frames()
    assert 3 <= k <= 20, k
    rate = re.search(r"NON_SENSOR_UPDATE_RATE = (\d+);", read(FC_CONFIG))
    assert rate and int(rate.group(1)) == 500
    run_ms = k * 1000 / int(rate.group(1))
    pad = re.search(r"PRELAUNCH_DURATION_MS = ([\d.]+)f;", read(SIM_H))
    assert pad and float(pad.group(1)) >= 1000
    assert run_ms <= 50 and run_ms * 20 < float(pad.group(1)), (run_ms, pad.group(1))
    host = read(os.path.join(ROOT, "tests_cpp", "test_sim_flag_run.cpp"))
    assert "EXPECT_EQ(kFrames * 1000u / kNonSensorHz, 20u);" in host and k == 10
