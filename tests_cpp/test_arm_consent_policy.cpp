// When the OC gives its half of the Beetle's two-processor pyro arm
// (OC_ARM_EN -> Q14). Until this existed nothing raised the pin, so the board
// could not fire a channel in a ground test or in flight, while continuity read
// true and a bench LED lit through R73's 2.2 k (2026-09-24).
//
// The load-bearing rows:
//   - a fire test's window covers the FC's real timeline: the command is read
//     on the NEXT 250 ms poll, possibly only on the third delivery, and the
//     pulse runs 210 ms after that;
//   - a real flight holds consent THROUGH a silent FC (owner decision): an FC
//     reboot or a link dropout near apogee must not cost the deployment;
//   - a simulated flight never raises it, including the sim's give-up tick,
//     where NSF_SIM_ACTIVE drops while the FC still says INFLIGHT.

#include <gtest/gtest.h>

#include "arm_consent_policy.h"
#include "inflight_refusal_policy.h"

using ArmConsentPolicy::decide;
using ArmConsentPolicy::FireTestWindow;
using ArmConsentPolicy::fireTestWindowOpen;
using ArmConsentPolicy::kFireTestHoldMs;
using ArmConsentPolicy::onFireTestStaged;
using ArmConsentPolicy::pinHigh;
using ArmConsentPolicy::Reason;
using ArmConsentPolicy::SimFlightLatch;
using ArmConsentPolicy::step;

namespace {
constexpr uint32_t kSec = 1000U;

// The FC's side of a fire test, from flight_computer main.cpp and config.h.
constexpr uint32_t kFcPollMs       = 250;   // (now_ms - out_ready_request_time_ms) > 250U
constexpr uint32_t kFcReadTimeout  = 50;    // masterRead(..., 50)
constexpr uint32_t kFcArmSettleMs  = 10;    // PYRO_ARM_SETTLE_MS
constexpr uint32_t kFcFireMs       = 200;   // PYRO_FIRE_DURATION_MS
constexpr uint32_t kCmdRepeatLimit = 3;     // OC CMD_REPEAT_LIMIT

// Consent as serviceArmConsent() computes it, with the #1162 hold composed in.
Reason consent(bool test_open, bool state_inflight, bool fc_fresh,
               uint32_t inflight_age_ms, bool inflight_is_sim)
{
    return decide(test_open,
                  InflightRefusalPolicy::refuse(state_inflight, fc_fresh, inflight_age_ms),
                  inflight_is_sim);
}
}  // namespace

// ---------------------------------------------------------------------------
// The fire-test window
// ---------------------------------------------------------------------------

TEST(ArmConsentFireTest, ClosedUntilAFireTestIsStaged) {
    FireTestWindow w;
    EXPECT_FALSE(fireTestWindowOpen(w, 0));
    EXPECT_FALSE(fireTestWindowOpen(w, 123456));
}

TEST(ArmConsentFireTest, OpenForTheHoldThenClosed) {
    FireTestWindow w;
    onFireTestStaged(w, 10 * kSec);
    EXPECT_TRUE(fireTestWindowOpen(w, 10 * kSec));
    EXPECT_TRUE(fireTestWindowOpen(w, 10 * kSec + kFireTestHoldMs - 1));
    EXPECT_FALSE(fireTestWindowOpen(w, 10 * kSec + kFireTestHoldMs));
    EXPECT_FALSE(fireTestWindowOpen(w, 10 * kSec + kFireTestHoldMs + 1));
}

TEST(ArmConsentFireTest, EveryDeliveryPushesTheEndOut) {
    // The OC stages the command once per FC poll, CMD_REPEAT_LIMIT times.
    FireTestWindow w;
    for (uint32_t i = 0; i < kCmdRepeatLimit; ++i) onFireTestStaged(w, i * kFcPollMs);
    const uint32_t last = (kCmdRepeatLimit - 1) * kFcPollMs;
    EXPECT_TRUE(fireTestWindowOpen(w, last + kFireTestHoldMs - 1));
    EXPECT_FALSE(fireTestWindowOpen(w, last + kFireTestHoldMs));
}

TEST(ArmConsentFireTest, CoversTheFcTimelineEvenOnTheLastDelivery) {
    // The FC reads a staged frame on its NEXT poll, then runs ARM -> settle ->
    // FIRE synchronously. Worst nominal case: the first two reads fail, so it
    // acts on the last delivery, a full poll plus a read timeout after it was
    // staged. Consent has to be up for the whole pulse.
    FireTestWindow w;
    uint32_t last_staged = 0;
    for (uint32_t i = 0; i < kCmdRepeatLimit; ++i) {
        last_staged = i * kFcPollMs;
        onFireTestStaged(w, last_staged);
    }
    const uint32_t fc_reads   = last_staged + kFcPollMs + kFcReadTimeout;
    const uint32_t pulse_ends = fc_reads + kFcArmSettleMs + kFcFireMs;
    for (uint32_t t = fc_reads; t <= pulse_ends; ++t) {
        EXPECT_TRUE(fireTestWindowOpen(w, t)) << t;
    }
    // ...with margin for FC loop stalls: at least twice the nominal gap.
    EXPECT_GE(kFireTestHoldMs, 2U * (kFcPollMs + kFcReadTimeout + kFcArmSettleMs + kFcFireMs));
}

TEST(ArmConsentFireTest, SurvivesTheMillisRollover) {
    FireTestWindow w;
    const uint32_t staged = 0xFFFFFFFFu - 500U;
    onFireTestStaged(w, staged);
    EXPECT_TRUE(fireTestWindowOpen(w, 0xFFFFFFFFu));
    EXPECT_TRUE(fireTestWindowOpen(w, 1000U));   // wrapped, 1501 ms in
    EXPECT_FALSE(fireTestWindowOpen(w, staged + kFireTestHoldMs));
}

TEST(ArmConsentFireTest, AnExpiredWindowNeverReopens) {
    // A deadline compared by signed difference reads "open" again 2^31 ms after
    // it expired. Observing the expiry clears the flag, so it cannot.
    FireTestWindow w;
    onFireTestStaged(w, 0);
    EXPECT_FALSE(fireTestWindowOpen(w, kFireTestHoldMs));
    EXPECT_FALSE(fireTestWindowOpen(w, kFireTestHoldMs + 0x80000000u));
    EXPECT_FALSE(fireTestWindowOpen(w, kFireTestHoldMs + 0x80000001u));
    EXPECT_FALSE(fireTestWindowOpen(w, 0xFFFFFFFFu));
}

// ---------------------------------------------------------------------------
// The simulated-flight latch (fed one NonSensorData frame at a time)
// ---------------------------------------------------------------------------

TEST(ArmConsentSimLatch, ARealFlightIsNotASim) {
    SimFlightLatch s;
    EXPECT_FALSE(step(s, false, false));   // PRELAUNCH
    EXPECT_FALSE(step(s, true, false));    // launch
    EXPECT_FALSE(step(s, true, false));
    EXPECT_FALSE(step(s, false, false));   // LANDED
}

TEST(ArmConsentSimLatch, ASimIsASimFromItsFirstInflightFrame) {
    SimFlightLatch s;
    EXPECT_FALSE(step(s, false, true));    // sim running, still on the pad
    EXPECT_TRUE(step(s, true, true));      // sim launch
    EXPECT_TRUE(step(s, true, true));
}

TEST(ArmConsentSimLatch, TheGiveUpTickStaysASim) {
    // #1104: the sim gives up with the FC still INFLIGHT, and NSF_SIM_ACTIVE
    // drops before the FC resets the flight. The FC is still dry-firing then,
    // so this must not read as a real flight for even one frame.
    SimFlightLatch s;
    step(s, false, true);
    EXPECT_TRUE(step(s, true, true));
    EXPECT_TRUE(step(s, true, false));
    EXPECT_TRUE(step(s, true, false));
    EXPECT_FALSE(step(s, false, false));   // FC resets to READY
}

TEST(ArmConsentSimLatch, ARealFlightAfterASimIsReal) {
    // No OC reboot between a bench sim and a real flight.
    SimFlightLatch s;
    step(s, false, true);
    EXPECT_TRUE(step(s, true, true));
    EXPECT_FALSE(step(s, false, true));    // sim LANDED
    EXPECT_FALSE(step(s, false, false));   // Stop -> READY
    EXPECT_FALSE(step(s, false, false));   // PRELAUNCH
    EXPECT_FALSE(step(s, true, false));    // real launch
}

TEST(ArmConsentSimLatch, AnOcThatJoinsMidFlightDecidesOnItsFirstFrame) {
    // An OC reboot during a flight: the first frame it processes is already
    // INFLIGHT. That frame's flag decides.
    SimFlightLatch real;
    EXPECT_FALSE(step(real, true, false));
    SimFlightLatch sim;
    EXPECT_TRUE(step(sim, true, true));
}

TEST(ArmConsentSimLatch, ASimFlagMidFlightLatchesOn) {
    // Mirrors the FC's simulated() = latched || active: if the FC says a sim is
    // active it is dry-firing, so consent would buy nothing.
    SimFlightLatch s;
    EXPECT_FALSE(step(s, true, false));
    EXPECT_TRUE(step(s, true, true));
    EXPECT_TRUE(step(s, true, false));
}

// ---------------------------------------------------------------------------
// The decision
// ---------------------------------------------------------------------------

TEST(ArmConsentDecide, NothingToConsentTo) {
    EXPECT_EQ(decide(false, false, false), Reason::None);
    EXPECT_EQ(decide(false, false, true), Reason::None);
    EXPECT_FALSE(pinHigh(Reason::None));
}

TEST(ArmConsentDecide, FireTestAlone) {
    EXPECT_EQ(decide(true, false, false), Reason::FireTest);
    EXPECT_TRUE(pinHigh(Reason::FireTest));
}

TEST(ArmConsentDecide, RealFlight) {
    EXPECT_EQ(decide(false, true, false), Reason::Flight);
    EXPECT_EQ(decide(true, true, false), Reason::Flight);
    EXPECT_TRUE(pinHigh(Reason::Flight));
}

TEST(ArmConsentDecide, SimulatedFlightNeverConsents) {
    EXPECT_EQ(decide(false, true, true), Reason::None);
}

// ---------------------------------------------------------------------------
// With the #1162 in-flight hold composed in, as serviceArmConsent() runs it
// ---------------------------------------------------------------------------

TEST(ArmConsentScenario, PadStaysLow) {
    // PRELAUNCH can last hours with people around the rocket.
    EXPECT_EQ(consent(false, false, true, 0, false), Reason::None);
    EXPECT_EQ(consent(false, false, false, 0, false), Reason::None);
}

TEST(ArmConsentScenario, LiveFcInFlight) {
    EXPECT_EQ(consent(false, true, true, 0, false), Reason::Flight);
    EXPECT_EQ(consent(false, true, true, 20 * kSec, false), Reason::Flight);
    // A live FC is never overridden by the flight-time bound.
    EXPECT_EQ(consent(false, true, true, 3U * InflightRefusalPolicy::kMaxFlightTimeMs, false),
              Reason::Flight);
}

TEST(ArmConsentScenario, HeldThroughASilentFc) {
    // The owner's call: an FC reboot at T+40 s (~10 s of setup_fc with no
    // frames), or an I2S dropout, keeps consent up so the recovered FC can
    // still deploy.
    EXPECT_EQ(consent(false, true, false, 48 * kSec, false), Reason::Flight);
    EXPECT_EQ(consent(false, true, false, InflightRefusalPolicy::kMaxFlightTimeMs - 1, false),
              Reason::Flight);
}

TEST(ArmConsentScenario, ReleasedOnceNoFlightCouldRemain) {
    EXPECT_EQ(consent(false, true, false, InflightRefusalPolicy::kMaxFlightTimeMs, false),
              Reason::None);
}

TEST(ArmConsentScenario, LandedDropsIt) {
    // latest_rocket_state moves off INFLIGHT, so the hold ends at once.
    EXPECT_EQ(consent(false, false, true, 90 * kSec, false), Reason::None);
}

TEST(ArmConsentScenario, SimulatedFlightStaysLow) {
    EXPECT_EQ(consent(false, true, true, 5 * kSec, true), Reason::None);
    EXPECT_EQ(consent(false, true, false, 5 * kSec, true), Reason::None);
}

TEST(ArmConsentScenario, FireTestOnThePad) {
    EXPECT_EQ(consent(true, false, true, 0, false), Reason::FireTest);
}
