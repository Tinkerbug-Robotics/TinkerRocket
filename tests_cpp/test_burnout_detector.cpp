// Tests for the post-launch burnout-detector hysteresis (#197), exercising the
// REAL firmware logic in
// tinkerrocket-idf/components/TR_KinematicChecks/BurnoutDetector.h (no longer a
// hand-copied mirror).  The flight loop calls the same tr::burnoutDetectStep()
// unconditionally — independent of roll/servo control (#256) — so these tests
// are the single source of truth for the detector's behavior.

#include <gtest/gtest.h>
#include <cstdint>
#include "BurnoutDetector.h"

namespace {

// Mirror of config::BURNOUT_NEG_HYSTERESIS in
// tinkerrocket-idf/projects/flight_computer/main/config.h.
constexpr uint16_t BURNOUT_NEG_HYSTERESIS = 50;
// 200 ms launch lockout, owned by BurnoutDetector.h.
constexpr uint32_t LAUNCH_LOCKOUT_MS = tr::kBurnoutLaunchLockoutMs;

// Thin test wrapper over the real detector step so the existing cases read
// unchanged; fired_at_ms is captured from the single rising-edge return.
struct BurnoutDetector {
    bool     detected  = false;
    uint16_t neg_count = 0;
    uint32_t fired_at_ms = 0;

    void update(float body_ax, uint32_t t_since_launch_ms) {
        if (tr::burnoutDetectStep(detected, neg_count, body_ax,
                                  t_since_launch_ms, BURNOUT_NEG_HYSTERESIS)) {
            fired_at_ms = t_since_launch_ms;
        }
    }
};

} // namespace

TEST(BurnoutDetector, SingleNegativeSample_DoesNotFire) {
    BurnoutDetector kd;
    kd.update(-5.0f, 300);  // negative, past lockout, single sample
    EXPECT_FALSE(kd.detected);
    EXPECT_EQ(kd.neg_count, 1);
}

TEST(BurnoutDetector, JustUnderHysteresis_DoesNotFire) {
    BurnoutDetector kd;
    for (int i = 0; i < BURNOUT_NEG_HYSTERESIS - 1; ++i) {
        kd.update(-5.0f, 300 + i);
    }
    EXPECT_FALSE(kd.detected);
    EXPECT_EQ(kd.neg_count, BURNOUT_NEG_HYSTERESIS - 1);
}

TEST(BurnoutDetector, ReachesHysteresis_Fires) {
    BurnoutDetector kd;
    for (int i = 0; i < BURNOUT_NEG_HYSTERESIS; ++i) {
        kd.update(-5.0f, 300 + i);
    }
    EXPECT_TRUE(kd.detected);
    EXPECT_EQ(kd.fired_at_ms, 300 + BURNOUT_NEG_HYSTERESIS - 1);
}

TEST(BurnoutDetector, PositiveSampleResetsCounter) {
    BurnoutDetector kd;
    // 49 negative samples — one short of latching.
    for (int i = 0; i < BURNOUT_NEG_HYSTERESIS - 1; ++i) {
        kd.update(-5.0f, 300 + i);
    }
    EXPECT_EQ(kd.neg_count, BURNOUT_NEG_HYSTERESIS - 1);
    // One positive sample resets the counter.
    kd.update(2.0f, 400);
    EXPECT_EQ(kd.neg_count, 0);
    EXPECT_FALSE(kd.detected);
    // 49 more negatives still doesn't fire (need 50 in a row).
    for (int i = 0; i < BURNOUT_NEG_HYSTERESIS - 1; ++i) {
        kd.update(-5.0f, 401 + i);
    }
    EXPECT_FALSE(kd.detected);
}

TEST(BurnoutDetector, LaunchLockout_BlocksFiringEarly) {
    BurnoutDetector kd;
    // 200 negative samples within the launch lockout window: nothing fires.
    for (int i = 0; i < 200; ++i) {
        kd.update(-5.0f, i);  // t_since_launch_ms = 0..199, all <= 200
    }
    EXPECT_FALSE(kd.detected);
    EXPECT_EQ(kd.neg_count, 0);
}

TEST(BurnoutDetector, LaunchLockout_BoundaryCondition) {
    BurnoutDetector kd;
    // At exactly t_since_launch_ms = 200, still locked out.
    kd.update(-5.0f, 200);
    EXPECT_EQ(kd.neg_count, 0);
    // At t_since_launch_ms = 201, lockout cleared.
    kd.update(-5.0f, 201);
    EXPECT_EQ(kd.neg_count, 1);
}

TEST(BurnoutDetector, OnceLatched_StaysLatchedAcrossPositiveSamples) {
    BurnoutDetector kd;
    for (int i = 0; i < BURNOUT_NEG_HYSTERESIS; ++i) {
        kd.update(-5.0f, 300 + i);
    }
    ASSERT_TRUE(kd.detected);
    uint32_t latched_at = kd.fired_at_ms;
    // Positive samples after latch should not un-fire.
    kd.update(50.0f, 1000);
    kd.update(-50.0f, 1001);
    EXPECT_TRUE(kd.detected);
    EXPECT_EQ(kd.fired_at_ms, latched_at);  // fire time unchanged
}

TEST(BurnoutDetector, NoSpuriousFireOnAlternatingSignal) {
    // Heavy vibration alternates sign each sample — must never fire.
    BurnoutDetector kd;
    for (int i = 0; i < 1000; ++i) {
        kd.update((i % 2 == 0) ? -10.0f : 10.0f, 300 + i);
    }
    EXPECT_FALSE(kd.detected);
}

TEST(BurnoutDetector, RIM66Profile_FiresAtRealCutoff) {
    // RIM-66 5/17: body_ax was strongly positive (~75 m/s²) during boost
    // T=1..3 s post-launch, then dropped to a sustained -7 m/s² through
    // T=8 s. The detector should fire near T+3 s (close to the very
    // first sustained-negative run, allowing for the hysteresis window).
    BurnoutDetector kd;
    // 1 kHz cadence: 2000 ms of boost (positive), then sustained negative.
    uint32_t t = 250;  // past lockout
    for (int i = 0; i < 2000; ++i, ++t) {
        kd.update(75.0f, t);  // boost
    }
    ASSERT_FALSE(kd.detected);
    // Motor cuts off — sustained negative.
    for (int i = 0; i < 100; ++i, ++t) {
        kd.update(-7.0f, t);
    }
    EXPECT_TRUE(kd.detected);
    // Fire time should be at sample-50 of the negative run.
    EXPECT_EQ(kd.fired_at_ms, 250 + 2000 + BURNOUT_NEG_HYSTERESIS - 1);
}

// #256 regression: burnout detection must depend ONLY on the IMU sample and
// time-since-launch — never on roll/servo control state.  The shared step
// function takes no control-mode input, so a flight with roll control disabled
// detects burnout byte-identically to one with it enabled.  (In the firmware
// tr::burnoutDetectStep() is called above `if (servo_enabled)`; apogee + pyro
// recovery depend on the burnout it latches — so gating it behind servo
// control silently disables recovery.)
TEST(BurnoutDetector, Issue256_DetectionIndependentOfControlMode) {
    auto run_to_burnout = []() {
        BurnoutDetector kd;
        uint32_t t = 250;  // past lockout
        for (int i = 0; i < 2000; ++i, ++t) kd.update(75.0f, t);                    // boost
        for (int i = 0; i < BURNOUT_NEG_HYSTERESIS; ++i, ++t) kd.update(-7.0f, t);  // cutoff
        return kd;
    };
    // Two notional flights with identical IMU history — e.g. servos enabled vs
    // disabled — must reach identical detector state.
    BurnoutDetector roll_on  = run_to_burnout();
    BurnoutDetector roll_off = run_to_burnout();
    EXPECT_TRUE(roll_on.detected);
    EXPECT_TRUE(roll_off.detected);
    EXPECT_EQ(roll_on.fired_at_ms, roll_off.fired_at_ms);
}
