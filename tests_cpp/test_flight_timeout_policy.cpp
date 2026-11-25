// #1753 — the INFLIGHT safety timeout and a main due after T+600 s.
//
// #1706 let the 600 s timeout run to the 1800 s ceiling while a deployment
// channel was owed after apogee, but "owed" counted only Idle channels. On the
// pass a late main triggered, servicePyroChannels() had already moved it to
// ArmSettle, so the timeout at the end of that pass saw nothing owed, safed the
// main and declared LANDED. The main never fired.
//
// The first block tests the policy's table. The second walks the flight loop's
// pass order (servicePyroChannels, then the timeout, on the same now_ms) with
// main.cpp's channel transitions and constants, which is where the defect was.

#include <gtest/gtest.h>

#include <cstdint>

#include "flight_timeout_policy.h"

using FlightTimeoutPolicy::Channel;
using FlightTimeoutPolicy::expired;
using FlightTimeoutPolicy::In;
using FlightTimeoutPolicy::kAfterFireMs;
using FlightTimeoutPolicy::kBaseMs;
using FlightTimeoutPolicy::kCeilingMs;
using FlightTimeoutPolicy::limitMs;
using FlightTimeoutPolicy::owed;

namespace {
constexpr uint32_t kSec = 1000U;
}

// ---- the policy's table ------------------------------------------------------

TEST(FlightTimeoutOwed, AChannelIsOwedUntilItsPulseIsDone) {
    EXPECT_TRUE(owed(/*enabled=*/true, Channel::Idle));
    EXPECT_TRUE(owed(true, Channel::Pulsing));   // #1753: ArmSettle and Firing are still owed
    EXPECT_FALSE(owed(true, Channel::Done));
}

TEST(FlightTimeoutOwed, ADisabledChannelIsOwedOnlyMidPulse) {
    EXPECT_FALSE(owed(/*enabled=*/false, Channel::Idle));
    EXPECT_TRUE(owed(false, Channel::Pulsing));  // never cut a pulse that has begun
    EXPECT_FALSE(owed(false, Channel::Done));
}

TEST(FlightTimeoutLimit, TheBoundsAreTheOnesTheOcAndTheRestoreGateMatch) {
    EXPECT_EQ(kBaseMs, 600U * kSec);
    EXPECT_EQ(kCeilingMs, 1800U * kSec);   // = RecoveryArmGate max_flight_ms, OC kMaxFlightTimeMs
    EXPECT_EQ(kAfterFireMs, 300U * kSec);
}

TEST(FlightTimeoutLimit, BeforeApogeeTheBaseHolds) {
    // Nothing can fire before apogee, so nothing extends the bound.
    EXPECT_EQ(limitMs({false, true, false, 0}), kBaseMs);
    EXPECT_EQ(limitMs({false, false, false, 0}), kBaseMs);
}

TEST(FlightTimeoutLimit, AnOwedChannelAfterApogeeTakesTheCeiling) {
    EXPECT_EQ(limitMs({true, /*owed=*/true, false, 0}), kCeilingMs);
    // Even with an earlier channel already fired.
    EXPECT_EQ(limitMs({true, true, true, 100U * kSec}), kCeilingMs);
}

TEST(FlightTimeoutLimit, NothingOwedAndNothingFiredIsTheBase) {
    EXPECT_EQ(limitMs({true, false, false, 0}), kBaseMs);
}

TEST(FlightTimeoutLimit, AnEarlyLastFireLeavesTheBaseUnchanged) {
    // The normal flight: drogue at apogee, main well before T+300 s. The bound is
    // the 600 s it always was.
    EXPECT_EQ(limitMs({true, false, true, 20U * kSec}), kBaseMs);
    EXPECT_EQ(limitMs({true, false, true, 300U * kSec}), kBaseMs);
}

TEST(FlightTimeoutLimit, ALateLastFireHoldsTheFlightOpenAfterIt) {
    EXPECT_EQ(limitMs({true, false, true, 590U * kSec}), 890U * kSec);
    EXPECT_EQ(limitMs({true, false, true, 650U * kSec}), 950U * kSec);
}

TEST(FlightTimeoutLimit, TheHoldNeverPassesTheCeiling) {
    EXPECT_EQ(limitMs({true, false, true, 1600U * kSec}), kCeilingMs);
    EXPECT_EQ(limitMs({true, false, true, 0xFFFFFFFFU}), kCeilingMs);   // no wrap
}

TEST(FlightTimeoutExpired, StrictlyPastTheLimit) {
    const In in{true, false, false, 0};
    EXPECT_FALSE(expired(kBaseMs, in));
    EXPECT_TRUE(expired(kBaseMs + 1, in));
}

// ---- the flight loop's pass order ---------------------------------------------
//
// main.cpp, case INFLIGHT: servicePyroChannels(now_ms) runs first, then (with
// nothing else between them on a normal pass) the timeout on the same now_ms.
// A channel goes Idle -> ArmSettle on its trigger, ArmSettle -> Firing once
// PYRO_ARM_SETTLE_MS has passed, Firing -> Done after PYRO_FIRE_DURATION_MS. A
// pulse ending sets the last-fire time. The timeout, once expired, safes every
// channel that is not Done back to Idle and the flight is LANDED for good.

namespace {

constexpr uint32_t kArmSettleMs = 10;    // config.h PYRO_ARM_SETTLE_MS
constexpr uint32_t kFireMs      = 200;   // config.h PYRO_FIRE_DURATION_MS

enum class St { Idle, ArmSettle, Firing, Done };

struct Ch {
    bool     enabled  = false;
    uint32_t due_ms   = 0;     // when its trigger is met, ms after launch (0: never)
    St       st       = St::Idle;
    uint32_t phase_ms = 0;
    bool     fired    = false; // FIRE went high at some point
};

struct Flight {
    Ch       ch[4];
    uint32_t apogee_ms        = 0;   // ms after launch
    bool     fired_any        = false;
    uint32_t last_fire_ms     = 0;   // ms after launch
    bool     landed           = false;
    uint32_t landed_at_ms     = 0;
    uint32_t landing_detect_ms = 0;  // when landing detection would fire (0: never)

    static Channel toPolicy(St s) {
        return s == St::Idle ? Channel::Idle : s == St::Done ? Channel::Done : Channel::Pulsing;
    }

    void pass(uint32_t t) {
        if (landed) return;
        if (landing_detect_ms && t >= landing_detect_ms) {
            landed = true; landed_at_ms = t; return;
        }
        // servicePyroChannels
        for (Ch& c : ch) {
            if (c.st == St::Idle && c.enabled && t >= apogee_ms && c.due_ms && t >= c.due_ms) {
                c.st = St::ArmSettle; c.phase_ms = t;
            }
            if (c.st == St::ArmSettle && t - c.phase_ms >= kArmSettleMs) {
                c.st = St::Firing; c.phase_ms = t; c.fired = true;
            }
            if (c.st == St::Firing && t - c.phase_ms >= kFireMs) {
                c.st = St::Done; c.phase_ms = 0;
                fired_any = true; last_fire_ms = t;
            }
        }
        // the timeout, same pass
        bool any_owed = false;
        for (const Ch& c : ch) any_owed = any_owed || owed(c.enabled, toPolicy(c.st));
        const In in{t >= apogee_ms, any_owed, fired_any, last_fire_ms};
        if (expired(t, in)) {
            for (Ch& c : ch) if (c.st != St::Done) { c.st = St::Idle; c.phase_ms = 0; }
            landed = true; landed_at_ms = t;
        }
    }

    void fly(uint32_t until_ms, uint32_t step_ms = 1) {
        for (uint32_t t = 0; t <= until_ms && !landed; t += step_ms) pass(t);
    }
};

Flight dualDeploy(uint32_t apogee_s, uint32_t main_due_ms) {
    Flight f;
    f.apogee_ms = apogee_s * kSec;
    f.ch[0].enabled = true; f.ch[0].due_ms = f.apogee_ms;   // drogue at apogee
    f.ch[1].enabled = true; f.ch[1].due_ms = main_due_ms;   // main on descent
    return f;
}

} // namespace

TEST(FlightTimeoutPassOrder, AMainDueAfterT600FiresAndTheFlightStaysOpenAfterIt) {
    // The issue's case: 6 km apogee under a 10 m/s drogue, main due ~T+615 s.
    // Landing detection never fires here, so only the timeout can end the flight.
    for (uint32_t step : {1U, 2U, 5U}) {
        Flight f = dualDeploy(60, 615U * kSec);
        f.fly(1900U * kSec, step);
        EXPECT_TRUE(f.ch[1].fired) << "step " << step;
        EXPECT_EQ(f.ch[1].st, St::Done) << "step " << step;
        // Held kAfterFireMs past the main's pulse, not LANDED in the air.
        EXPECT_GT(f.landed_at_ms, f.last_fire_ms + kAfterFireMs - 1) << "step " << step;
        EXPECT_LE(f.landed_at_ms, f.last_fire_ms + kAfterFireMs + step) << "step " << step;
    }
}

TEST(FlightTimeoutPassOrder, TheIssueTableMainsAllFire) {
    // #1753's harness table: due at T+601, T+650 and T+1200 s never fired.
    for (uint32_t due_s : {601U, 650U, 1200U, 1795U}) {
        Flight f = dualDeploy(60, due_s * kSec);
        f.fly(1900U * kSec);
        EXPECT_TRUE(f.ch[1].fired) << "main due at T+" << due_s << " s";
    }
}

TEST(FlightTimeoutPassOrder, AMainDueBeforeT600StillFires) {
    Flight f = dualDeploy(60, 590U * kSec);
    f.fly(1900U * kSec);
    EXPECT_TRUE(f.ch[1].fired);
    // And the flight is not cut at T+600 s right after it.
    EXPECT_GE(f.landed_at_ms, 590U * kSec + kFireMs + kAfterFireMs);
}

TEST(FlightTimeoutPassOrder, ABackupDueLaterThanTheMainFiresToo) {
    // The issue: a third channel kept the limit at the ceiling for the primary
    // main, then was itself cut on its own trigger pass. Due more than
    // kAfterFireMs after the main's pulse, so only the owed rule, not the
    // after-fire hold, keeps it.
    Flight f = dualDeploy(60, 650U * kSec);
    f.ch[2].enabled = true; f.ch[2].due_ms = 1000U * kSec;
    f.fly(1900U * kSec);
    EXPECT_TRUE(f.ch[1].fired);
    EXPECT_TRUE(f.ch[2].fired);
}

TEST(FlightTimeoutPassOrder, ANormalFlightIsUnchanged) {
    // Apogee T+15 s, main T+60 s, landing detected at T+90 s: landing ends it.
    Flight f = dualDeploy(15, 60U * kSec);
    f.landing_detect_ms = 90U * kSec;
    f.fly(1900U * kSec);
    EXPECT_TRUE(f.ch[0].fired);
    EXPECT_TRUE(f.ch[1].fired);
    EXPECT_EQ(f.landed_at_ms, 90U * kSec);
}

TEST(FlightTimeoutPassOrder, ALandingDetectionFailureStillEndsAtT600) {
    // Everything fired early and landing detection never comes: the 600 s base,
    // as before #1706.
    Flight f = dualDeploy(15, 60U * kSec);
    f.fly(1900U * kSec);
    EXPECT_EQ(f.landed_at_ms, kBaseMs + 1);
}

TEST(FlightTimeoutPassOrder, AMainThatNeverTriggersEndsAtTheCeiling) {
    // An enabled main whose trigger never comes (a failed altitude source) keeps
    // the flight open to the ceiling, as #1706 intended, and no longer.
    Flight f = dualDeploy(60, 0);
    f.fly(1900U * kSec);
    EXPECT_FALSE(f.ch[1].fired);
    EXPECT_EQ(f.landed_at_ms, kCeilingMs + 1);
}
