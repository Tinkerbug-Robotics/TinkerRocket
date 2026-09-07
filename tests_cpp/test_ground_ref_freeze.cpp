#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include "GroundRefFreeze.h"

// #1108: may the barometric ground reference follow the current sample?  The
// FC re-seeds the datum every pass before PRELAUNCH; this helper says "not
// while the pressure is moving at launch rates", and hands back the
// pre-motion pressure on the edge so the datum is not left a few metres high.

using GroundRefFreeze::State;
using GroundRefFreeze::Verdict;

namespace {
constexpr float    P0     = 101325.0f;
constexpr float    PA_PER_M = 12.0f;        // near sea level
constexpr uint32_t BARO_DT_US = 4167;       // ~240 Hz BMP585

// Run a pressure-vs-time function through the helper at the baro rate and
// return the first FreezeEdge time (or -1) and the last verdict.
struct DriveResult { int64_t first_freeze_us = -1; Verdict last = Verdict::Track; float rollback = 0.0f; int freezes = 0; };

template <typename F>
DriveResult drive(State& st, F p_of_t_us, uint32_t span_us, uint32_t t0_us = 0)
{
    DriveResult r;
    for (uint32_t t = t0_us; t < t0_us + span_us; t += BARO_DT_US) {
        const Verdict v = GroundRefFreeze::step(st, t, p_of_t_us(t));
        if (v == Verdict::FreezeEdge) {
            if (r.first_freeze_us < 0) { r.first_freeze_us = t; r.rollback = st.rollback_pa; }
            r.freezes++;
        }
        r.last = v;
    }
    return r;
}
}  // namespace

// Weather: a pascal a minute, for ten minutes.  Never held.
TEST(GroundRefFreeze, WeatherDriftTracks) {
    State st;
    const DriveResult r = drive(st, [](uint32_t t) { return P0 - (t * 1e-6f) * (1.0f / 60.0f); }, 600u * 1000000u);
    EXPECT_EQ(r.first_freeze_us, -1);
    EXPECT_EQ(r.last, Verdict::Track);
}

// Handling: carrying the airframe up a ladder at 1 m/s (~12 Pa/s) is under
// the bar, so the datum keeps following the vehicle to its new height.
TEST(GroundRefFreeze, CarryAtOneMetrePerSecondTracks) {
    State st;
    const DriveResult r = drive(st, [](uint32_t t) { return P0 - (t * 1e-6f) * 1.0f * PA_PER_M; }, 5u * 1000000u);
    EXPECT_EQ(r.first_freeze_us, -1);
}

// Sensor noise: 3 Pa one-sigma on a still pad (BMP585 is ~2) for ten minutes
// must never hold.  With the ends of the window averaged the rate is ~10 Pa/s
// one-sigma here, six sigma under the bar; a single-sample rate at a 40 Pa/s
// bar false-held every few seconds when this was first written.
TEST(GroundRefFreeze, StillPadWithNoiseNeverHolds) {
    State st;
    std::mt19937 rng(7);
    std::normal_distribution<float> noise(0.0f, 3.0f);
    const DriveResult r = drive(st, [&](uint32_t) { return P0 + noise(rng); }, 600u * 1000000u);
    EXPECT_EQ(r.freezes, 0) << "first at t=" << r.first_freeze_us;
}

// A launch: 3 g net from a standing start.  The datum must be held within the
// first few hundred milliseconds, and the rollback must be the pad pressure,
// not the pressure at the moment the rate crossed the bar.
TEST(GroundRefFreeze, LaunchHoldsEarlyAndRollsBackToThePad) {
    State st;
    // one second of still pad first, so the history is full of pad pressure
    drive(st, [](uint32_t) { return P0; }, 1000000u);
    auto climb = [](uint32_t t) {
        const float s = (t - 1000000u) * 1e-6f;
        return P0 - 0.5f * 30.0f * s * s * PA_PER_M;   // h = 1/2 a t^2
    };
    const DriveResult r = drive(st, climb, 2000000u, 1000000u);
    ASSERT_GE(r.first_freeze_us, 0);
    const float t_hold_s = (r.first_freeze_us - 1000000) * 1e-6f;
    EXPECT_LT(t_hold_s, 0.5f) << "held at +" << t_hold_s << " s";
    EXPECT_NEAR(r.rollback, P0, 1.5f * PA_PER_M) << "rollback must be the pad, within ~1.5 m";
    EXPECT_EQ(r.last, Verdict::Frozen) << "still climbing, still held";
}

// A hoist onto the pad at 3 m/s is UNDER the bar: the datum simply follows
// the airframe to its new height, as it always has.
TEST(GroundRefFreeze, HoistAtThreeMetresPerSecondTracks) {
    State st;
    drive(st, [](uint32_t) { return P0; }, 1000000u);
    auto lift = [](uint32_t t) {
        const float s = (t - 1000000u) * 1e-6f;                 // 0..0.5 s: rise 1.5 m at 3 m/s
        const float h = s < 0.5f ? 3.0f * s : 1.5f;
        return P0 - h * PA_PER_M;
    };
    const DriveResult r = drive(st, lift, 3000000u, 1000000u);
    EXPECT_EQ(r.first_freeze_us, -1);
}

// A quick jerk upward (1.5 m at 8 m/s, over the bar) is held while it lasts,
// and tracking resumes a second after it stops, re-seeding at the new height.
TEST(GroundRefFreeze, JerkHoldsThenResumesTrackingAfterASteadySecond) {
    State st;
    drive(st, [](uint32_t) { return P0; }, 1000000u);
    auto jerk = [](uint32_t t) {
        const float s = (t - 1000000u) * 1e-6f;
        const float h = s < 0.1875f ? 8.0f * s : 1.5f;
        return P0 - h * PA_PER_M;
    };
    Verdict at_half_s = Verdict::Track, at_end = Verdict::Track;
    bool held = false;
    for (uint32_t t = 1000000u; t < 4000000u; t += BARO_DT_US) {
        const Verdict v = GroundRefFreeze::step(st, t, jerk(t));
        if (v == Verdict::FreezeEdge) held = true;
        if (t < 1500000u + BARO_DT_US && t >= 1500000u) at_half_s = v;
        at_end = v;
    }
    ASSERT_TRUE(held) << "8 m/s is over the bar";
    EXPECT_EQ(at_half_s, Verdict::Frozen) << "still inside the steady second";
    EXPECT_EQ(at_end, Verdict::Track) << "steady for over a second: tracking again";
}

// In flight the window rate can dip under the bar for a moment (the boost
// suction transient snapping back at burnout cancels the climb for a window).
// One Track verdict there would re-seed the datum at altitude, so a hold is
// sticky: tracking resumes only after a full second under the bar.
TEST(GroundRefFreeze, MomentaryDipInFlightDoesNotReseed) {
    State st;
    drive(st, [](uint32_t) { return P0; }, 1000000u);
    auto climb_with_dip = [](uint32_t t) {
        const float s = (t - 1000000u) * 1e-6f;
        float h = 20.0f * s;                                   // 20 m/s climb
        if (s > 1.0f && s < 1.4f) h = 20.0f;                   // 400 ms flat: window rate ~0
        else if (s >= 1.4f) h = 20.0f + 20.0f * (s - 1.4f);
        return P0 - h * PA_PER_M;
    };
    int tracks_after_hold = 0;
    bool held = false;
    for (uint32_t t = 1000000u; t < 4000000u; t += BARO_DT_US) {
        const Verdict v = GroundRefFreeze::step(st, t, climb_with_dip(t));
        if (v == Verdict::FreezeEdge) held = true;
        if (held && v == Verdict::Track) tracks_after_hold++;
    }
    ASSERT_TRUE(held);
    EXPECT_EQ(tracks_after_hold, 0) << "the datum re-seeded at altitude";
}

// Reset empties the history: the first quarter second after a reset cannot
// produce a rate and must be treated as trackable.
TEST(GroundRefFreeze, ResetClearsHistory) {
    State st;
    drive(st, [](uint32_t t) { return P0 - (t * 1e-6f) * 20.0f * PA_PER_M; }, 1000000u);
    ASSERT_TRUE(st.frozen);
    GroundRefFreeze::reset(st);
    EXPECT_FALSE(st.frozen);
    EXPECT_EQ(st.count, 0);
    EXPECT_EQ(GroundRefFreeze::step(st, 5000000u, P0), Verdict::Track);
}
