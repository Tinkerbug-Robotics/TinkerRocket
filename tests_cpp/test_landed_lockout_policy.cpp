// #317 / #1235 item 5 — the OC's post-LANDED flight-log lockout.
//
// Two things are pinned here.  The lockout has to HOLD through an FC-only
// reset after a real landing: the FC's post_flight_lockout is plain RAM, so a
// reset walks it INITIALIZATION -> READY -> PRELAUNCH with the OC still up,
// and the old rule ("any departure from LANDED is a sim re-arm") cleared on
// exactly that.  And it has to CLEAR on the two departures that genuinely are
// the FC's sim reset — a frame carrying NSF_SIM_ACTIVE, and the end of a
// LANDED phase that was itself a sim flight — or a real flight after a bench
// sim goes unlogged until an OC reboot, which the old rule never did.
//
// The frame shapes below follow the FC (flight_computer main.cpp): a flown-out
// sim holds LANDED with the flag up through the sim's landed hold, the sim
// then gives up and the FC keeps holding LANDED with the flag DOWN
// (resetFlightStateForSim is deliberately not called on natural completion),
// and the operator's sim Stop resets it to READY with the flag still down.

#include <gtest/gtest.h>

#include "landed_lockout_policy.h"

using LandedLockout::State;
using LandedLockout::step;

namespace {

struct Frame { RocketState state; bool sim; };

// Feed a frame sequence; returns the lockout after the last one.
bool feed(State& s, std::initializer_list<Frame> frames)
{
    bool locked = s.locked;
    for (const Frame& f : frames) locked = step(s, f.state, f.sim);
    return locked;
}

// A real flight up to and including touchdown.
constexpr std::initializer_list<Frame> kRealFlightToLanded = {
    {INITIALIZATION, false}, {READY, false}, {PRELAUNCH, false},
    {INFLIGHT, false}, {LANDED, false}, {LANDED, false},
};

}  // namespace

// ── latching ────────────────────────────────────────────────────────────────

TEST(LandedLockoutPolicy, NothingLocksBeforeALanding)
{
    State s;
    EXPECT_FALSE(feed(s, {{INITIALIZATION, false}, {READY, false},
                          {PRELAUNCH, false}, {INFLIGHT, false}}));
    // A sim that starts and is stopped on the pad never touches it either.
    EXPECT_FALSE(feed(s, {{READY, true}, {PRELAUNCH, true}, {READY, false}}));
}

TEST(LandedLockoutPolicy, ARealLandingLatches)
{
    State s;
    EXPECT_TRUE(feed(s, kRealFlightToLanded));
    EXPECT_FALSE(s.landed_was_sim);
    EXPECT_TRUE(feed(s, {{LANDED, false}, {LANDED, false}}));
}

TEST(LandedLockoutPolicy, AnOcThatBootsIntoALandedRocketLatchesToo)
{
    // The OC rebooted (or first heard the FC) with the vehicle already down:
    // its first frame is LANDED.  Same latch, same hold.
    State s;
    EXPECT_TRUE(feed(s, {{LANDED, false}}));
    EXPECT_TRUE(feed(s, {{INITIALIZATION, false}, {READY, false}, {PRELAUNCH, false}}));
}

// ── item 5: the hold ────────────────────────────────────────────────────────

TEST(LandedLockoutPolicy, AnFcResetAfterARealLandingDoesNotClearIt)
{
    // THE regression.  The FC reboots with the OC up; its RAM lockout is
    // gone and it walks back to PRELAUNCH.  The old rule cleared on the
    // LANDED -> INITIALIZATION edge and the PRELAUNCH hook opened a session.
    State s;
    feed(s, kRealFlightToLanded);
    EXPECT_TRUE(feed(s, {{INITIALIZATION, false}}));
    EXPECT_TRUE(feed(s, {{READY, false}}));
    EXPECT_TRUE(feed(s, {{PRELAUNCH, false}}));
    // ...and a re-tripped launch detect after that is still refused.
    EXPECT_TRUE(feed(s, {{INFLIGHT, false}}));
}

TEST(LandedLockoutPolicy, AnFcThatResetsStraightToReadyIsHeldToo)
{
    // The OC may miss the boot-time INITIALIZATION frames (no I2S traffic
    // while the FC is in setup); the first frame it sees is READY.
    State s;
    feed(s, kRealFlightToLanded);
    EXPECT_TRUE(feed(s, {{READY, false}, {PRELAUNCH, false}}));
}

TEST(LandedLockoutPolicy, ASimStopThatReachesARealLandingChangesNothing)
{
    // #1113: the FC ignores it and stays LANDED.  Nothing to clear on.
    State s;
    feed(s, kRealFlightToLanded);
    EXPECT_TRUE(feed(s, {{LANDED, false}, {LANDED, false}}));
}

// ── item 5: the two demonstrable sim re-arms ────────────────────────────────

TEST(LandedLockoutPolicy, ASimStartAfterARealLandingClearsIt)
{
    // (a) The FC accepted a sim START post-landing: resetFlightStateForSim
    // ran and the next frames carry NSF_SIM_ACTIVE.  A deliberate operator
    // action, and the sim flight should log.
    State s;
    feed(s, kRealFlightToLanded);
    EXPECT_FALSE(feed(s, {{READY, true}}));
    EXPECT_FALSE(feed(s, {{PRELAUNCH, true}, {INFLIGHT, true}}));
}

TEST(LandedLockoutPolicy, AFlownOutSimThenStopClearsIt)
{
    // (b) A sim flight lands and is later stopped.  The departure frame has
    // the flag DOWN — the sim gave up during the LANDED hold — so only the
    // memory that this LANDED phase was a sim can clear it.  Without that a
    // real flight after a bench sim is unlogged until an OC reboot.
    State s;
    EXPECT_TRUE(feed(s, {{READY, true}, {PRELAUNCH, true}, {INFLIGHT, true},
                         {LANDED, true}, {LANDED, true},      // the sim's landed hold
                         {LANDED, false}, {LANDED, false}})); // the sim gave up; FC holds
    EXPECT_TRUE(s.landed_was_sim);
    EXPECT_FALSE(feed(s, {{READY, false}}));                  // sim Stop: FC resets
    // ...and the real flight that follows is logged.
    EXPECT_FALSE(feed(s, {{PRELAUNCH, false}, {INFLIGHT, false}}));
}

TEST(LandedLockoutPolicy, AFlownOutSimThenANewSimStartClearsIt)
{
    State s;
    feed(s, {{READY, true}, {INFLIGHT, true}, {LANDED, true}, {LANDED, false}});
    EXPECT_FALSE(feed(s, {{READY, true}}));
}

TEST(LandedLockoutPolicy, AnFcResetAfterASimLandingClearsIt)
{
    // The bench, not the field: the vehicle that landed was simulated, so a
    // fresh session on the next PRELAUNCH is an ordinary pad session.  This is
    // what the old rule did too.
    State s;
    feed(s, {{READY, true}, {INFLIGHT, true}, {LANDED, true}, {LANDED, false}});
    EXPECT_FALSE(feed(s, {{INITIALIZATION, false}}));
}

// ── phase bookkeeping ───────────────────────────────────────────────────────

TEST(LandedLockoutPolicy, ARealLandingAfterASimIsHeldLikeAnyRealLanding)
{
    // The sim memory belongs to ITS LANDED phase.  A later real flight's
    // LANDED starts a fresh phase, and an FC reset after it must hold.
    State s;
    feed(s, {{READY, true}, {INFLIGHT, true}, {LANDED, true}, {LANDED, false}});
    EXPECT_FALSE(feed(s, {{READY, false}}));                  // sim Stop
    EXPECT_TRUE(feed(s, {{PRELAUNCH, false}, {INFLIGHT, false}, {LANDED, false}}));
    EXPECT_FALSE(s.landed_was_sim);
    EXPECT_TRUE(feed(s, {{INITIALIZATION, false}, {READY, false}, {PRELAUNCH, false}}));
}

TEST(LandedLockoutPolicy, ItRelatchesOnTheNextLanding)
{
    State s;
    feed(s, kRealFlightToLanded);
    EXPECT_FALSE(feed(s, {{READY, true}}));                   // sim re-arm
    EXPECT_TRUE(feed(s, {{INFLIGHT, true}, {LANDED, true}}));
}

TEST(LandedLockoutPolicy, StepReturnsTheStoredVerdict)
{
    State s;
    EXPECT_EQ(step(s, LANDED, false), s.locked);
    EXPECT_TRUE(s.locked);
    EXPECT_EQ(step(s, READY, true), s.locked);
    EXPECT_FALSE(s.locked);
}
