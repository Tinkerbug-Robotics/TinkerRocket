// #1104: the sim can end while the FC is still INFLIGHT.
//
// The FC acted only on the RISING edge of isSimActive(), on the assumption
// that the falling edge always coincides with the FC in LANDED — the sim's
// natural completion, where the FC must hold LANDED so the post-flight
// lockout stays validated (#317).  #971 broke that assumption: the SIM_LANDED
// hold gives up after HOLD_MAX_MS when landing detection never fires and
// drops the sim to SIM_IDLE with the FC still INFLIGHT.  From that tick the
// pyro dry-fire gate (keyed off isSimActive()) was OFF and the I2C command
// poll (skipped in a non-sim INFLIGHT) stopped: real ARM/FIRE outputs on the
// bench — a TIME_AFTER_APOGEE delay still counting from the simulated apogee
// needs no motion at all — and no way to reach the FC until the 10-minute
// flight backstop.
//
// Two pure rules close it, and these tests are what pins them:
//   * the edge rule keys on the FC's OWN state — a falling edge with the FC
//     in LANDED is the flown-out sim (hold); any other falling edge is a Stop;
//   * the dry-fire predicate is LATCHED for the flight, because the sim steps
//     its physics inside the FC's IMU read, so servicePyroChannels() runs
//     once with isSimActive() already false before the edge is handled.
//
// #1113 adds a third, pinned here too: a SIM_STOP_CMD acts only on a sim
// flight.  The Stop handler resets the flight state and clears the #317
// post-flight lockout with no state gate of its own, so a stray Stop — a
// broadcast uplink meant for another vehicle, a cmd 7 queued mid-flight and
// delivered at touchdown, a stale app banner — used to end a REAL flight's
// terminal LANDED with a failed channel's e-match still live.

#include <gtest/gtest.h>
#include "sim_flight_policy.h"
#include "sim_landed_hold.h"

namespace {

constexpr uint8_t INFLIGHT = 3;
constexpr uint8_t LANDED   = 4;

using sim_flight::Edge;
using sim_flight::classify;
using sim_flight::simulated;
using sim_flight::stopApplies;

// ── The edge rule ───────────────────────────────────────────────────────────

TEST(SimFlightPolicy, risingEdgeIsAFreshRun)
{
    EXPECT_EQ(classify(false, true, /*fc_landed=*/false), Edge::Start);
    // A new run started while the FC still sits in LANDED from the previous
    // flown-out sim is a fresh run too — the sim's equivalent of a reboot.
    EXPECT_EQ(classify(false, true, /*fc_landed=*/true), Edge::Start);
}

TEST(SimFlightPolicy, steadyStatesDoNothing)
{
    EXPECT_EQ(classify(false, false, false), Edge::None);
    EXPECT_EQ(classify(false, false, true),  Edge::None);
    EXPECT_EQ(classify(true,  true,  false), Edge::None);
    EXPECT_EQ(classify(true,  true,  true),  Edge::None);
}

TEST(SimFlightPolicy, flownOutSimHoldsLanded)
{
    // The falling edge the original rule was written for (#317): the FC
    // reached LANDED, the sim saw it and went idle.  Hold — the post-flight
    // lockout must stay validated.
    EXPECT_EQ(classify(true, false, /*fc_landed=*/true), Edge::EndedLanded);
}

TEST(SimFlightPolicy, anyOtherFallingEdgeIsAStop)
{
    // The falling edge the original rule did NOT anticipate: the sim is gone
    // and the FC is still flying.  There is no safe way to leave that alone.
    EXPECT_EQ(classify(true, false, /*fc_landed=*/false), Edge::EndedEarly);
}

TEST(SimFlightPolicy, theGiveUpIsAnEarlyEnd)
{
    // The chain from #971 to #1104, stated end to end.  The SIM_LANDED hold
    // gives up with the FC not in LANDED...
    ASSERT_EQ(sim_landed::decide(INFLIGHT, LANDED, sim_landed::HOLD_MAX_MS),
              sim_landed::Exit::GaveUp);
    // ...so isSimActive() falls with the FC still INFLIGHT — and that must be
    // treated as a Stop, never as a flown-out sim.
    EXPECT_EQ(classify(true, false, /*fc_landed=*/(INFLIGHT == LANDED)),
              Edge::EndedEarly);
}

TEST(SimFlightPolicy, theHoldEndingOnLandedIsTheFlownOutSim)
{
    // And the good exit maps to the hold, so #971's fix keeps working.
    ASSERT_EQ(sim_landed::decide(LANDED, LANDED, 1000),
              sim_landed::Exit::FcLanded);
    EXPECT_EQ(classify(true, false, /*fc_landed=*/(LANDED == LANDED)),
              Edge::EndedLanded);
}

// ── The dry-fire predicate ──────────────────────────────────────────────────

TEST(SimFlightPolicy, dryFireIsLatchedForTheFlight)
{
    // The tick that matters: the sim has already gone idle inside the IMU
    // read, the edge has not been handled yet, and the pyro service runs.
    EXPECT_TRUE(simulated(/*latched=*/true, /*sim_active=*/false))
        << "the pass in which the sim gives up must stay dry";
    // Belt and braces: an active sim is dry whatever the latch says.
    EXPECT_TRUE(simulated(true,  true));
    EXPECT_TRUE(simulated(false, true));
    // A real flight: no sim ever started, nothing latched — live.
    EXPECT_FALSE(simulated(false, false));
}

TEST(SimFlightPolicy, theGiveUpPassStaysDryAndTheResetMakesItLiveAgain)
{
    // Script the loop passes around a give-up, with the two pieces of FC state
    // the rules read: the edge detector's memory and the flight latch.
    bool prev_active = false;
    bool latched     = false;

    // Pass 1: SIM_START arrived — rising edge, reset + latch.
    ASSERT_EQ(classify(prev_active, true, false), Edge::Start);
    latched     = true;    // set after resetFlightStateForSim("start")
    prev_active = true;

    // Passes 2..N: the sim flies; every pyro decision is dry.
    ASSERT_EQ(classify(prev_active, true, false), Edge::None);
    EXPECT_TRUE(simulated(latched, true));

    // Pass N+1: the hold gave up inside the IMU read.  The pyro service runs
    // FIRST, with the sim already idle...
    const bool sim_active_now = false;
    EXPECT_TRUE(simulated(latched, sim_active_now))
        << "sampled isSimActive() would read live here — the latch must not";
    // ...then the edge handler sees the falling edge with the FC INFLIGHT.
    ASSERT_EQ(classify(prev_active, sim_active_now, /*fc_landed=*/false),
              Edge::EndedEarly);
    // resetFlightStateForSim("ended early"): pyros safed, READY, latch off,
    // edge detector re-synced.
    latched     = false;
    prev_active = sim_active_now;

    // Pass N+2: nothing happens, and the FC is live again — in READY, where no
    // channel can leave Idle (servicePyroChannels only runs from INFLIGHT).
    EXPECT_EQ(classify(prev_active, false, false), Edge::None);
    EXPECT_FALSE(simulated(latched, false));
}

TEST(SimFlightPolicy, aUserStopIsNotCountedTwice)
{
    // SIM_STOP_CMD: stopSim() then resetFlightStateForSim("stop"), which
    // re-syncs the edge detector to the now-idle sim.  When the edge handler
    // runs later in the same pass it must see nothing — otherwise every Stop
    // would reset the flight state twice.
    bool prev_active = true;               // sim was flying
    const bool sim_active_after_stop = false;
    prev_active = sim_active_after_stop;   // the re-sync inside the reset
    EXPECT_EQ(classify(prev_active, sim_active_after_stop, false), Edge::None);
}

// ── The Stop gate (#1113) ───────────────────────────────────────────────────

TEST(SimFlightPolicy, aStopWithNoSimThisRunIsAStray)
{
    // The #1113 case: a real flight has landed — post_flight_lockout set, one
    // channel maybe still holding a live e-match — no sim ever started this
    // boot, and a cmd 7 arrives: broadcast, queued at launch and delivered at
    // touchdown, or a stale app banner.  Nothing to stop; the lockout holds.
    EXPECT_FALSE(stopApplies(/*latched=*/false, /*sim_active=*/false))
        << "a real flight's LANDED must stay terminal";
}

TEST(SimFlightPolicy, aStopMidSimApplies)
{
    EXPECT_TRUE(stopApplies(/*latched=*/true, /*sim_active=*/true));
}

TEST(SimFlightPolicy, aStopAfterAFlownOutSimApplies)
{
    // The case the handler always promised to serve: the sim completed, the
    // FC holds LANDED to validate the lockout, isSimActive() is false — and
    // the Stop is how the user re-arms.  Only the latch tells this apart from
    // a real landing.
    EXPECT_TRUE(stopApplies(/*latched=*/true, /*sim_active=*/false));
}

TEST(SimFlightPolicy, aStopInTheStartTickApplies)
{
    // The mini drains its whole command queue in one pass, so a START and a
    // STOP can be handled in the same tick — before the edge handler at the
    // end of the pass has latched.  The active sim alone must carry the Stop.
    EXPECT_TRUE(stopApplies(/*latched=*/false, /*sim_active=*/true));
}

TEST(SimFlightPolicy, aRealLandingAfterAStoppedSimIsStillTerminal)
{
    // One boot: a sim run, stopped by the user, then a real flight — and a
    // stray Stop after the real landing.  Track the two pieces of FC state
    // the rules read.
    bool prev_active = false;
    bool latched     = false;

    // Sim start edge: reset + latch.
    ASSERT_EQ(classify(prev_active, true, false), Edge::Start);
    latched     = true;
    prev_active = true;

    // User Stop mid-sim: applies.  resetFlightStateForSim("stop") drops the
    // latch and re-syncs the edge detector to the now-idle sim.
    ASSERT_TRUE(stopApplies(latched, true));
    latched     = false;
    prev_active = false;
    EXPECT_EQ(classify(prev_active, false, false), Edge::None);

    // The real flight: no edge, nothing latched, every pyro decision live.
    EXPECT_EQ(classify(prev_active, false, false), Edge::None);
    EXPECT_FALSE(simulated(latched, false));

    // Real landing (post_flight_lockout set), then the stray Stop: nothing to
    // stop, and the lockout must hold.
    EXPECT_FALSE(stopApplies(latched, false))
        << "a stray Stop after a real landing must not re-arm";
}

TEST(SimFlightPolicy, aFlownOutSimIsReArmedByStopAndNotByASecondOne)
{
    bool prev_active = false;
    bool latched     = false;

    ASSERT_EQ(classify(prev_active, true, false), Edge::Start);
    latched     = true;
    prev_active = true;

    // The sim flies out: the FC reaches LANDED, the hold sees it, the sim
    // goes idle.  Hold LANDED — and the latch stays, because nothing reset.
    ASSERT_EQ(classify(prev_active, false, /*fc_landed=*/true),
              Edge::EndedLanded);
    prev_active = false;
    EXPECT_TRUE(simulated(latched, false));

    // The user's Stop from LANDED is the re-arm: it applies, and the reset
    // drops the latch.
    ASSERT_TRUE(stopApplies(latched, false));
    latched = false;

    // A repeat (the BS retry train, a second tap) has nothing left to reset.
    EXPECT_FALSE(stopApplies(latched, false));
}

}  // namespace
