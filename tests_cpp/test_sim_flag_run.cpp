// #1774: the OC believes NSF_SIM_ACTIVE only once ten frames in a row carry it
// (out_computer/main/sim_flag_run.h).
//
// Before, one frame decided. A NonSensorData frame with flags bit 7 and payload
// byte 6 bit 6 flipped passes the link's CRC-16 (the two bits are 255 apart,
// the order of x modulo x^16+x^15+1). One such frame between launch and the
// last deployment would latch "sim", hold arm consent low until LANDED, and
// lose every later deployment. The same frame would make the landing a sim's
// for the post-landing log lockout.
//
// Here the real CRC (crc16_fast.h) and the real NonSensorData layout make that
// frame, and both OC policies are fed the stream around it.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "RocketComputerTypes.h"
#include "arm_consent_policy.h"
#include "crc16_fast.h"
#include "inflight_refusal_policy.h"
#include "landed_lockout_policy.h"
#include "sim_flag_run.h"

namespace {

using SimFlagRun::kFrames;

// The CRC-covered part of an FC->OC frame: type, length, payload
// (TR_I2C_Interface.cpp: crc16Frame(frame + 4, 1 + 1 + payload_len)).
struct Wire
{
    std::vector<uint8_t> covered;
    uint16_t crc = 0;
};

Wire frameOf(const NonSensorData& ns)
{
    Wire w;
    w.covered.push_back(NON_SENSOR_MSG);
    w.covered.push_back((uint8_t)sizeof(NonSensorData));
    const auto* p = reinterpret_cast<const uint8_t*>(&ns);
    w.covered.insert(w.covered.end(), p, p + sizeof(NonSensorData));
    w.crc = crc16Frame(w.covered.data(), w.covered.size());
    return w;
}

NonSensorData decode(const Wire& w)
{
    NonSensorData ns;
    std::memcpy(&ns, w.covered.data() + 2, sizeof(ns));
    return ns;
}

NonSensorData flightFrame(uint32_t i)
{
    NonSensorData ns{};
    ns.time_us      = 1000000u + 2000u * i;   // 500 Hz
    ns.q0           = (int16_t)(12000 + (i % 97));
    ns.q1           = (int16_t)(300 + (i % 53));
    ns.flags        = NSF_LAUNCH | NSF_PYRO_ARMED;
    ns.rocket_state = INFLIGHT;
    return ns;
}

// The issue's 2-bit error: flags bit 7 (NSF_SIM_ACTIVE) and payload byte 6
// bit 6 (in q1's low byte).
Wire corrupt(Wire w)
{
    w.covered[2 + offsetof(NonSensorData, flags)] ^= NSF_SIM_ACTIVE;
    w.covered[2 + 6] ^= (uint8_t)(1u << 6);
    return w;   // the CRC as sent
}

bool crcPasses(const Wire& w)
{
    return crc16Frame(w.covered.data(), w.covered.size()) == w.crc;
}

// What the OC does with one frame that passed the CRC: both steps, as
// processFrame feeds them, and consent as serviceArmConsent() decides it with a
// live FC.
struct Oc
{
    ArmConsentPolicy::SimFlightLatch latch;
    LandedLockout::State             lockout;

    ArmConsentPolicy::Reason take(const NonSensorData& ns)
    {
        const bool inflight = (RocketState)ns.rocket_state == INFLIGHT;
        const bool sim_bit  = (ns.flags & NSF_SIM_ACTIVE) != 0;
        const bool is_sim   = ArmConsentPolicy::step(latch, inflight, sim_bit);
        LandedLockout::step(lockout, (RocketState)ns.rocket_state, sim_bit);
        return ArmConsentPolicy::decide(false, InflightRefusalPolicy::refuse(inflight, true, 0),
                                        is_sim);
    }
};

}  // namespace

// ---------------------------------------------------------------------------
// The run itself
// ---------------------------------------------------------------------------

TEST(SimFlagRun, TheFlagCountsOnlyOnItsTenthFrameInARow)
{
    SimFlagRun::Run r;
    for (uint8_t i = 1; i < kFrames; ++i) EXPECT_FALSE(SimFlagRun::step(r, true)) << int(i);
    EXPECT_TRUE(SimFlagRun::step(r, true));
    for (int i = 0; i < 1000; ++i) ASSERT_TRUE(SimFlagRun::step(r, true)) << i;   // held, no wrap
}

TEST(SimFlagRun, AFrameWithoutTheFlagStartsTheCountAgain)
{
    SimFlagRun::Run r;
    for (uint8_t i = 0; i < kFrames; ++i) SimFlagRun::step(r, true);
    EXPECT_FALSE(SimFlagRun::step(r, false));
    for (uint8_t i = 1; i < kFrames; ++i) EXPECT_FALSE(SimFlagRun::step(r, true)) << int(i);
    EXPECT_TRUE(SimFlagRun::step(r, true));
    // a run of nine, a gap, a run of nine: never ten in a row
    SimFlagRun::Run g;
    for (int k = 0; k < 2; ++k)
    {
        for (uint8_t i = 1; i < kFrames; ++i) EXPECT_FALSE(SimFlagRun::step(g, true));
        EXPECT_FALSE(SimFlagRun::step(g, false));
    }
}

TEST(SimFlagRun, TenFramesIsTwentyMillisecondsAtTheFcsRate)
{
    // NonSensorData goes out at 500 Hz (flight_computer config.h). A sim
    // carries the flag from its Start, at least its own 5 s pad before its
    // launch (the FC's 3 s GNSS dwell in READY runs inside it).
    constexpr uint32_t kNonSensorHz = 500;
    EXPECT_EQ(kFrames * 1000u / kNonSensorHz, 20u);
}

// ---------------------------------------------------------------------------
// The issue's frame
// ---------------------------------------------------------------------------

TEST(SimFlagRunIssueFrame, TheTwoBitErrorPassesTheCrcAndKeepsTheState)
{
    // Why the OC cannot act on one frame: this error is invisible to the link.
    static_assert(offsetof(NonSensorData, flags) == 38, "flags moved");
    static_assert(offsetof(NonSensorData, rocket_state) == 39, "rocket_state moved");
    for (uint32_t i = 0; i < 1000; ++i)
    {
        const Wire bad = corrupt(frameOf(flightFrame(i)));
        ASSERT_TRUE(crcPasses(bad)) << i;
        const NonSensorData ns = decode(bad);
        ASSERT_EQ(ns.rocket_state, (uint8_t)INFLIGHT);
        ASSERT_TRUE(ns.flags & NSF_SIM_ACTIVE);
    }
}

TEST(SimFlagRunIssueFrame, ARealFlightKeepsConsentThroughIt)
{
    Oc oc;
    NonSensorData pad = flightFrame(0);
    pad.rocket_state  = PRELAUNCH;
    pad.flags         = NSF_PYRO_ARMED;
    for (int i = 0; i < 2500; ++i) EXPECT_EQ(oc.take(pad), ArmConsentPolicy::Reason::None);

    uint32_t t = 0;
    for (; t < 3000; ++t) ASSERT_EQ(oc.take(flightFrame(t)), ArmConsentPolicy::Reason::Flight);

    // the frame, 6 s after launch: before apogee and the drogue
    const Wire bad = corrupt(frameOf(flightFrame(t++)));
    ASSERT_TRUE(crcPasses(bad));
    EXPECT_EQ(oc.take(decode(bad)), ArmConsentPolicy::Reason::Flight);

    // the 30,000 clean frames that the old latch held consent low through
    // (60 s: the drogue and the main both fall in them)
    for (uint32_t k = 0; k < 30000; ++k, ++t)
        ASSERT_EQ(oc.take(flightFrame(t)), ArmConsentPolicy::Reason::Flight) << k;
    EXPECT_FALSE(oc.lockout.flight_was_sim);
}

TEST(SimFlagRunIssueFrame, TheLandingStaysARealOnesAndTheLockoutHolds)
{
    // The same frame would make the landing a sim's, so the lockout would
    // clear when an FC reset walked the FC out of LANDED.
    Oc oc;
    uint32_t t = 0;
    for (; t < 3000; ++t) oc.take(flightFrame(t));
    oc.take(decode(corrupt(frameOf(flightFrame(t++)))));
    for (uint32_t k = 0; k < 3000; ++k, ++t) oc.take(flightFrame(t));
    NonSensorData landed = flightFrame(t);
    landed.rocket_state  = LANDED;
    for (int k = 0; k < 500; ++k) oc.take(landed);
    EXPECT_TRUE(oc.lockout.locked);
    EXPECT_FALSE(oc.lockout.landed_was_sim);
    // ...and a corrupted frame in the held LANDED does not do it either
    NonSensorData bad = decode(corrupt(frameOf(landed)));
    oc.take(bad);
    for (int k = 0; k < 500; ++k) oc.take(landed);
    EXPECT_FALSE(oc.lockout.landed_was_sim);
    // the FC resets: INITIALIZATION, READY, PRELAUNCH, still locked
    for (RocketState st : {INITIALIZATION, READY, PRELAUNCH})
    {
        NonSensorData f = landed;
        f.rocket_state  = st;
        f.flags         = 0;
        for (int k = 0; k < 50; ++k)
        {
            oc.take(f);
            EXPECT_TRUE(oc.lockout.locked) << int(st);
        }
        // one corrupted frame there does not clear it ((a) is a sim's start)
        const Wire bad_off_landed = corrupt(frameOf(f));
        ASSERT_TRUE(crcPasses(bad_off_landed));
        oc.take(decode(bad_off_landed));
        EXPECT_TRUE(oc.lockout.locked) << int(st);
    }
}

TEST(SimFlagRunIssueFrame, ABenchSimStillHoldsConsentLowFromItsFirstInflightFrame)
{
    // The sim's Start sets the flag on the pad, and its 5 s pad phase starts
    // there: the FC's 3 s GNSS dwell in READY, then 2 s of PRELAUNCH.
    Oc oc;
    NonSensorData pad = flightFrame(0);
    pad.rocket_state  = READY;
    pad.flags         = NSF_SIM_ACTIVE;
    for (int i = 0; i < 1500; ++i) oc.take(pad);   // the 3 s GNSS dwell
    pad.rocket_state = PRELAUNCH;
    for (int i = 0; i < 1000; ++i) oc.take(pad);   // the rest of the sim's 5 s pad
    for (uint32_t t = 0; t < 5000; ++t)
    {
        NonSensorData f = flightFrame(t);
        f.flags |= NSF_SIM_ACTIVE;
        ASSERT_EQ(oc.take(f), ArmConsentPolicy::Reason::None) << t;
    }
    EXPECT_TRUE(oc.lockout.flight_was_sim);
}
