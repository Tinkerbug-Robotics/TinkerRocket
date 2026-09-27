// Host tests for GnssAscentGate (owner's rule 2026-09-26): the receiver's
// altitude and vertical velocity are held out of the EKF, and GNSS out of the
// apogee vote, from launch until, after burnout, its altitude has agreed with
// the barometer for a sustained second.
//
// The flight numbers in the comments are from the 2026-09-26 survey of the 26
// historical flights (see the header).

#include <gtest/gtest.h>
#include "GnssAscentGate.h"

#include <cmath>

using namespace GnssAscentGate;

namespace {

// One tick at 1 kHz; a new fix every 55 ms (the SAM-M10Q's 18 Hz).
struct Sim {
    State  st;
    Inputs in;
    Config cfg;
    uint32_t next_fix_ms = 0;
    uint32_t fix_id      = 0;
    uint32_t fix_period  = 55;
    bool     fixes_on    = true;

    Sim()
    {
        in.now_ms        = 1000;
        in.gnss_fix_ok   = true;
        in.baro_healthy  = true;
        in.ekf_valid     = true;
    }

    // Advance `ms` milliseconds; `f(t_ms)` sets the per-tick inputs.
    template <typename F>
    void run(uint32_t ms, F f)
    {
        for (uint32_t i = 0; i < ms; ++i)
        {
            in.now_ms += 1;
            f(in.now_ms);
            if (fixes_on && in.now_ms >= next_fix_ms)
            {
                ++fix_id;
                next_fix_ms = in.now_ms + fix_period;
            }
            in.gnss_fix_id = fixes_on ? fix_id : in.gnss_fix_id;
            step(st, in, cfg);
        }
    }
    void run(uint32_t ms) { run(ms, [](uint32_t) {}); }

    void padThenLaunch(uint32_t pad_ms = 6000)
    {
        in.in_flight = false;
        run(pad_ms);
        onLaunch(st, in.now_ms);
        in.in_flight = true;
    }
};

}  // namespace

// ---------------------------------------------------------------------------
// Phases
// ---------------------------------------------------------------------------
TEST(GnssAscentGate, PadFeedsGnssAsBefore)
{
    Sim s;
    s.run(3000);
    EXPECT_EQ(s.st.phase, Phase::Pad);
    EXPECT_TRUE(gnssVerticalToEkf(s.st));
    EXPECT_TRUE(gnssVotes(s.st));
    EXPECT_TRUE(baroFusable(s.st, true));
}

TEST(GnssAscentGate, LaunchHoldsGnssOut)
{
    Sim s;
    s.padThenLaunch();
    EXPECT_EQ(s.st.phase, Phase::HeldOut);
    EXPECT_FALSE(gnssVerticalToEkf(s.st));
    EXPECT_FALSE(gnssVotes(s.st));
}

TEST(GnssAscentGate, AgreementBeforeBurnoutDoesNotAdmit)
{
    Sim s;
    s.padThenLaunch();
    s.run(10000);   // perfect agreement, no burnout, no apogee
    EXPECT_EQ(s.st.phase, Phase::HeldOut);
}

TEST(GnssAscentGate, AdmitsAfterBurnoutOnceAgreementIsHeldOneSecond)
{
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    s.in.baro_agl_m = 200.0f;
    s.in.gnss_agl_m = 205.0f;
    s.run(900);
    EXPECT_EQ(s.st.phase, Phase::HeldOut) << "admitted before a second of agreement";
    s.run(300);
    EXPECT_EQ(s.st.phase, Phase::Admitted);
    EXPECT_EQ(s.st.reason, Reason::BaroAgreed);
    EXPECT_TRUE(gnssVerticalToEkf(s.st));
    EXPECT_TRUE(gnssVotes(s.st));
}

TEST(GnssAscentGate, AdmissionIsLatchedForTheFlight)
{
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    s.run(1500);
    ASSERT_EQ(s.st.phase, Phase::Admitted);
    s.in.gnss_agl_m = 400.0f;   // wildly off afterwards
    s.run(5000);
    EXPECT_EQ(s.st.phase, Phase::Admitted);
}

TEST(GnssAscentGate, NextLaunchHoldsOutAgain)
{
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    s.run(1500);
    ASSERT_EQ(s.st.phase, Phase::Admitted);
    onLaunch(s.st, s.in.now_ms);
    EXPECT_EQ(s.st.phase, Phase::HeldOut);
}

// ---------------------------------------------------------------------------
// The agreement test
// ---------------------------------------------------------------------------

// Rolly Polly III 06-14: GNSS altitude went from 300 m low to 40 m high,
// sweeping through the barometer.  A sweep crosses the band in well under a
// second; it must not admit.  Once it settles, it must.
TEST(GnssAscentGate, AFixSweepingThroughTheBandDoesNotAdmit)
{
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    s.in.baro_agl_m = 300.0f;               // band = +/-30 m
    float err = -300.0f;
    bool ever = false;
    s.run(5000, [&](uint32_t) {
        err += 0.080f;                      // 80 m/s: the 60 m band in 0.75 s
        s.in.gnss_agl_m = s.in.baro_agl_m + err;
        if (s.st.phase == Phase::Admitted) ever = true;
    });
    EXPECT_FALSE(ever) << "a sweep through the band admitted GNSS";
    // Settle 20 m high, inside the band, and hold.
    s.in.gnss_agl_m = s.in.baro_agl_m + 20.0f;
    s.run(1200);
    EXPECT_EQ(s.st.phase, Phase::Admitted);
}

TEST(GnssAscentGate, DisagreementRestartsTheDwell)
{
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    s.in.baro_agl_m = 100.0f;
    s.in.gnss_agl_m = 100.0f;
    s.run(800);
    s.in.gnss_agl_m = 140.0f;               // one bad stretch
    s.run(100);
    s.in.gnss_agl_m = 100.0f;
    s.run(900);
    EXPECT_EQ(s.st.phase, Phase::HeldOut);
    s.run(200);
    EXPECT_EQ(s.st.phase, Phase::Admitted);
}

TEST(GnssAscentGate, AGapInFixesRestartsTheDwell)
{
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    s.run(800);
    s.fixes_on = false;                     // 0.6 s outage
    s.run(600);
    EXPECT_EQ(s.st.phase, Phase::HeldOut);
    s.fixes_on = true;
    s.run(900);
    EXPECT_EQ(s.st.phase, Phase::HeldOut) << "the outage should have restarted the second";
    s.run(200);
    EXPECT_EQ(s.st.phase, Phase::Admitted);
}

TEST(GnssAscentGate, InvalidFixesNeitherAdmitNorCount)
{
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    s.in.gnss_fix_ok = false;
    s.run(5000);
    EXPECT_EQ(s.st.phase, Phase::HeldOut);
}

// V9 nosecone 08-29: GNSS sat 23 m above the barometer at launch with both
// healthy.  That offset is the datum's, not the ascent's.
TEST(GnssAscentGate, ThePadOffsetDoesNotCountAgainstTheReceiver)
{
    Sim s;
    s.in.gnss_agl_m = 23.0f;
    s.in.baro_agl_m = 0.0f;
    s.padThenLaunch(20000);
    EXPECT_NEAR(s.st.pad_offset_m, 23.0f, 0.5f);
    s.in.burnout = true;
    s.in.baro_agl_m = 60.0f;                // band 15 m
    s.in.gnss_agl_m = 83.0f;                // 23 m above: the pad offset
    s.run(1200);
    EXPECT_EQ(s.st.phase, Phase::Admitted);
}

TEST(GnssAscentGate, PadOffsetIsFrozenAtLaunch)
{
    Sim s;
    s.in.gnss_agl_m = 10.0f;
    s.padThenLaunch(20000);
    const float frozen = s.st.pad_offset_m;
    s.in.gnss_agl_m = 80.0f;                // held out, no burnout: must not learn
    s.run(5000);
    EXPECT_FLOAT_EQ(s.st.pad_offset_m, frozen);
}

// The barometer's own scale error: ISA on a hot or cold day is several
// percent of height (+2..+11 % summer, -3..-17 % spring, over the descents).
TEST(GnssAscentGate, TheBandGrowsWithHeight)
{
    for (float dz : {35.0f, -35.0f})        // 8.75 % of 400 m: inside
    {
        Sim s;
        s.padThenLaunch();
        s.in.burnout = true;
        s.in.baro_agl_m = 400.0f;
        s.in.gnss_agl_m = 400.0f + dz;
        s.run(1200);
        EXPECT_EQ(s.st.phase, Phase::Admitted) << dz;
    }
    for (float dz : {45.0f, -45.0f})        // 11.25 %: outside
    {
        Sim s;
        s.padThenLaunch();
        s.in.burnout = true;
        s.in.baro_agl_m = 400.0f;
        s.in.gnss_agl_m = 400.0f + dz;
        s.run(3000);
        EXPECT_EQ(s.st.phase, Phase::HeldOut) << dz;
    }
    EXPECT_FLOAT_EQ(band(50.0f, Config{}), 15.0f);
    EXPECT_FLOAT_EQ(band(-400.0f, Config{}), 40.0f);
}

TEST(GnssAscentGate, NonFiniteGnssNeverAgrees)
{
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    s.in.gnss_agl_m = NAN;
    s.run(3000);
    EXPECT_EQ(s.st.phase, Phase::HeldOut);
}

TEST(GnssAscentGate, ApogeeWithoutABurnoutStillLetsItQualify)
{
    // A burnout the detector never latched must not keep GNSS out for good.
    Sim s;
    s.padThenLaunch();
    s.in.apogee = true;
    s.run(1200);
    EXPECT_EQ(s.st.phase, Phase::Admitted);
}

// ---------------------------------------------------------------------------
// When the barometer cannot vouch: the EKF's own altitude
// ---------------------------------------------------------------------------
TEST(GnssAscentGate, AnUnhealthyBarometerHandsTheReferenceToTheFilter)
{
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    s.in.baro_healthy = false;
    s.in.baro_agl_m = 0.0f;                 // meaningless now
    s.in.ekf_agl_m  = 500.0f;
    s.in.gnss_agl_m = 480.0f;               // band 50 m
    s.run(1200);
    EXPECT_EQ(s.st.phase, Phase::Admitted);
    EXPECT_EQ(s.st.reason, Reason::FilterAgreed);
}

TEST(GnssAscentGate, TheTransonicLockoutHandsTheReferenceToTheFilter)
{
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    s.in.baro_locked = true;
    s.in.baro_agl_m = 9000.0f;              // a transonic pressure error
    s.in.ekf_agl_m  = 3000.0f;
    s.in.gnss_agl_m = 3010.0f;
    s.run(1200);
    EXPECT_EQ(s.st.phase, Phase::Admitted);
    EXPECT_EQ(s.st.reason, Reason::FilterAgreed);
}

TEST(GnssAscentGate, NoReferenceMeansNoAdmission)
{
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    s.in.baro_healthy = false;
    s.in.ekf_valid = false;
    s.run(5000);
    EXPECT_EQ(s.st.phase, Phase::HeldOut);
}

TEST(GnssAscentGate, SwitchingReferenceMidDwellRestartsIt)
{
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    s.in.baro_agl_m = 100.0f;
    s.in.ekf_agl_m  = 100.0f;
    s.in.gnss_agl_m = 100.0f;
    s.run(800);
    s.in.baro_locked = true;                // now against the filter
    s.run(800);
    EXPECT_EQ(s.st.phase, Phase::HeldOut);
    s.run(300);
    EXPECT_EQ(s.st.phase, Phase::Admitted);
    EXPECT_EQ(s.st.reason, Reason::FilterAgreed);
}

// ---------------------------------------------------------------------------
// The stuck barometer (sealed or taped static port)
// ---------------------------------------------------------------------------
TEST(GnssAscentGate, AFlatBarometerWhileGnssClimbsIsJudgedStuck)
{
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    s.in.baro_agl_m    = 0.0f;              // sealed on the pad
    s.in.baro_rate_mps = 0.0f;
    s.in.gnss_agl_m    = 150.0f;
    s.in.gnss_vel_u_mps = 40.0f;
    s.in.ekf_agl_m     = 3.0f;              // the filter followed the stuck baro
    s.run(1900);
    EXPECT_FALSE(s.st.baro_stuck);
    EXPECT_EQ(s.st.phase, Phase::HeldOut);
    EXPECT_TRUE(baroFusable(s.st, s.in.baro_healthy));
    s.run(200);
    EXPECT_TRUE(s.st.baro_stuck);
    EXPECT_FALSE(baroFusable(s.st, s.in.baro_healthy));
    EXPECT_EQ(s.st.phase, Phase::Admitted);
    EXPECT_EQ(s.st.reason, Reason::BaroStuck);
}

TEST(GnssAscentGate, TheStuckVerdictIsLeaky)
{
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    s.in.gnss_vel_u_mps = 40.0f;
    s.run(2500);
    ASSERT_TRUE(s.st.baro_stuck);
    s.in.apogee = true;                     // the mirror only runs on the climb
    s.run(1900);
    EXPECT_TRUE(s.st.baro_stuck);
    s.run(200);
    EXPECT_FALSE(s.st.baro_stuck);
    EXPECT_TRUE(baroFusable(s.st, true));
    EXPECT_EQ(s.st.phase, Phase::Admitted) << "admission stays latched";
}

// A receiver lag reads LOW on a climb (Rolly Polly V nose: 3-6 m/s through a
// J570), so it cannot fake a flat barometer while GNSS climbs.
TEST(GnssAscentGate, ALaggingReceiverCannotFakeAStuckBarometer)
{
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    s.in.baro_rate_mps  = 80.0f;
    s.in.gnss_vel_u_mps = 3.0f;
    s.in.baro_agl_m = 200.0f;
    s.in.gnss_agl_m = 20.0f;                // and 180 m low
    s.run(10000);
    EXPECT_FALSE(s.st.baro_stuck);
    EXPECT_EQ(s.st.phase, Phase::HeldOut);
}

TEST(GnssAscentGate, TheStuckMirrorWaitsForBurnout)
{
    Sim s;
    s.padThenLaunch();
    s.in.gnss_vel_u_mps = 40.0f;            // boost, flat baro
    s.run(5000);
    EXPECT_FALSE(s.st.baro_stuck);
}

TEST(GnssAscentGate, AnApogeeBarometerIsNotStuck)
{
    // At apogee a healthy barometer is flat for ~0.2 s while a lagging
    // receiver still climbs a few m/s.  Far short of the 2 s dwell.
    Sim s;
    s.padThenLaunch();
    s.in.burnout = true;
    float v = 10.0f;
    s.run(3000, [&](uint32_t) {
        v -= 0.0098f;                       // 1 g deceleration
        s.in.baro_rate_mps  = v;
        s.in.gnss_vel_u_mps = v + 5.0f;     // 0.5 s behind
    });
    EXPECT_FALSE(s.st.baro_stuck);
}

// ---------------------------------------------------------------------------
// Snapshot byte
// ---------------------------------------------------------------------------
TEST(GnssAscentGate, SnapshotByteRoundTrips)
{
    State st;
    EXPECT_EQ(encode(st), CODE_PAD);
    onLaunch(st, 0);
    EXPECT_EQ(encode(st), CODE_HELD_OUT);
    for (auto [why, code] : {std::pair{Reason::BaroAgreed,   CODE_BARO_AGREED},
                             std::pair{Reason::FilterAgreed, CODE_EKF_AGREED},
                             std::pair{Reason::BaroStuck,    CODE_BARO_STUCK}})
    {
        st.phase  = Phase::Admitted;
        st.reason = why;
        EXPECT_EQ(encode(st), code);
        State back;
        restore(back, code, 5000);
        EXPECT_EQ(back.phase, Phase::Admitted);
        EXPECT_EQ(back.reason, why);
    }
    st.baro_stuck = true;
    EXPECT_EQ(encode(st) & CODE_STUCK_BIT, CODE_STUCK_BIT);
}

TEST(GnssAscentGate, AnythingButAnAdmissionRequalifiesAfterAReboot)
{
    for (uint8_t code : {uint8_t{0}, CODE_PAD, CODE_HELD_OUT, uint8_t(CODE_HELD_OUT | CODE_STUCK_BIT), uint8_t{9}})
    {
        State st;
        restore(st, code, 1234);
        EXPECT_EQ(st.phase, Phase::HeldOut) << int(code);
        EXPECT_FALSE(gnssVerticalToEkf(st));
        EXPECT_FALSE(st.baro_stuck) << "the stuck verdict is re-derived, not restored";
    }
}
