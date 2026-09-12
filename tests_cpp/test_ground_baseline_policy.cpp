// #1150 — the comms side's ground-pressure reference after it joined a flight
// in progress.
//
// The finding: an out computer that reset in flight re-joined the FC's stream
// with its rocket-state cache at the boot default, took its "pad" reference
// from the first baro frame -- at altitude -- and reported ~0 m for the rest
// of the flight. Two rules close it. The reference may only be tracked while
// the flight side's state is KNOWN and not INFLIGHT; and a comms side that is
// INFLIGHT with no reference adopts the flight side's own, from the snapshot
// it already streams, exactly once. Both are pinned here, and the issue's
// sequence is walked end to end.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "ground_baseline_policy.h"

using ground_baseline::Adoption;
using ground_baseline::Verdict;
using ground_baseline::adoptFromSnapshot;
using ground_baseline::trackFromBaro;

namespace
{

FlightSnapshotData inflightSnapshot(float p0 = 98500.0f,
                                    float max_alt = 812.5f,
                                    float max_speed = 140.0f)
{
    FlightSnapshotData s = {};
    s.magic              = FlightSnapshotData::MAGIC;
    s.version            = FlightSnapshotData::VERSION;
    s.rocket_state       = (uint8_t)INFLIGHT;
    s.flight_elapsed_ms  = 12345;
    s.ground_pressure_pa = p0;
    s.max_alt_m          = max_alt;
    s.max_speed_mps      = max_speed;
    return s;
}

}  // namespace

// ---- Rule 1: tracking the reference from baro samples ----------------------

TEST(GroundBaselinePolicy, UnknownStateNeverTracks)
{
    // The boot-default enum is not evidence, whatever it happens to read.
    EXPECT_FALSE(trackFromBaro(/*known=*/false, /*inflight=*/false));
    EXPECT_FALSE(trackFromBaro(/*known=*/false, /*inflight=*/true));
}

TEST(GroundBaselinePolicy, KnownGroundStateTracks)
{
    EXPECT_TRUE(trackFromBaro(/*known=*/true, /*inflight=*/false));
}

TEST(GroundBaselinePolicy, KnownInflightNeverTracks)
{
    EXPECT_FALSE(trackFromBaro(/*known=*/true, /*inflight=*/true));
}

// ---- Rule 2: adopting the flight side's reference ---------------------------

TEST(GroundBaselinePolicy, AdoptsWhenNoReferenceForTheFlight)
{
    Adoption out;
    const auto v = adoptFromSnapshot(/*own_baseline_set=*/false,
                                     inflightSnapshot(98500.0f, 812.5f, 140.0f), out);
    EXPECT_EQ(v, Verdict::Adopt);
    EXPECT_FLOAT_EQ(out.ground_pressure_pa, 98500.0f);
    EXPECT_FLOAT_EQ(out.max_alt_m, 812.5f);
    EXPECT_FLOAT_EQ(out.max_speed_mps, 140.0f);
}

TEST(GroundBaselinePolicy, ACommsSideThatWatchedThePadKeepsItsOwn)
{
    // The nominal flight: the reference it tracked itself stays, and the
    // output is left untouched so a careless caller cannot install anything.
    Adoption out;
    out.ground_pressure_pa = 1.0f;
    const auto v = adoptFromSnapshot(/*own_baseline_set=*/true, inflightSnapshot(), out);
    EXPECT_EQ(v, Verdict::HaveBaseline);
    EXPECT_FLOAT_EQ(out.ground_pressure_pa, 1.0f);
}

TEST(GroundBaselinePolicy, AdoptionIsOneShot)
{
    // Once adopted, the caller's own flag is set and every later snapshot is
    // HaveBaseline -- the reference does not follow the stream.
    Adoption out;
    bool own_set = false;
    ASSERT_EQ(adoptFromSnapshot(own_set, inflightSnapshot(98500.0f), out), Verdict::Adopt);
    own_set = true;
    EXPECT_EQ(adoptFromSnapshot(own_set, inflightSnapshot(98000.0f), out), Verdict::HaveBaseline);
    EXPECT_FLOAT_EQ(out.ground_pressure_pa, 98500.0f);
}

TEST(GroundBaselinePolicy, TheLandedClearAdoptsNothing)
{
    // clearFlightSnapshot() sends the same struct with rocket_state=LANDED;
    // a comms side with no reference must not take a landing-site "pad".
    FlightSnapshotData s = inflightSnapshot();
    s.rocket_state = (uint8_t)LANDED;
    Adoption out;
    EXPECT_EQ(adoptFromSnapshot(false, s, out), Verdict::NotInflight);
    EXPECT_FLOAT_EQ(out.ground_pressure_pa, 0.0f);
}

TEST(GroundBaselinePolicy, EveryOtherStateAdoptsNothing)
{
    for (uint8_t st : {(uint8_t)INITIALIZATION, (uint8_t)READY, (uint8_t)PRELAUNCH,
                       (uint8_t)LANDED, (uint8_t)MAG_CALIBRATION})
    {
        FlightSnapshotData s = inflightSnapshot();
        s.rocket_state = st;
        Adoption out;
        EXPECT_EQ(adoptFromSnapshot(false, s, out), Verdict::NotInflight) << "state " << (int)st;
    }
}

TEST(GroundBaselinePolicy, WrongMagicIsRejected)
{
    FlightSnapshotData s = inflightSnapshot();
    s.magic = 0xF1A7C0DE;   // the old NVS magic
    Adoption out;
    EXPECT_EQ(adoptFromSnapshot(false, s, out), Verdict::Rejected);
}

TEST(GroundBaselinePolicy, WrongVersionIsRejected)
{
    // v4 carried Euler angles where v5 carries the maxima: a mismatched FC
    // image must not have its cached angles latched as an apogee.
    FlightSnapshotData s = inflightSnapshot();
    s.version = 4;
    Adoption out;
    EXPECT_EQ(adoptFromSnapshot(false, s, out), Verdict::Rejected);
}

TEST(GroundBaselinePolicy, RejectionOutranksTheClear)
{
    // A frame that cannot be trusted is Rejected whatever its state says.
    FlightSnapshotData s = inflightSnapshot();
    s.magic        = 0;
    s.rocket_state = (uint8_t)LANDED;
    Adoption out;
    EXPECT_EQ(adoptFromSnapshot(false, s, out), Verdict::Rejected);
}

TEST(GroundBaselinePolicy, PressureOutsideTheBmpBandIsRejected)
{
    Adoption out;
    for (float p0 : {0.0f, -5.0f, BMP_PRESSURE_MIN_PA - 1.0f, BMP_PRESSURE_MAX_PA + 1.0f,
                     std::numeric_limits<float>::quiet_NaN(),
                     std::numeric_limits<float>::infinity()})
    {
        EXPECT_EQ(adoptFromSnapshot(false, inflightSnapshot(p0), out), Verdict::Rejected)
            << "p0 " << p0;
    }
    // The band edges themselves are in.
    EXPECT_EQ(adoptFromSnapshot(false, inflightSnapshot(BMP_PRESSURE_MIN_PA), out), Verdict::Adopt);
    EXPECT_EQ(adoptFromSnapshot(false, inflightSnapshot(BMP_PRESSURE_MAX_PA), out), Verdict::Adopt);
}

TEST(GroundBaselinePolicy, GarbageMaximaContributeNothingButKeepTheReference)
{
    // The reference is the load-bearing field; a corrupt maximum folds into
    // the caller's max() as 0 rather than costing the adoption.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    struct Case { float alt, speed; };
    for (const Case& c : {Case{nan, 10.0f}, Case{1.0e9f, 10.0f}, Case{-600.0f, 10.0f},
                          Case{10.0f, nan}, Case{10.0f, 5000.0f}, Case{10.0f, -1.0f}})
    {
        Adoption out;
        ASSERT_EQ(adoptFromSnapshot(false, inflightSnapshot(98500.0f, c.alt, c.speed), out),
                  Verdict::Adopt);
        EXPECT_FLOAT_EQ(out.ground_pressure_pa, 98500.0f);
        const bool alt_ok   = c.alt > ground_baseline::kMaxAltMinM &&
                              c.alt < ground_baseline::kMaxAltMaxM;
        const bool speed_ok = c.speed >= 0.0f && c.speed <= ground_baseline::kMaxSpeedMps;
        EXPECT_FLOAT_EQ(out.max_alt_m, alt_ok ? c.alt : 0.0f);
        EXPECT_FLOAT_EQ(out.max_speed_mps, speed_ok ? c.speed : 0.0f);
    }
}

TEST(GroundBaselinePolicy, ZeroMaximaAreFine)
{
    // A flight that reboots before it climbed: 0 is a legitimate maximum.
    Adoption out;
    ASSERT_EQ(adoptFromSnapshot(false, inflightSnapshot(98500.0f, 0.0f, 0.0f), out), Verdict::Adopt);
    EXPECT_FLOAT_EQ(out.max_alt_m, 0.0f);
    EXPECT_FLOAT_EQ(out.max_speed_mps, 0.0f);
}

// ---- The issue's sequence, end to end -----------------------------------------

// A minimal model of the comms side's state as main.cpp / comms.cpp keep it.
struct CommsSide
{
    bool  fc_state_known = false;   // latest_non_sensor_valid
    bool  fc_inflight    = false;   // latest_rocket_state == INFLIGHT
    float p0             = 101325.0f;
    bool  p0_set         = false;
    float max_alt        = 0.0f;

    // updateDerivedAltitudeFromBMP()
    float baro(float p)
    {
        if (trackFromBaro(fc_state_known, fc_inflight))
        {
            p0 = p;
            p0_set = true;
        }
        const float alt = p0_set ? 44330.0f * (1.0f - std::pow(p / p0, 1.0f / 5.255f)) : 0.0f;
        if (alt > ground_baseline::kMaxAltMinM && alt < ground_baseline::kMaxAltMaxM)
            max_alt = std::max(max_alt, alt);
        return alt;
    }
    // NON_SENSOR_MSG
    void nonSensor(bool inflight)
    {
        fc_state_known = true;
        fc_inflight = inflight;
    }
    // SNAPSHOT_MSG
    Verdict snapshot(const FlightSnapshotData& s)
    {
        Adoption a;
        const Verdict v = adoptFromSnapshot(p0_set, s, a);
        if (v == Verdict::Adopt)
        {
            p0 = a.ground_pressure_pa;
            p0_set = true;
            max_alt = std::max(max_alt, a.max_alt_m);
        }
        return v;
    }
};

constexpr float kPadPa     = 101000.0f;
constexpr float k800mPa    = 91800.0f;   // roughly 800 m above the pad
constexpr float k1000mPa   = 89600.0f;

TEST(GroundBaselineScenario, TheIssueAsFiled_ARestartAtAltitudeUsedToRebaseline)
{
    // Pre-#1150 behaviour, for the record: the first baro frame after the
    // reset became the reference because the state read "not INFLIGHT".
    CommsSide oc;
    const bool old_rule = (false /*INITIALIZATION*/ != true /*INFLIGHT*/);
    EXPECT_TRUE(old_rule);   // the default-initialised enum passed the old gate
    // New rule: the same frame is held.
    EXPECT_FLOAT_EQ(oc.baro(k800mPa), 0.0f);
    EXPECT_FALSE(oc.p0_set);
}

TEST(GroundBaselineScenario, RestartAtAltitude_SnapshotRestoresTheFlight)
{
    CommsSide oc;                       // fresh boot mid-flight, all defaults
    oc.baro(k800mPa);                   // the frame that used to win the race
    EXPECT_FALSE(oc.p0_set);
    oc.nonSensor(/*inflight=*/true);    // the FC's word arrives
    EXPECT_FLOAT_EQ(oc.baro(k800mPa), 0.0f);   // still held, still 0
    // Within one snapshot period the FC's own reference and maxima land.
    EXPECT_EQ(oc.snapshot(inflightSnapshot(kPadPa, 812.5f, 140.0f)), Verdict::Adopt);
    EXPECT_FLOAT_EQ(oc.p0, kPadPa);
    EXPECT_FLOAT_EQ(oc.max_alt, 812.5f);
    // From here the OC's own baro math is right again: ~800 m, climbing to
    // ~1000 m, and max_alt keeps the FC's figure until the vehicle beats it.
    EXPECT_NEAR(oc.baro(k800mPa), 800.0f, 25.0f);
    EXPECT_NEAR(oc.baro(k1000mPa), 1000.0f, 30.0f);
    EXPECT_NEAR(oc.max_alt, 1000.0f, 30.0f);
    // Later snapshots change nothing.
    EXPECT_EQ(oc.snapshot(inflightSnapshot(kPadPa - 100.0f)), Verdict::HaveBaseline);
    EXPECT_FLOAT_EQ(oc.p0, kPadPa);
}

TEST(GroundBaselineScenario, NominalFlightIsUntouched)
{
    // Booted on the pad, watched READY/PRELAUNCH, launched: the reference it
    // tracked itself is the one it flies with, snapshots or not.
    CommsSide oc;
    oc.nonSensor(/*inflight=*/false);
    oc.baro(kPadPa + 20.0f);
    oc.baro(kPadPa);                    // last pad sample before launch detection
    EXPECT_TRUE(oc.p0_set);
    oc.nonSensor(/*inflight=*/true);
    EXPECT_EQ(oc.snapshot(inflightSnapshot(kPadPa - 300.0f, 5.0f, 2.0f)), Verdict::HaveBaseline);
    EXPECT_FLOAT_EQ(oc.p0, kPadPa);
    EXPECT_NEAR(oc.baro(k800mPa), 800.0f, 25.0f);
}

TEST(GroundBaselineScenario, LandingStillRebaselinesAtTheSiteAsBefore)
{
    // Unchanged behaviour: LANDED is a known non-INFLIGHT state, so the
    // reference tracks the landing site again (the issue noted this and it
    // is what the pre-#1150 code did too).
    CommsSide oc;
    oc.nonSensor(true);
    oc.snapshot(inflightSnapshot(kPadPa));
    oc.nonSensor(false);                // LANDED
    oc.baro(k800mPa);
    EXPECT_FLOAT_EQ(oc.p0, k800mPa);
}

TEST(GroundBaselineScenario, BothMcuResetWhoseFcRestoredInSetup)
{
    // The FC restores INFLIGHT inside setup_fc, so its first NonSensor frame
    // is already INFLIGHT and no pad state is ever seen this boot. Same path
    // as the OC-only reset: hold, then adopt.
    CommsSide oc;
    oc.nonSensor(/*inflight=*/true);
    EXPECT_FLOAT_EQ(oc.baro(k800mPa), 0.0f);
    EXPECT_EQ(oc.snapshot(inflightSnapshot(kPadPa, 400.0f, 90.0f)), Verdict::Adopt);
    EXPECT_NEAR(oc.baro(k800mPa), 800.0f, 25.0f);
}
