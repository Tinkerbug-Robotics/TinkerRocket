// The mosaic-only image's choice of GPS broadcast ionosphere for its raw-GNSS
// filter (flight_computer/main/raw_iono_rule.h): the app's set from a pad
// calibration against the receiver's own GPSIon.
#include <gtest/gtest.h>

#include "raw_iono_rule.h"

namespace
{

constexpr int8_t kA[8] = {12, 7, -60, -60, 75, 49, -65, -65};   // one day's broadcast bytes
constexpr int8_t kB[8] = {13, 7, -60, -60, 76, 49, -65, -65};   // another's
constexpr int8_t kC[8] = {11, 6, -59, -60, 74, 48, -64, -65};

constexpr bool kApp = false;
constexpr bool kRx = true;

}  // namespace

TEST(RawIonoRule, TheFirstSetFromEitherSourceIsTaken)
{
    RawIonoRule::State a, r;
    EXPECT_TRUE(RawIonoRule::offer(a, kA, 17000, kApp));
    EXPECT_TRUE(RawIonoRule::offer(r, kA, 17000, kRx));
    EXPECT_TRUE(a.have && !a.from_rx && a.gps_day == 17000 && a.sets == 1);
    EXPECT_TRUE(r.have && r.from_rx);
    EXPECT_EQ(0, std::memcmp(a.raw, kA, 8));
}

TEST(RawIonoRule, TheSameBytesAreNeverAppliedAgain)
{
    // The receiver's GPSIon is offered every pass of the GNSS task: each
    // re-apply would start the pad survey over
    RawIonoRule::State st;
    ASSERT_TRUE(RawIonoRule::offer(st, kA, 17000, kRx));
    for (int i = 0; i < 100; ++i) EXPECT_FALSE(RawIonoRule::offer(st, kA, 17000, kRx));
    // nor from the other source, nor on a later day
    EXPECT_FALSE(RawIonoRule::offer(st, kA, 17000, kApp));
    EXPECT_FALSE(RawIonoRule::offer(st, kA, 17001, kApp));
    EXPECT_FALSE(RawIonoRule::offer(st, kA, 17002, kRx));
    EXPECT_EQ(1u, st.sets);
    EXPECT_TRUE(st.from_rx);
    EXPECT_EQ(17000, st.gps_day);
}

TEST(RawIonoRule, AnOlderSetIsNotTaken)
{
    RawIonoRule::State st;
    ASSERT_TRUE(RawIonoRule::offer(st, kA, 17000, kRx));
    EXPECT_FALSE(RawIonoRule::offer(st, kB, 16999, kApp));   // last week's file
    EXPECT_FALSE(RawIonoRule::offer(st, kB, 16999, kRx));
    EXPECT_EQ(0, std::memcmp(st.raw, kA, 8));
    // the app's newer file is
    EXPECT_TRUE(RawIonoRule::offer(st, kB, 17001, kApp));
    EXPECT_FALSE(st.from_rx);
    EXPECT_EQ(0, std::memcmp(st.raw, kB, 8));
}

TEST(RawIonoRule, OnTheSameDayTheReceiversWins)
{
    // the app's in use, the receiver's of that day replaces it
    RawIonoRule::State st;
    ASSERT_TRUE(RawIonoRule::offer(st, kA, 17000, kApp));
    EXPECT_TRUE(RawIonoRule::offer(st, kB, 17000, kRx));
    EXPECT_TRUE(st.from_rx);
    // and the app's of that day, sent again, does not take it back
    EXPECT_FALSE(RawIonoRule::offer(st, kA, 17000, kApp));
    EXPECT_FALSE(RawIonoRule::offer(st, kC, 17000, kApp));
    EXPECT_EQ(0, std::memcmp(st.raw, kB, 8));
    // a new upload of a later day does
    EXPECT_TRUE(RawIonoRule::offer(st, kC, 17001, kApp));
    EXPECT_FALSE(st.from_rx);
}

TEST(RawIonoRule, ANewSetOfTheSameSourceAndDayIsTaken)
{
    // the receiver decoding a changed set the same day (its OnChange), or the
    // app's upload replaced the same day: the newer bytes win
    RawIonoRule::State st;
    ASSERT_TRUE(RawIonoRule::offer(st, kA, 17000, kRx));
    EXPECT_TRUE(RawIonoRule::offer(st, kB, 17000, kRx));
    EXPECT_EQ(0, std::memcmp(st.raw, kB, 8));
    RawIonoRule::State app;
    ASSERT_TRUE(RawIonoRule::offer(app, kA, 17000, kApp));
    EXPECT_TRUE(RawIonoRule::offer(app, kB, 17000, kApp));
    EXPECT_EQ(2u, app.sets);
}

TEST(RawIonoRule, TheReceiversSetAcrossDays)
{
    // a pad wait across midnight: the receiver's next day's set is taken, its
    // repeat is not, and yesterday's app file never comes back
    RawIonoRule::State st;
    ASSERT_TRUE(RawIonoRule::offer(st, kA, 17000, kApp));
    EXPECT_TRUE(RawIonoRule::offer(st, kB, 17000, kRx));
    EXPECT_TRUE(RawIonoRule::offer(st, kC, 17001, kRx));
    EXPECT_FALSE(RawIonoRule::offer(st, kC, 17001, kRx));
    EXPECT_FALSE(RawIonoRule::offer(st, kA, 17000, kApp));
    EXPECT_EQ(3u, st.sets);
    EXPECT_EQ(17001, st.gps_day);
}

// ---- the rig replay (offerRigReplay: a TR_SIM_REPLAY_BLOB build of the
// mosaic-only image). The rig's sky is a recorded day: rp5_0829's broadcast
// file is GPS day 17,037 (2026-08-29), the V9's NVS set from an October
// download is about 49 days later.

namespace
{
constexpr uint16_t kRigDay = 17037;
constexpr uint16_t kNvsDay = 17086;   // 2026-10-17
}  // namespace

TEST(RawIonoRuleRig, TheFlightRuleKeepsALaterAppSetOverTheRigsOwn)
{
    // why the rig needs its own rule: by the date rule the NVS set from the
    // real sky's day beats the rig receiver's GPSIon
    RawIonoRule::State st;
    ASSERT_TRUE(RawIonoRule::offer(st, kA, kNvsDay, kApp));
    EXPECT_FALSE(RawIonoRule::offer(st, kB, kRigDay, kRx));
    EXPECT_FALSE(st.from_rx);
}

TEST(RawIonoRuleRig, TheRigReceiversSetReplacesALaterAppSet)
{
    RawIonoRule::State st;
    // the NVS set at boot, before the receiver has heard GPSIon
    ASSERT_TRUE(RawIonoRule::offerRigReplay(st, kA, kNvsDay, kApp));
    EXPECT_FALSE(st.from_rx);
    // the rig's, whatever its day
    EXPECT_TRUE(RawIonoRule::offerRigReplay(st, kB, kRigDay, kRx));
    EXPECT_TRUE(st.from_rx);
    EXPECT_EQ(kRigDay, st.gps_day);
    EXPECT_EQ(0, std::memcmp(st.raw, kB, 8));
    EXPECT_EQ(2u, st.sets);
    // offered every pass after: never applied again
    for (int i = 0; i < 10; ++i) EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kB, kRigDay, kRx));
    EXPECT_EQ(2u, st.sets);
}

TEST(RawIonoRuleRig, AnAppSetNeverDisplacesTheRigReceivers)
{
    RawIonoRule::State st;
    ASSERT_TRUE(RawIonoRule::offerRigReplay(st, kB, kRigDay, kRx));
    // a pad calibration sent mid-run, of any day
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kA, kNvsDay, kApp));
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kC, kRigDay, kApp));
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kC, kRigDay - 1, kApp));
    EXPECT_TRUE(st.from_rx);
    EXPECT_EQ(0, std::memcmp(st.raw, kB, 8));
    EXPECT_EQ(1u, st.sets);
}

TEST(RawIonoRuleRig, WithoutTheReceiversTheAppSetStands)
{
    // with the sky's day not known (0): GPSIon comes once every 12.5 min
    // (subframe 4 page 18), and the rig's three files never carry it, so the
    // app's set is what there is -- the reason a replay build passes the
    // sky's day (below) and carries the sky's own set
    RawIonoRule::State st;
    EXPECT_TRUE(RawIonoRule::offerRigReplay(st, kA, kNvsDay, kApp));
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kA, kNvsDay, kApp));
    // a newer app set by the date rule, as on the real sky
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kC, kNvsDay - 1, kApp));
    EXPECT_TRUE(RawIonoRule::offerRigReplay(st, kC, kNvsDay + 1, kApp));
    EXPECT_EQ(2u, st.sets);
}

TEST(RawIonoRuleRig, TheReceiversOwnSetsKeepTheDateRule)
{
    RawIonoRule::State st;
    ASSERT_TRUE(RawIonoRule::offerRigReplay(st, kA, kRigDay, kRx));
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kB, kRigDay - 1, kRx));   // an older one
    EXPECT_TRUE(RawIonoRule::offerRigReplay(st, kB, kRigDay, kRx));        // changed the same day
    EXPECT_TRUE(RawIonoRule::offerRigReplay(st, kC, kRigDay + 1, kRx));
    EXPECT_EQ(3u, st.sets);
}

TEST(RawIonoRuleRig, TheReceiverConfirmingTheAppsBytesTakesItOverWithoutApplyingIt)
{
    // an app set of the rig's own day (the scenario's file loaded through the
    // app): the receiver's identical GPSIon changes nothing the stage uses,
    // and from then on an app set no longer displaces it
    RawIonoRule::State st;
    ASSERT_TRUE(RawIonoRule::offerRigReplay(st, kA, kRigDay, kApp));
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kA, kRigDay, kRx));
    EXPECT_TRUE(st.from_rx);
    EXPECT_EQ(1u, st.sets);
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kB, kNvsDay, kApp));
    EXPECT_EQ(0, std::memcmp(st.raw, kA, 8));
}

// ---- the sky's day (main.cpp passes the replayed flight's GPS day): the
// guard the reviewers asked for. The rig's files (TOW 573606 on, 241-257 s)
// hold LNAV subframe 4 pages 21-25 and 1-4, never page 18, so without it the
// NVS set of the pad calibration's day stood for the whole run.

TEST(RawIonoRuleRig, NoSetFromAnotherDayThanTheSkysIsTaken)
{
    RawIonoRule::State st;
    // the V9's NVS set from an October pad calibration, at boot
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kA, kNvsDay, kApp, kRigDay));
    // one the receiver decoded from the real sky before its cold start
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kC, kNvsDay, kRx, kRigDay));
    // and an older download than the sky's
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kC, kRigDay - 2, kApp, kRigDay));
    EXPECT_FALSE(st.have);
    EXPECT_EQ(0u, st.sets);
    // nothing taken: the filter builds the sky with no Klobuchar, as before
    // any set, rather than with another day's
}

TEST(RawIonoRuleRig, TheSkysOwnSetIsTakenOnceWhateverWasOfferedFirst)
{
    // a replay build's stand-in for the GPSIon the file never broadcasts
    // (TR_Sensor_Collector_Sim's getGNSSBroadcastIono): offered as the
    // receiver's on every pass of the GNSS task, dated the sky's day
    RawIonoRule::State st;
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kA, kNvsDay, kApp, kRigDay));   // the NVS set first
    EXPECT_TRUE(RawIonoRule::offerRigReplay(st, kB, kRigDay, kRx, kRigDay));
    for (int i = 0; i < 10; ++i) EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kB, kRigDay, kRx, kRigDay));
    // a pad calibration mid-run changes nothing
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kA, kNvsDay, kApp, kRigDay));
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kC, kRigDay, kApp, kRigDay));
    EXPECT_TRUE(st.from_rx);
    EXPECT_EQ(kRigDay, st.gps_day);
    EXPECT_EQ(0, std::memcmp(st.raw, kB, 8));
    EXPECT_EQ(1u, st.sets);
}

TEST(RawIonoRuleRig, ASetWithinADayOfTheSkysPasses)
{
    // the sky's file starts 180 s before ignition and may cross midnight; an
    // app set loaded from the scenario's own broadcast file is the sky's
    for (int d : {-1, 0, 1})
    {
        RawIonoRule::State st;
        EXPECT_TRUE(RawIonoRule::offerRigReplay(st, kA, static_cast<uint16_t>(kRigDay + d), kApp, kRigDay)) << d;
    }
    RawIonoRule::State st;
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kA, kRigDay + 2, kApp, kRigDay));
    EXPECT_FALSE(RawIonoRule::offerRigReplay(st, kA, kRigDay - 2, kRx, kRigDay));
}
