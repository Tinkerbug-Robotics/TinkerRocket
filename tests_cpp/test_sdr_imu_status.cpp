// Host tests for whether the gnss-sdr-dev board's own IMU aids its tracking
// loops, as the flight computer reports it to the apps' health dot
// (flight_computer/main/sdr_imu_status_policy.h): from each epoch's aiding
// block (4001 sub-type 1, SdrAid.cpp decodeEpochAid) and the board's mounting
// report (4001 sub-type 5, SdrRtcm.cpp decodeTrAlign). The epoch frames are
// test_sdr_aid.cpp's goldens.
#include <gtest/gtest.h>

#include <initializer_list>
#include <vector>

#include "RocketComputerTypes.h"
#include "SdrAid.h"
#include "SdrRtcm.h"
#include "sdr_imu_status_policy.h"

namespace {

using sdr_imu_status::Tracker;
using Bytes = std::vector<uint8_t>;

// test_sdr_aid.cpp's E1: a version 1 block, the board's IMU aiding
const Bytes kE1 = {
    0xD3, 0x00, 0x22, 0xFA, 0x11, 0x09, 0x57, 0x90, 0x32, 0x0F, 0xDA, 0x17,
    0xB8, 0x7F, 0xFF, 0xFF, 0xFF, 0xFE, 0x29, 0x0C, 0xBA, 0xD6, 0xC4, 0x8D,
    0x15, 0x9E, 0x26, 0xAF, 0x03, 0x0A, 0xEF, 0x01, 0x86, 0x34, 0xF0, 0x29,
    0x40, 0x7C, 0x97, 0xF2};
// and E4: a version 2 block, the IMU refused by the pad check
const Bytes kE4 = {
    0xD3, 0x00, 0x29, 0xFA, 0x11, 0x09, 0x57, 0x90, 0x32, 0x0F, 0xDA, 0x17,
    0xB8, 0x7F, 0xFF, 0xFF, 0xFF, 0xFE, 0x29, 0x0C, 0xBA, 0xD6, 0xC4, 0x8D,
    0x15, 0x9E, 0x26, 0xAF, 0x03, 0x0A, 0xEF, 0x01, 0x88, 0xA4, 0xB3, 0x8F,
    0x00, 0xD2, 0x40, 0x7F, 0xF7, 0xA0, 0x5C, 0x00, 0x8A, 0xF2, 0xDB};

constexpr int64_t kT = 50'000'000;   // some time after boot, us

sdr_aid::EpochAid block(uint8_t src, uint8_t check, uint8_t why, uint8_t ver = 2)
{
    sdr_aid::EpochAid a;
    a.present = true;
    a.ver = ver;
    a.src = src;
    a.check = check;
    a.why = why;
    return a;
}

sdr_rtcm::TrAlign report(uint8_t imu, uint8_t have = sdr_rtcm::TrAlign::kHaveWhole)
{
    sdr_rtcm::TrAlign al;
    al.imu = imu;
    al.have = have;
    return al;
}

}  // namespace

TEST(SdrImuStatus, NoBoardIsNotApplicable)
{
    Tracker t;
    EXPECT_EQ(t.state(false, kT), SH_NA);
    t.noteAid(block(2, 1, sdr_aid::kWhyAids), kT);
    EXPECT_EQ(t.state(false, kT), SH_NA);   // another receiver on J1: the board's frames do not count
}

TEST(SdrImuStatus, ABoardThatSaysNothingIsDegraded)
{
    Tracker t;
    EXPECT_EQ(t.state(true, kT), SH_DEGRADED);
    sdr_aid::EpochAid none;   // an epoch with no block (an older board image)
    t.noteAid(none, kT);
    EXPECT_EQ(t.state(true, kT), SH_DEGRADED);
}

TEST(SdrImuStatus, TheGoldenEpochs)
{
    Tracker t;
    sdr_aid::EpochAid a;
    ASSERT_TRUE(sdr_aid::decodeEpochAid(kE1.data(), kE1.size(), a));
    t.noteAid(a, kT);
    EXPECT_EQ(t.state(true, kT), SH_OK);
    ASSERT_TRUE(sdr_aid::decodeEpochAid(kE4.data(), kE4.size(), a));
    t.noteAid(a, kT + 10'000);
    EXPECT_EQ(t.state(true, kT + 10'000), SH_BAD);
}

TEST(SdrImuStatus, AidingIsOk)
{
    Tracker t;
    t.noteAid(block(2, 1, sdr_aid::kWhyAids), kT);
    EXPECT_EQ(t.state(true, kT), SH_OK);
    t.noteAid(block(2, 0, sdr_aid::kWhyAids, 1), kT);   // version 1 says no reason
    EXPECT_EQ(t.state(true, kT), SH_OK);
}

TEST(SdrImuStatus, AidingTheVerticalOnlyIsDegraded)
{
    // The board aids with its own levelling before it knows its heading: src 2 (its IMU), why 9.
    Tracker t;
    t.noteAid(block(2, 1, sdr_aid::kWhyVertical), kT);
    EXPECT_EQ(t.state(true, kT), SH_DEGRADED);
    t.noteAid(block(2, 1, sdr_aid::kWhyAids), kT + 10'000);   // its heading found: a full aid
    EXPECT_EQ(t.state(true, kT + 10'000), SH_OK);
    t.noteAid(block(2, 0, sdr_aid::kWhyVertical, 1), kT);      // version 1's bits there are reserved
    EXPECT_EQ(t.state(true, kT), SH_OK);
    // A whole mounting that disagrees with this flight computer's attitude: the board's own, the vertical only
    t.noteAid(block(2, 1, sdr_aid::kWhyMountDisagrees), kT);
    EXPECT_EQ(t.state(true, kT), SH_DEGRADED);
    t.noteAid(block(2, 1, 13), kT);                               // a reason from a later board: not full either
    EXPECT_EQ(t.state(true, kT), SH_DEGRADED);
}

TEST(SdrImuStatus, NotAidingByChoiceOrNoMountingIsDegraded)
{
    Tracker t;
    for (uint8_t why : std::initializer_list<uint8_t>{sdr_aid::kWhyAids, sdr_aid::kWhyLetGo, sdr_aid::kWhyNoMount, 9, 15})
    {
        t.noteAid(block(1, 0, why), kT);   // the GPS-derived rate aids
        EXPECT_EQ(t.state(true, kT), SH_DEGRADED) << int(why);
        t.noteAid(block(0, 0, why), kT);   // nothing aids
        EXPECT_EQ(t.state(true, kT), SH_DEGRADED) << int(why);
    }
}

TEST(SdrImuStatus, RefusedOrDroppedIsBad)
{
    Tracker t;
    t.noteAid(block(0, 2, sdr_aid::kWhyAids), kT);   // the pad check refused it
    EXPECT_EQ(t.state(true, kT), SH_BAD);
    t.noteAid(block(0, 2, sdr_aid::kWhyAids, 1), kT);   // so did version 1's
    EXPECT_EQ(t.state(true, kT), SH_BAD);
    for (uint8_t why : {sdr_aid::kWhyStale, sdr_aid::kWhyGap, sdr_aid::kWhyAttitude, sdr_aid::kWhyNaN,
                        sdr_aid::kWhyRange, sdr_aid::kWhyPadCheck})
    {
        t.noteAid(block(1, 1, why), kT);
        EXPECT_EQ(t.state(true, kT), SH_BAD) << int(why);
    }
    // Version 1's bits there are reserved: never read as a drop
    t.noteAid(block(1, 1, sdr_aid::kWhyGap, 1), kT);
    EXPECT_EQ(t.state(true, kT), SH_DEGRADED);
}

TEST(SdrImuStatus, TheEpochsGoStale)
{
    Tracker t;
    t.noteAid(block(2, 1, sdr_aid::kWhyAids), kT);
    EXPECT_EQ(t.state(true, kT + Tracker::kStaleUs - 1), SH_OK);
    EXPECT_EQ(t.state(true, kT + Tracker::kStaleUs), SH_DEGRADED);
}

TEST(SdrImuStatus, TheReportAloneSaysAidingWhileItIsFresh)
{
    // Before its first fix the board sends no epochs, only its mounting report.
    Tracker t;
    t.noteAlign(report(2), kT);                                   // aiding, a whole mounting in use
    EXPECT_EQ(t.state(true, kT), SH_OK);
    EXPECT_EQ(t.state(true, kT + Tracker::kStaleUs), SH_DEGRADED);
    t.noteAlign(report(2, sdr_rtcm::TrAlign::kHaveTilt), kT);    // aiding, its heading not known: the vertical only
    EXPECT_EQ(t.state(true, kT), SH_DEGRADED);
    t.noteAlign(report(2, sdr_rtcm::TrAlign::kHaveNone), kT);    // its own levelling alone
    EXPECT_EQ(t.state(true, kT), SH_DEGRADED);
    t.noteAlign(report(1), kT);   // watching only
    EXPECT_EQ(t.state(true, kT), SH_DEGRADED);
    t.noteAlign(report(0), kT);   // off
    EXPECT_EQ(t.state(true, kT), SH_DEGRADED);
}

TEST(SdrImuStatus, TheEpochsDecideOverTheReport)
{
    Tracker t;
    t.noteAlign(report(2), kT);
    t.noteAid(block(0, 0, sdr_aid::kWhyGap), kT);   // the IMU said aiding, then a gap dropped it
    EXPECT_EQ(t.state(true, kT), SH_BAD);
    t.noteAlign(report(1), kT + 100'000);
    t.noteAid(block(2, 1, sdr_aid::kWhyAids), kT + 100'000);
    EXPECT_EQ(t.state(true, kT + 100'000), SH_OK);
    // The epochs stop (the fix lost), the report goes on: it decides again
    t.noteAlign(report(2), kT + 3'000'000);
    EXPECT_EQ(t.state(true, kT + 3'000'000), SH_OK);
    t.noteAlign(report(1), kT + 3'000'000);
    EXPECT_EQ(t.state(true, kT + 3'000'000), SH_DEGRADED);
}
