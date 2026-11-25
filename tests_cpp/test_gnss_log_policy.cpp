// The GNSS log setting (BLE cmd 79): DETAILED logs every SBF block a
// mosaic-G5 sends and holds the IMU at 3840 Hz; REDUCED logs PVTGeodetic at
// 20 Hz, the status and navigation blocks, and the raw measurements once a
// second, and gives the IMU the board's own limit (8k Dynamic on the V9).
//
// Pinned here: the setting's values, the IMU limit each gives
// (RocketComputerTypes.h imuRateMaxForGnss), and the cut itself
// (gnss_log_policy.h), on made-up streams and on a recorded epoch
// (fixtures/gnss_detect_sbf.h). What each costs on the link and the OC's
// flash is test_i2s_link_budget's.

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "RocketComputerTypes.h"
#include "gnss_log_policy.h"
#include "fixtures/gnss_detect_sbf.h"

namespace {

constexpr uint16_t kPvtGeodetic = 4007;
constexpr uint16_t kReceiverStatus = 4014;
constexpr uint16_t kGpsNav = 5891;

// An SBF block as the driver hands it on: "$@", CRC (not checked here),
// ID (number, revision 1), Length, TOW, WNc, then a body of zeros.
std::vector<uint8_t> block(uint16_t number, uint32_t tow_ms, size_t len = 96)
{
    std::vector<uint8_t> b(len, 0);
    b[0] = '$';
    b[1] = '@';
    const uint16_t id = (uint16_t)(number | (1u << 13));
    b[4] = (uint8_t)(id & 0xFF);
    b[5] = (uint8_t)(id >> 8);
    b[6] = (uint8_t)(len & 0xFF);
    b[7] = (uint8_t)(len >> 8);
    for (int i = 0; i < 4; i++) b[8 + i] = (uint8_t)(tow_ms >> (8 * i));
    b[12] = 0x88;   // WNc 2440
    b[13] = 0x09;
    return b;
}

bool keep(gnss_log::SbfCut& cut, const std::vector<uint8_t>& b, bool reduced = true)
{
    return gnss_log::keep(cut, b.data(), b.size(), reduced);
}

// The receiver's stream, epoch by epoch from `tow0_ms`: MeasEpoch, MeasExtra
// and PVTGeodetic every `step_ms`, the status on each whole second. Counts
// what the cut keeps of each.
struct Kept { int meas_epoch = 0, meas_extra = 0, pvt = 0, status = 0; std::vector<uint32_t> meas_tows; };

Kept run(uint32_t tow0_ms, int epochs, uint32_t step_ms, bool reduced)
{
    gnss_log::SbfCut cut;
    Kept k;
    for (int e = 0; e < epochs; e++)
    {
        const uint32_t tow = tow0_ms + (uint32_t)e * step_ms;
        if (keep(cut, block(gnss_log::kSbfMeasEpoch, tow, 1496), reduced))
        {
            k.meas_epoch++;
            k.meas_tows.push_back(tow);
        }
        if (keep(cut, block(gnss_log::kSbfMeasExtra, tow, 1632), reduced)) k.meas_extra++;
        if (keep(cut, block(kPvtGeodetic, tow), reduced)) k.pvt++;
        if (tow % 1000 == 0 && keep(cut, block(kReceiverStatus, tow, 68), reduced)) k.status++;
    }
    return k;
}

}  // namespace

// ── The setting ─────────────────────────────────────────────────────────

TEST(GnssLogSetting, TwoValuesAndDetailedIsTheDefault)
{
    EXPECT_TRUE(gnssLogSettingValid(GNSS_LOG_DETAILED));
    EXPECT_TRUE(gnssLogSettingValid(GNSS_LOG_REDUCED));
    EXPECT_FALSE(gnssLogSettingValid(2));
    EXPECT_FALSE(gnssLogSettingValid(0xFF));
    EXPECT_EQ(GNSS_LOG_DETAILED, 0);   // a fresh NVS and an all-zero frame read as today's log
    EXPECT_STREQ(gnssLogName(GNSS_LOG_DETAILED), "detailed");
    EXPECT_STREQ(gnssLogName(GNSS_LOG_REDUCED), "reduced");
    EXPECT_EQ(sizeof(GnssLogConfigData), 1u);
}

// ── The IMU limit each gives ────────────────────────────────────────────

TEST(GnssLogImuLimit, TheDetailedLogHoldsTheMosaicAt3840)
{
    EXPECT_EQ(imuRateMaxForGnss(7680, GNSS_TYPE_MOSAIC, GNSS_LOG_DETAILED), 3840);
    // The two-argument form is the detailed log, the rule before the setting.
    EXPECT_EQ(imuRateMaxForGnss(7680, GNSS_TYPE_MOSAIC), 3840);
}

TEST(GnssLogImuLimit, TheReducedLogGivesTheBoardItsOwnLimit)
{
    EXPECT_EQ(imuRateMaxForGnss(7680, GNSS_TYPE_MOSAIC, GNSS_LOG_REDUCED), 7680);
    EXPECT_EQ(imuRateMaxForGnss(3840, GNSS_TYPE_MOSAIC, GNSS_LOG_REDUCED), 3840);
}

TEST(GnssLogImuLimit, OtherReceiversIgnoreTheSetting)
{
    for (uint8_t log : {GNSS_LOG_DETAILED, GNSS_LOG_REDUCED})
    {
        EXPECT_EQ(imuRateMaxForGnss(7680, GNSS_TYPE_UBLOX, log), 7680) << (int)log;
        EXPECT_EQ(imuRateMaxForGnss(7680, GNSS_TYPE_LC86G, log), 7680) << (int)log;
        EXPECT_EQ(imuRateMaxForGnss(7680, GNSS_TYPE_NONE, log), 7680) << (int)log;
        EXPECT_EQ(imuRateMaxForGnss(3840, GNSS_TYPE_UBLOX, log), 3840) << (int)log;
    }
}

TEST(GnssLogImuLimit, ASettingThatIsNoneKeepsTheCap)
{
    // Only REDUCED lifts it: whatever else arrives gets the limit that is
    // safe with every block in the log.
    EXPECT_EQ(imuRateMaxForGnss(7680, GNSS_TYPE_MOSAIC, 2), 3840);
    EXPECT_EQ(imuRateMaxForGnss(7680, GNSS_TYPE_MOSAIC, 0xFF), 3840);
}

TEST(GnssLogImuLimit, An8kSettingKeptUnderTheCapFliesAgainWithTheReducedLog)
{
    // What the FC and the OC do with an 8k setting as the log setting moves:
    // the setting is kept, the clamp of it flies.
    for (uint16_t s : {IMU_RATE_DYNAMIC_8K, (uint16_t)7680})
    {
        const uint16_t detailed = imuRateSettingClamp(s, imuRateMaxForGnss(7680, GNSS_TYPE_MOSAIC, GNSS_LOG_DETAILED));
        const uint16_t reduced  = imuRateSettingClamp(s, imuRateMaxForGnss(7680, GNSS_TYPE_MOSAIC, GNSS_LOG_REDUCED));
        EXPECT_EQ(reduced, s);
        EXPECT_NE(detailed, s);
        EXPECT_EQ(imuRatePeakHz(detailed), 3840);
    }
    // 4k Dynamic flies as itself under either.
    EXPECT_EQ(imuRateSettingClamp(IMU_RATE_DYNAMIC, imuRateMaxForGnss(7680, GNSS_TYPE_MOSAIC, GNSS_LOG_DETAILED)),
              IMU_RATE_DYNAMIC);
}

// ── The cut ─────────────────────────────────────────────────────────────

TEST(GnssSbfCut, TheDetailedLogKeepsEveryBlock)
{
    const Kept k = run(427415000u, 60, 50, /*reduced=*/false);
    EXPECT_EQ(k.meas_epoch, 60);
    EXPECT_EQ(k.meas_extra, 60);
    EXPECT_EQ(k.pvt, 60);
    EXPECT_EQ(k.status, 3);
}

TEST(GnssSbfCut, TheReducedLogKeepsTheWholeSecondsEpochAndEverythingElse)
{
    // Three seconds of the 20 Hz stream from a whole second.
    const Kept k = run(427415000u, 60, 50, /*reduced=*/true);
    EXPECT_EQ(k.pvt, 60);       // the fix at 20 Hz
    EXPECT_EQ(k.status, 3);
    EXPECT_EQ(k.meas_epoch, 3);
    EXPECT_EQ(k.meas_extra, 3);   // the same epochs: both blocks of each
    ASSERT_EQ(k.meas_tows.size(), 3u);
    for (uint32_t tow : k.meas_tows) EXPECT_EQ(tow % 1000, 0u) << tow;
}

TEST(GnssSbfCut, StartingMidSecondTakesItsFirstEpochThenTheWholeSeconds)
{
    // The setting can change at any moment on the pad: the first epoch seen
    // in a second is that second's.
    const Kept k = run(427415350u, 33, 50, true);   // 427415.35 .. 427416.95 s
    ASSERT_EQ(k.meas_tows.size(), 2u);
    EXPECT_EQ(k.meas_tows[0], 427415350u);
    EXPECT_EQ(k.meas_tows[1], 427416000u);
    EXPECT_EQ(k.meas_extra, 2);
}

TEST(GnssSbfCut, AGridThatMissesTheWholeSecondStillGivesOneASecond)
{
    // Epochs 25 ms off the second, as at a rate or clock the FC does not set.
    const Kept k = run(427415025u, 100, 50, true);   // five seconds
    EXPECT_EQ(k.meas_epoch, 5);
    EXPECT_EQ(k.meas_extra, 5);
    for (uint32_t tow : k.meas_tows) EXPECT_EQ(tow % 1000, 25u) << tow;
    // And at 10 Hz from the whole second, one a second still.
    EXPECT_EQ(run(427415000u, 50, 100, true).meas_epoch, 5);
}

TEST(GnssSbfCut, TheMeasExtraKeptIsTheMeasEpochsOwn)
{
    // The MeasEpoch of the whole second lost on the line (a CRC failure
    // drops it before the cut): its MeasExtra still opens the second, and
    // the next epoch's MeasEpoch is not taken in its place, so no block
    // logged stands for another epoch than the one it was taken in.
    gnss_log::SbfCut cut;
    EXPECT_TRUE(keep(cut, block(gnss_log::kSbfMeasExtra, 427415000u)));
    EXPECT_FALSE(keep(cut, block(gnss_log::kSbfMeasEpoch, 427415050u)));
    EXPECT_FALSE(keep(cut, block(gnss_log::kSbfMeasExtra, 427415050u)));
    EXPECT_TRUE(keep(cut, block(gnss_log::kSbfMeasEpoch, 427416000u)));
    EXPECT_TRUE(keep(cut, block(gnss_log::kSbfMeasExtra, 427416000u)));
}

TEST(GnssSbfCut, AMeasurementWithNoTimeYetIsLeftOut)
{
    gnss_log::SbfCut cut;
    EXPECT_FALSE(keep(cut, block(gnss_log::kSbfMeasEpoch, gnss_log::kTowDnu)));
    EXPECT_FALSE(keep(cut, block(gnss_log::kSbfMeasExtra, gnss_log::kTowDnu)));
    // Every other block is kept with or without time.
    EXPECT_TRUE(keep(cut, block(kPvtGeodetic, gnss_log::kTowDnu)));
    EXPECT_TRUE(keep(cut, block(kReceiverStatus, gnss_log::kTowDnu)));
    // The first epoch with a time is then kept.
    EXPECT_TRUE(keep(cut, block(gnss_log::kSbfMeasEpoch, 427415100u)));
    // Without the reduced log a block with no time goes in as before.
    EXPECT_TRUE(keep(cut, block(gnss_log::kSbfMeasEpoch, gnss_log::kTowDnu), false));
}

TEST(GnssSbfCut, TheWeekRolloverOpensANewSecond)
{
    gnss_log::SbfCut cut;
    EXPECT_TRUE(keep(cut, block(gnss_log::kSbfMeasEpoch, 604799000u)));
    EXPECT_FALSE(keep(cut, block(gnss_log::kSbfMeasEpoch, 604799950u)));
    EXPECT_TRUE(keep(cut, block(gnss_log::kSbfMeasEpoch, 0u)));
    EXPECT_TRUE(keep(cut, block(gnss_log::kSbfMeasExtra, 0u)));
}

TEST(GnssSbfCut, NavigationDataAndAnythingElsePass)
{
    gnss_log::SbfCut cut;
    for (int i = 0; i < 5; i++) EXPECT_TRUE(keep(cut, block(kGpsNav, 427415000u + 50u * i)));
    // The LC86G's RTCM3 rides the same record on the Beetle.
    const uint8_t rtcm[] = {0xD3, 0x00, 0x13, 0x3E, 0xD0, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    EXPECT_TRUE(gnss_log::keep(cut, rtcm, sizeof(rtcm), true));
    // Too short to be a block, or nothing at all: not the cut's to drop.
    const uint8_t stub[] = {'$', '@', 0, 0, 0xBB, 0x0F};
    EXPECT_TRUE(gnss_log::keep(cut, stub, sizeof(stub), true));
    EXPECT_TRUE(gnss_log::keep(cut, nullptr, 0, true));
}

TEST(GnssSbfCut, TheRecordedEpoch)
{
    // The mosaic-G5's own blocks (2026-10-15 spin-rig capture), split by
    // their Length fields: the epoch on a whole second, then the same three
    // blocks 50 ms later.
    std::vector<std::vector<uint8_t>> blocks;
    for (size_t off = 0; off + 8 <= sizeof(kMosaicEpoch);)
    {
        const size_t len = (size_t)kMosaicEpoch[off + 6] | ((size_t)kMosaicEpoch[off + 7] << 8);
        ASSERT_GE(len, 14u);
        ASSERT_LE(off + len, sizeof(kMosaicEpoch));
        blocks.emplace_back(kMosaicEpoch + off, kMosaicEpoch + off + len);
        off += len;
    }
    ASSERT_EQ(blocks.size(), 3u);   // MeasEpoch, MeasExtra, PVTGeodetic

    gnss_log::SbfCut cut;
    for (const auto& b : blocks) EXPECT_TRUE(keep(cut, b));
    EXPECT_EQ(cut.kept_tow, 427415000u);

    for (auto b : blocks)
    {
        const uint32_t tow = 427415050u;
        for (int i = 0; i < 4; i++) b[8 + i] = (uint8_t)(tow >> (8 * i));
        const uint16_t number = (uint16_t)((b[4] | (b[5] << 8)) & 0x1FFF);
        EXPECT_EQ(keep(cut, b), number == kPvtGeodetic) << number;
    }

    // The 1 Hz blocks of that second all go in.
    for (size_t off = 0; off + 8 <= sizeof(kMosaicSecond);)
    {
        const size_t len = (size_t)kMosaicSecond[off + 6] | ((size_t)kMosaicSecond[off + 7] << 8);
        ASSERT_GE(len, 14u);
        ASSERT_LE(off + len, sizeof(kMosaicSecond));
        EXPECT_TRUE(gnss_log::keep(cut, kMosaicSecond + off, len, true)) << off;
        off += len;
    }
}

TEST(GnssSbfCut, TheReducedLogKeepsAbout8PercentOfTheBytes)
{
    // Ten seconds at the sizes a 33-satellite sky gave (MeasEpoch 1,495 B,
    // MeasExtra 1,631 B, PVTGeodetic 96 B): the bytes the cut keeps, against
    // all of them. The link and flash budgets build on this ratio.
    gnss_log::SbfCut cut;
    size_t all = 0, kept = 0;
    for (int e = 0; e < 200; e++)
    {
        const uint32_t tow = 427415000u + 50u * (uint32_t)e;
        for (const auto& b : {block(gnss_log::kSbfMeasEpoch, tow, 1495), block(gnss_log::kSbfMeasExtra, tow, 1631),
                              block(kPvtGeodetic, tow, 96)})
        {
            all += b.size();
            if (keep(cut, b)) kept += b.size();
        }
    }
    EXPECT_EQ(all, 200u * (1495 + 1631 + 96));
    EXPECT_EQ(kept, 10u * (1495 + 1631) + 200u * 96);
    EXPECT_LT((double)kept / (double)all, 0.085);
}
