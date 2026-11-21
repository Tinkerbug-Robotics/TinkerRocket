// Host tests for which navigation filter the one V9/V10 image flies at boot
// (flight_computer/main/raw_nav_auto.h rawNavChoice, pure): the raw-GNSS
// filter on the receiver found when it gives measurements (the mosaic-G5,
// the PX1105R, the gnss-sdr-dev board), else on the one remembered from last
// boot when that one does (a board before its first fix, a crash boot that
// left the receiver out), else none -- GpsInsEKF, as the auto image. And that
// outside that image RAW_NAV_WHEN and RAW_NAV_AND leave the code as it was.
#include <gtest/gtest.h>

#include "raw_nav_auto.h"

namespace
{
const uint8_t kRaw[] = {GNSS_TYPE_MOSAIC, GNSS_TYPE_SDR, GNSS_TYPE_PX1105R};
const uint8_t kAll[] = {GNSS_TYPE_NONE, GNSS_TYPE_UBLOX, GNSS_TYPE_MOSAIC, GNSS_TYPE_LC86G,
                        GNSS_TYPE_SDR, GNSS_TYPE_PX1105R, 0xFF};
}  // namespace

TEST(RawNavChoice, OnlyTheThreeRawReceiversGiveMeasurements)
{
    for (uint8_t t : kAll)
    {
        const bool raw = t == GNSS_TYPE_MOSAIC || t == GNSS_TYPE_SDR || t == GNSS_TYPE_PX1105R;
        EXPECT_EQ(gnssTypeGivesRaw(t), raw) << int(t);
    }
}

TEST(RawNavChoice, TheReceiverFoundDecidesWhenItGivesMeasurements)
{
    for (uint8_t found : kRaw)
        for (uint8_t remembered : kAll)
            EXPECT_EQ(rawNavChoice(found, remembered), found) << int(found) << " " << int(remembered);
}

TEST(RawNavChoice, AUbloxFoundFliesGpsInsEkfWhateverWasRemembered)
{
    for (uint8_t remembered : kAll)
        EXPECT_EQ(rawNavChoice(GNSS_TYPE_UBLOX, remembered), GNSS_TYPE_NONE) << int(remembered);
}

TEST(RawNavChoice, NothingFoundFliesTheRememberedRawReceiversFilter)
{
    // the board before its first fix, a PX1105R or mosaic still booting, a
    // crash boot that left the receiver out: their own images' filter
    for (uint8_t remembered : kRaw)
        EXPECT_EQ(rawNavChoice(GNSS_TYPE_NONE, remembered), remembered) << int(remembered);
    // a u-blox or nothing remembered: the auto image's GNSS-absent GpsInsEKF
    EXPECT_EQ(rawNavChoice(GNSS_TYPE_NONE, GNSS_TYPE_UBLOX), GNSS_TYPE_NONE);
    EXPECT_EQ(rawNavChoice(GNSS_TYPE_NONE, GNSS_TYPE_NONE), GNSS_TYPE_NONE);
    EXPECT_EQ(rawNavChoice(GNSS_TYPE_NONE, 0xFF), GNSS_TYPE_NONE);
}

TEST(RawNavChoice, OutsideTheOneImageTheGatesAreNothing)
{
    // this test is built without TR_RAW_NAV_AUTO, as every image but the one
    EXPECT_FALSE(kRawNavAuto);
    int ran = 0;
    RAW_NAV_WHEN(false)
    ran = 1;
    EXPECT_EQ(ran, 1);
    const bool b = RAW_NAV_AND(false) true;
    EXPECT_TRUE(b);
}
