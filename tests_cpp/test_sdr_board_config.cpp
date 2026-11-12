// The V10's gnss-sdr-dev image (-DTR_BOARD_V10=1 -DTR_GNSS_SDR=1, which the
// project CMakeLists turns into TR_GNSS_DRIVER_SDR=1): what config.h makes of
// it -- the raw-GNSS filter as the one navigation filter, its own paths
// (TR_RAW_NAV_SDR) beside those it shares with the mosaic-only image
// (TR_RAW_NAV_P4) and none of the mosaic's own, the heading and the descent
// as every raw-filter image flies them, the board's poll rate and no 1PPS --
// and the receiver profile startRawShadow builds from (sdrDevProfile with
// applySdrFlightProfile): 10 Hz fused, 16 satellites, the Doppler means of
// ten epochs taken 45 ms back.
#include <gtest/gtest.h>

#include "config.h"   // flight_computer/main/config.h
#include "ReceiverProfile.h"
#include "ReceiverProfileApply.h"

#if !(defined(TR_GNSS_DRIVER_SDR) && TR_GNSS_DRIVER_SDR) || !TR_BOARD_V10
#error "test_sdr_board_config is built with TR_BOARD_V10=1 TR_GNSS_DRIVER_SDR=1"
#endif

TEST(SdrBoardConfig, OneNavigationFilterTheRawOne)
{
    EXPECT_EQ(TR_RAW_GNSS_SHADOW, 1);
    EXPECT_EQ(TR_RAW_GNSS_PRIMARY, 1);
    EXPECT_EQ(TR_RAW_NAV_SDR, 1);
    EXPECT_EQ(TR_RAW_NAV_P4, 1);
#if defined(TR_RAW_NAV_MOSAIC)
    ADD_FAILURE() << "the mosaic image's own paths are on in the gnss-sdr-dev image";
#endif
#if TR_RAW_MULTIBAND
    ADD_FAILURE() << "the bands combined on a board with one band a system";
#endif
    EXPECT_EQ(TR_HEADING_FIT, 2);           // act, as wherever the raw filter navigates
    EXPECT_EQ(TR_RAW_DESCENT_IMU_FREE, 1);  // the descent on GNSS and the barometer
}

TEST(SdrBoardConfig, TheBoardOnJ1)
{
    EXPECT_EQ(config::GNSS_RX, 3);          // J1 pin 2, the board's TX
    EXPECT_EQ(config::GNSS_TX, 4);
    EXPECT_EQ(config::GNSS_UPDATE_RATE, 20);   // its driver read every 25 ms
    EXPECT_EQ(config::GNSS_PPS_PIN, -1);       // the V1 board has no 1PPS on pin 5
}

TEST(SdrBoardConfig, TheOneFiltersSettingsAreTheMosaicImages)
{
    EXPECT_EQ(config::RAW_RESTORE_POS_SIGMA_M, 100.0);
    EXPECT_EQ(config::RAW_RESTORE_VEL_SIGMA_MPS, 20.0);
    EXPECT_EQ(config::RAW_RESTORE_ATT_SIGMA_DEG, 15.0);
    EXPECT_EQ(config::RAW_HEADING_GATE, 1);
    EXPECT_EQ(config::RAW_INTERNAL_FLOOR_B, 56u * 1024u);
    EXPECT_EQ(config::RAW_MEAS_STACK_B, 6144u);
    EXPECT_FALSE(config::RAW_VOTE_GUARD_ACTS);
}

TEST(SdrBoardConfig, TheProfileItFlies)
{
    gnss_raw::GnssReceiverProfile rx = gnss_raw::sdrDevProfile();
    gnss_raw::applySdrFlightProfile(rx);
    EXPECT_EQ(rx.type, gnss_raw::kRxSdrDev);
    EXPECT_EQ(rx.fuse_epoch_period_s, 0.1);
    EXPECT_EQ(rx.raw_epoch_period_s, 0.01);
    EXPECT_EQ(rx.sat_cap, 16);
    EXPECT_EQ(rx.sat_cap_max, 0);
    EXPECT_TRUE(rx.build_lite);
    EXPECT_TRUE(rx.share_los);
    EXPECT_TRUE(rx.rr_float);
    EXPECT_TRUE(rx.rr_pair_avg);
    EXPECT_EQ(rx.rr_pair_max_diff_mps, 5.0);
    EXPECT_EQ(gnss_raw::sdrDopplerMeanEpochs(rx), 10);
    EXPECT_NEAR(gnss_raw::sdrDopplerMeanLag(rx), 0.045, 1e-12);
}
