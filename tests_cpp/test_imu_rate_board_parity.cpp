// #1485: the FC and the OC on one board must agree on the fastest IMU rate it
// flies, and on the link rate. Built once per board; each side is compiled
// against its own project's config.h (imu_rate_cap_fc.cpp / _oc.cpp).

#include <gtest/gtest.h>
#include <stdint.h>

#include "RocketComputerTypes.h"

uint16_t fcImuRateMaxHz();
uint32_t fcI2sSampleRate();
bool fcIsm6FifoCapture();
uint16_t ocImuRateMaxHz();
uint32_t ocI2sSampleRate();

TEST(ImuRateBoardParity, FcAndOcStateTheSameLimit) {
    // The OC refuses settings above its number and reports it as "irmax";
    // the FC refuses settings above its own. Different numbers means one side
    // accepts what the other refuses, and the app shows a rate nobody flies.
    EXPECT_EQ(fcImuRateMaxHz(), ocImuRateMaxHz());
}

TEST(ImuRateBoardParity, TheLimitIsAnOdrStepEveryDefaultFits) {
    EXPECT_TRUE(imuRateValid(fcImuRateMaxHz()));
    EXPECT_TRUE(imuRateSettingValid(IMU_RATE_DYNAMIC, fcImuRateMaxHz()));
    EXPECT_GE(fcImuRateMaxHz(), IMU_RATE_BASELINE_MAX_HZ);
}

TEST(ImuRateBoardParity, SevenKiloHertzOnlyWithFifoCapture) {
    // One read per data-ready edge cannot keep up with a 130 us period.
    if (fcImuRateMaxHz() > IMU_RATE_BASELINE_MAX_HZ) {
        EXPECT_TRUE(fcIsm6FifoCapture());
    }
}

TEST(ImuRateBoardParity, BothEndsRunTheSharedLinkRate) {
    EXPECT_EQ(fcI2sSampleRate(), I2S_LINK_SAMPLE_RATE_HZ);
    EXPECT_EQ(ocI2sSampleRate(), I2S_LINK_SAMPLE_RATE_HZ);
}
