// #1143 item 1: the IDENTITY band edges are reported by the modem and were
// enforced by neither end.
//
// The E220-900MM22S is matched for 850-930 MHz and says so in every IDENTITY
// frame; README.md states the contract ("Hosts clamp their radio config to the
// reported capabilities") and radio_board/config.h calls reporting them "the
// whole point of IDENTITY". A repo-wide grep before this change found the two
// fields written once and read only into an ESP_LOGI. Meanwhile the bare
// LLCC68 die accepts 150-960 MHz, so an uplinked frequency was applied, acked
// APPLIED, echoed in STATUS, agreed with by modem_config_ack::accepted(), and
// written to NVS — leaving a rocket transmitting into an unmatched network and
// still doing so after a reboot.

#include <gtest/gtest.h>

#include <cmath>

#include "RocketComputerTypes.h"

TEST(LoraBand, TheOperatingFrequenciesAreInBand) {
    EXPECT_TRUE(loraFreqInBand(LORA_FACTORY_RENDEZVOUS_MHZ));   // 915
    EXPECT_TRUE(loraFreqInBand(902.0f));
    EXPECT_TRUE(loraFreqInBand(928.0f));
}

TEST(LoraBand, TheEdgesThemselvesAreLegal) {
    // Inclusive: these are the module's own stated limits, not a margin.
    EXPECT_TRUE(loraFreqInBand(LORA_BAND_MIN_MHZ));
    EXPECT_TRUE(loraFreqInBand(LORA_BAND_MAX_MHZ));
    EXPECT_FALSE(loraFreqInBand(std::nextafter(LORA_BAND_MIN_MHZ, 0.0f)));
    EXPECT_FALSE(loraFreqInBand(std::nextafter(LORA_BAND_MAX_MHZ, 2000.0f)));
}

TEST(LoraBand, TheRangeTheBareDieWouldHaveAcceptedIsRefused) {
    // SX126x::setFrequency accepts 150-960 MHz. Everything in that range but
    // outside the matching network used to be applied and persisted.
    for (float f : {150.0f, 433.0f, 849.9f, 930.1f, 960.0f}) {
        EXPECT_FALSE(loraFreqInBand(f)) << f << " MHz";
    }
}

TEST(LoraBand, NanIsOutOfBandRatherThanPassedThrough) {
    // A NaN fails both comparisons, which is the answer we want — but state it,
    // because "not less than the minimum" is how a NaN sneaks through a check
    // written the other way round.
    EXPECT_FALSE(loraFreqInBand(std::nanf("")));
    EXPECT_FALSE(loraFreqInBand(INFINITY));
    EXPECT_FALSE(loraFreqInBand(-INFINITY));
}

TEST(LoraBand, ZeroAndNegativeAreRefused) {
    // cmd 10 memcpy's a float straight off the air; a corrupt frame is as
    // likely to produce 0 or a negative as anything else.
    EXPECT_FALSE(loraFreqInBand(0.0f));
    EXPECT_FALSE(loraFreqInBand(-915.0f));
}
