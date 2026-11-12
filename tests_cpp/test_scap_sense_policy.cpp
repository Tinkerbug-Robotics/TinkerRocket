// #1409: the Tinker-Mantis V10's hold-up supercap sense, as the flight
// computer reads it. R125/R126 halve V_SCAP onto P4 GPIO17; the FC averages
// eight calibrated samples of the pad once a second and sends V_SCAP in
// FcStatusData.scap_mv, where the out computer runs the mini's verdict on it
// (test_holdup_policy, HoldupPolicy1409). Built with the V10's flags so the
// cases run against the shipped pin, divider and period (config.h).
#include <gtest/gtest.h>

#include "config.h"              // flight_computer/main/config.h, TR_BOARD_V10
#include "scap_sense_policy.h"

#if !TR_BOARD_V10
#error "test_scap_sense_policy is built with TR_BOARD_V10=1"
#endif

using scap_sense::fieldFromPinSamples;

TEST(ScapSense1409, TheV10ReadsGpio17BehindATwoToOneDivider)
{
    EXPECT_EQ(config::SCAP_ADC_PIN, 17);
    EXPECT_FLOAT_EQ(config::SCAP_DIVIDER_RATIO, 2.0f);
    // The out computer judges once a second and calls a report stale after
    // FC_FRAME_STALE_MS (3 s); a reading at least once a second keeps every
    // judgement on a value no older than the one before it.
    EXPECT_LE(config::SCAP_SAMPLE_PERIOD_MS, 1000u);
    EXPECT_GE(config::SCAP_SAMPLE_PERIOD_MS, 100u);   // and no busier than it needs to be
}

TEST(ScapSense1409, AveragedPadMillivoltsTimesTheDivider)
{
    // A terminated cap: 2.5 V, 1250 mV at the pad.
    EXPECT_EQ(fieldFromPinSamples(8 * 1250, 8, config::SCAP_DIVIDER_RATIO), 2500u);
    // The 2.2 V bar.
    EXPECT_EQ(fieldFromPinSamples(8 * 1100, 8, config::SCAP_DIVIDER_RATIO), 2200u);
    // Averaged, then rounded: 1249, 1250 x 6, 1252 -> 1250.125 mV -> 2500.25 -> 2500.
    EXPECT_EQ(fieldFromPinSamples(1249 + 6 * 1250 + 1252, 8, config::SCAP_DIVIDER_RATIO), 2500u);
    // Fewer samples than asked for (some reads failed) average what was taken.
    EXPECT_EQ(fieldFromPinSamples(3 * 1240, 3, config::SCAP_DIVIDER_RATIO), 2480u);
    // An empty cap is a reading of zero, not no reading.
    EXPECT_EQ(fieldFromPinSamples(0, 8, config::SCAP_DIVIDER_RATIO), 0u);
}

TEST(ScapSense1409, NoSampleIsNoReading)
{
    EXPECT_EQ(fieldFromPinSamples(0, 0, config::SCAP_DIVIDER_RATIO), FCS_SCAP_NO_READING);
    EXPECT_EQ(fieldFromPinSamples(1234, -1, config::SCAP_DIVIDER_RATIO), FCS_SCAP_NO_READING);
}

TEST(ScapSense1409, TheFullDb12RangeStaysUnderTheSentinel)
{
    // The pad can read up to the 3.3 V rail at 12 dB: 6.6 V of V_SCAP, a
    // fault, but still a number and never "no reading".
    const uint16_t f = fieldFromPinSamples(8 * 3300, 8, config::SCAP_DIVIDER_RATIO);
    EXPECT_EQ(f, 6600u);
    EXPECT_NE(f, FCS_SCAP_NO_READING);
}
