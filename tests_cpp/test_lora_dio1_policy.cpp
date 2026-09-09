// #1155 item 7: a DIO1 edge during a scan dwell is not a transmit completion.
#include <gtest/gtest.h>
#include <LoRaDio1Policy.h>

using lora_dio1::Owner;
using lora_dio1::classify;

TEST(LoRaDio1Policy, TransmitInFlightOwnsTheEdge)
{
    EXPECT_EQ(classify(/*tx_ongoing=*/true, /*rx_mode=*/false), Owner::Tx);
    // rx_mode_ can read true at the polling instant of a retune (#105 TX
    // watchdog case); a transmit in flight still wins.
    EXPECT_EQ(classify(true, true), Owner::Tx);
}

TEST(LoRaDio1Policy, ArmedReceiverOwnsTheEdge)
{
    EXPECT_EQ(classify(false, true), Owner::Rx);
}

TEST(LoRaDio1Policy, ScanDwellOrRetuneOwnsNothing)
{
    // startScan()/hopToFrequencyMHz() clear rx_mode_ with no transmit in
    // flight: this is the case that used to be booked as tx_done_.
    EXPECT_EQ(classify(false, false), Owner::None);
}
