#include <gtest/gtest.h>

#include "config.h"   // flight_computer/main/config.h

// Built once per board that takes the mosaic-only image (-DTR_GNSS_MOSAIC=1):
// the V9, the Goblin's board for 2026-10-24, and the V8, which stays
// supported (tests_cpp/v9raw/build.cmake). Both carry the same J1, and the harness lands
// the receiver the same way on each, so a pin that moves on one board's
// header silently costs that board its receiver or its 1PPS. config.h itself
// refuses the mosaic driver on any other board.

#if !(defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC)
#error "test_mosaic_board_config is built with TR_GNSS_DRIVER_MOSAIC=1"
#endif

TEST(MosaicBoardConfig, ReceiverUartIsJ1Pins1And2)
{
    // The FC's own rx/tx, passed straight to uartBegin(baud, rx, tx): J1.2 is
    // the receiver's TXD on GPIO3, J1.1 its RXD on GPIO4.
    EXPECT_EQ(config::GNSS_RX, 3);
    EXPECT_EQ(config::GNSS_TX, 4);
}

TEST(MosaicBoardConfig, PpsIsJ1Pin5OnGpio2)
{
    // The harness brings the receiver's 1PPS to J1 pin 5 through 1 kOhm; this
    // build reads it, and only as an input.
    EXPECT_EQ(config::GNSS_J1_PIN5, 2);
    EXPECT_EQ(config::GNSS_PPS_PIN, config::GNSS_J1_PIN5);
}

TEST(MosaicBoardConfig, RateIsTheReceiversTwentyHertz)
{
    // Its permission file allows 20 Hz, position and measurements alike.
    EXPECT_EQ(config::GNSS_UPDATE_RATE, 20);
}
