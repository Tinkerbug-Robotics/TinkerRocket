#include <gtest/gtest.h>

#include "config.h"

// Built once per board revision (TR_BOARD_V7/V8/V9), exactly as the
// firmware selects its header — so the claim has to survive being stated
// three times.
//
// #837 item 6 — the GNSS "reset and re-verify" path claimed a reset it never
// performed.
//
// After a successful OTP burn, TR_GNSSReceiverUBlox_Serial::begin() logs
// "Resetting receiver to apply OTP high-clock config", calls pulseReset() and
// recurses. pulseReset() opened with a silent `if (reset_n_pin < 0) return;`
// — and EVERY flight-computer board revision declares GNSS_RESET_N = -1. So
// the receiver never restarted, the recursive begin() re-read the same
// pre-restart OTP state, and the second pass gave up with "still not verified
// after OTP write + reset".
//
// The consequence is a fresh SAM-M10Q flying its first mission at the default
// clock, with nothing in the log explaining why.
//
// This used to say "roughly half the intended GNSS rate". Measured 2026-08-27
// and withdrawn: the four flights in examples/flights/ all predate OTP
// programming (#426, 2026-07-07) and so ran on default-clock modules, yet
// every one delivered 18.18 Hz of distinct GNSS epochs while tracking 16-29
// satellites. There is no known rate loss. What is real is that a config the
// firmware reports as applied is not applied, and that the log claimed a
// reset that never happened.
//
// The fix routes that path through UBX-CFG-RST instead. This file pins the
// premise: there is no reset line to drive. The carrier confirms why — the
// SAM-M10Q's ~RESET (U1 pin 18) goes only to R2 (1k) up to +3V3 and is not
// brought out to J3.
//
// IF ONE OF THESE FAILS, a board has started wiring RESET_N. That is a fine
// thing to do — but go and re-check the OTP path in
// TR_GNSSReceiverUBlox_Serial.cpp before changing the number here: the
// pulseReset() branch becomes live again on that board, and its timing (2 ms
// high / 20 ms low / 250 ms settle) has never run against real hardware.

TEST(FcGnssResetMap, ThisBoardDoesNotWireResetN)
{
    EXPECT_EQ(-1, config::GNSS_RESET_N);
}

TEST(FcGnssResetMap, ThisBoardDoesNotWireSafebootN)
{
    // Same story, same connector: SAFEBOOT_N is not brought out either, so
    // the SAFEBOOT_N-high step in begin() is equally inert. It is harmless
    // (it only ever drives a pin HIGH), but it must not be mistaken for a
    // working recovery path either.
    EXPECT_EQ(-1, config::GNSS_SAFEBOOT_N);
}
