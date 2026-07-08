#include <gtest/gtest.h>

#include "bs_uplink_policy.h"

using bs_uplink_policy::mayTransmitUplink;

// The #379 fix: an uplink command must NOT be transmitted while a LoRa
// frequency scan is running (scan_passes_remaining != 0), because the TX would
// go out on the scan's dwell channel — silently missing the rocket and
// corrupting the pass's RSSI. Gating here (before the send that consumes a
// retry) also defers the command until the scan finishes rather than dropping
// it.

namespace {
constexpr uint8_t RETRIES = 8;  // config::UPLINK_RETRIES
}

// ---- No command pending ----

TEST(BsUplinkPolicy, NoTransmitWhenNothingPending) {
    EXPECT_FALSE(mayTransmitUplink(/*pending=*/false, RETRIES, /*scan=*/0,
                                   /*can_send=*/true));
}

TEST(BsUplinkPolicy, NoTransmitWhenRetriesExhausted) {
    EXPECT_FALSE(mayTransmitUplink(/*pending=*/true, /*retries=*/0, /*scan=*/0,
                                   /*can_send=*/true));
}

// ---- The scan gate (#379) ----

TEST(BsUplinkPolicy, NoTransmitDuringScanEvenWhenRadioReady) {
    // The regression: a pending command with retries and a ready radio must
    // still be held while any scan pass remains.
    EXPECT_FALSE(mayTransmitUplink(/*pending=*/true, RETRIES,
                                   /*scan_passes_remaining=*/1,
                                   /*can_send=*/true));
    EXPECT_FALSE(mayTransmitUplink(/*pending=*/true, RETRIES,
                                   /*scan_passes_remaining=*/5,
                                   /*can_send=*/true));
}

TEST(BsUplinkPolicy, RetriesPreservedImpliesDeferNotDrop) {
    // While scanning we refuse; the caller returns before decrementing retries,
    // so the SAME slot (still pending, retries intact) transmits once the scan
    // completes (scan_passes_remaining hits 0). Model that transition here.
    const uint8_t retries = 4;  // some retries already used, some left
    EXPECT_FALSE(mayTransmitUplink(true, retries, /*scan=*/2, /*can_send=*/true));
    EXPECT_TRUE(mayTransmitUplink(true, retries, /*scan=*/0, /*can_send=*/true));
}

// ---- Normal (no scan) behavior unchanged ----

TEST(BsUplinkPolicy, TransmitsWhenPendingReadyAndNotScanning) {
    EXPECT_TRUE(mayTransmitUplink(/*pending=*/true, RETRIES, /*scan=*/0,
                                  /*can_send=*/true));
}

TEST(BsUplinkPolicy, NoTransmitWhenRadioBusy) {
    // canSend() false (previous TX in progress) still blocks, scan or not.
    EXPECT_FALSE(mayTransmitUplink(/*pending=*/true, RETRIES, /*scan=*/0,
                                   /*can_send=*/false));
}
