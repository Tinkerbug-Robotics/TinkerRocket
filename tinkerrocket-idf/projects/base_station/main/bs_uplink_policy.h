#pragma once

// Base-station uplink transmit-gate policy (#379).
//
// Pure predicate extracted from main.cpp's serviceUplink so the host-side gtest
// can drive it without the LoRa radio / BLE stack.  Must stay free of ESP-IDF /
// FreeRTOS / hardware dependencies.

#include <cstdint>

namespace bs_uplink_policy {

// Whether serviceUplink may START an uplink transmit this pass.
//
// The scan gate (scan_passes_remaining == 0) is the #379 fix. A frequency scan
// owns the radio's tuning for its whole 5-pass run; an uplink TX started during
// it goes out on whatever channel the scan is currently dwelling on — so the
// command silently never reaches the rocket (there is no ACK to catch it) and
// the transmission corrupts that pass's RSSI samples. serviceHeartbeat already
// gates on the same counter (main.cpp: `if (scan_passes_remaining_ != 0)
// return;`); user uplink commands must too.
//
// scan_passes_remaining_ is used rather than the LoRa driver's per-pass
// scan_state_ because it stays non-zero across the ENTIRE scan, including the
// brief inter-pass gaps where the next pass is re-armed — exactly the windows a
// per-pass guard would miss.
//
// Because serviceUplink returns before the send() that consumes a retry, gating
// here defers the command (retries intact) until the scan finishes, rather than
// dropping it.
static inline bool mayTransmitUplink(bool uplink_pending,
                                     uint8_t retries_left,
                                     uint8_t scan_passes_remaining,
                                     bool radio_can_send)
{
    if (!uplink_pending || retries_left == 0) return false;
    if (scan_passes_remaining != 0)           return false;  // #379: defer through the scan
    return radio_can_send;
}

}  // namespace bs_uplink_policy
