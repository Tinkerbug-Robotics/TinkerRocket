#pragma once

#include <stdint.h>

// #1116: the FC's own exit from an OTA image session that nobody ends.
//
// An FC-relay OTA flips the FC's I2S link from master TX (telemetry out) to
// slave RX (image in) and sets fc_ota_data_mode. While that flag is set the
// I2S sender idles, the flight loop sleeps 5 ms per pass and the EKF does not
// run — all fine for the ~15 s of a transfer on the pad. The flag is cleared
// by fcRevertToTx(), which before this header was reached ONLY from the
// OTA_FINISH_CMD / OTA_ABORT_CMD handlers, i.e. only when the OC delivered
// one of those over I2C. Every OC path that ends a session (the app's own
// finish or abort, the BLE disconnect, the OC's 10 s stall watchdog, a failed
// flip) stages that command exactly once into a bounded queue that drops on
// overflow, serves it for three polls, and never re-checks that the FC came
// back. An OC reboot mid-transfer stages nothing at all. In each case the FC
// stayed in the degraded state until a battery pull: nothing on I2S, so the OC
// logged nothing and both downlinks froze; the EKF frozen; and a retry BEGIN
// could not help, because begin() refuses a session that is still open and
// the refusal rode the idle I2S sender.
//
// The rule below gives the FC a way out that depends on nobody else. It is
// deliberately the BACKSTOP: the OC's watchdog gives up on a stalled relay at
// OtaRelayPolicy::kRelayStallTimeoutMs (10 s), so whenever the OC is alive and
// its abort arrives, that abort ends the session first and this never fires.
//
// "Progress" is an accepted, in-order image byte (bytesWritten advancing) —
// not I2S activity. The OC clocks the link continuously while flipped, with
// idle fill between chunks, so RX callbacks prove only that the OC is still
// master. That is the second thing this rule needs to know: the FC must not
// seize BCLK as master while the OC is still driving it (both ends push-pull
// on one wire; the FINISH and ABORT handlers wait for the same silence for the
// same reason). So a stalled session is abandoned once the link has ALSO gone
// quiet — the OC has reverted to slave RX, as it does after its own watchdog
// or after a FINISH/ABORT it staged — and only a link still clocked far past
// the stall is seized regardless, which is the same bounded contention the
// ABORT handler already accepts.
//
// Pure: the sampling (bytesWritten, the RX callback counter) and the teardown
// (quiet wait, revert, receiver abort, ABORTED status) stay in main.cpp.

namespace FcOtaSessionPolicy {

// No accepted image byte for this long ends the session. Three times the OC's
// own stall watchdog (OtaRelayPolicy::kRelayStallTimeoutMs): the OC must
// always have had its chance to end the session the normal way first — the
// host test pins that ordering.
static constexpr uint32_t kNoProgressTimeoutMs = 30000;

// The OC has released BCLK once the FC's slave RX has seen no DMA callback
// for this long. A clocked link fires a callback every few ms (the OC
// idle-fills between chunks precisely so that it never stops), so a full
// second of silence is unambiguous.
static constexpr uint32_t kLinkQuietMs = 1000;

// A link still clocked this long after the last accepted byte is abandoned
// anyway. Reaching this means the OC is master, pumping, and none of it is
// landing (every frame failing CRC, or an OC whose loop task is dead while
// its feeder runs on): the OC's watchdog is fed by chunks it forwards, not by
// chunks the FC accepts, so it cannot end this one. Long enough that a slow
// transfer whose frames ARE landing never gets here — progress resets the
// clock on every accepted chunk.
static constexpr uint32_t kNoProgressHardCapMs = 120000;

enum class Verdict : uint8_t {
    Continue,             // session alive, or stalled but the OC still holds the clock
    AbandonLinkQuiet,     // stalled and the OC has released BCLK: revert now
    AbandonLinkClocked,   // stalled past the hard cap with the OC still clocking: seize
};

// `last_progress_ms`: when bytesWritten last advanced (stamped at the flip).
// `last_rx_ms`: when the RX callback counter last advanced (stamped at the flip).
inline Verdict evaluate(bool data_mode, uint32_t now_ms,
                        uint32_t last_progress_ms, uint32_t last_rx_ms)
{
    if (!data_mode) return Verdict::Continue;
    const uint32_t since_progress = (uint32_t)(now_ms - last_progress_ms);
    if (since_progress < kNoProgressTimeoutMs) return Verdict::Continue;
    if ((uint32_t)(now_ms - last_rx_ms) >= kLinkQuietMs) return Verdict::AbandonLinkQuiet;
    if (since_progress >= kNoProgressHardCapMs) return Verdict::AbandonLinkClocked;
    return Verdict::Continue;
}

// --------------------------------------------------------------------------
// The stable-run window fcMaybeMarkOtaValid() already used.
inline constexpr uint32_t kStableRunMs = 10000;

// #1123: when may a freshly OTA'd image cancel its own rollback?
//
// fcMaybeMarkOtaValid() used elapsed time alone: ~10 s of loop_fc ticks and
// the rollback net was gone. Nothing consulted the links. But the flight
// computer has NO update path of its own — every image arrives through the out
// computer (BLE -> I2C OTA_BEGIN_PENDING -> I2S image pump) — so an image whose
// I2S TX config, I2C master poll or frame CRC is broken ticks the loop happily
// for 10 s, marks itself valid, and permanently removes both the telemetry link
// AND the only way to replace it. The board is then only recoverable by opening
// the airframe and flashing over USB.
//
// docs/plans/08-ota-firmware-update.md:331 states the contract this restores —
// "the new image marks itself valid only after publishing its first I2S frame
// back to OC" — and §4 calls the round-trip gate "deliberately strong". The
// OC's own maybeMarkOtaValid() already honours it (called under
// isConnected() / after a successful sendTelemetry).
//
// Both links are required, because either one alone leaves the image
// unreplaceable: I2S carries telemetry to the OC, I2C carries the commands that
// start the next update.
//
// This is "not yet", never "never": the caller re-evaluates every tick, so a
// slow OC bring-up only delays validation. An image that never satisfies it
// stays PENDING_VERIFY and rolls back at the next reset, which is the point.
inline bool mayCancelRollback(uint32_t uptime_ms,
                              bool     out_ready,
                              uint32_t i2s_frames_accepted)
{
    return uptime_ms >= kStableRunMs && out_ready && i2s_frames_accepted > 0;
}

}  // namespace FcOtaSessionPolicy
