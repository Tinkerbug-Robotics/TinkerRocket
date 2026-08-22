#pragma once

#include <stdint.h>

// #834 items 6 and 7: keeping the OC's I2S link recoverable across an FC-relay
// OTA.
//
// The relay flips the OC's I2S from slave RX (normal telemetry ingest) to
// master TX (image pump) and back. Both edges had a hole that ends in the same
// state — the OC driving, or holding, no usable slave RX, with no path back:
//
//   * ocFlipToTx() recorded `oc_ota_tx_mode = (e == ESP_OK)` after end()ing the
//     channel. If beginMasterTx failed there was NO channel at all, and the only
//     restore path (ocRevertToRx) starts with `if (!oc_ota_tx_mode) return;` —
//     a no-op in exactly that state.
//   * ocRevertToRx() cleared oc_ota_tx_mode unconditionally, even when
//     beginSlaveRx errored, so a failed revert was equally unrecoverable.
//   * onDisconnect() never aborted an in-flight relay, so an app that walked out
//     of range left the OC flipped forever.
//
// Symptom in every case: dma_cb_count stops, no frames reach processFrame,
// nothing is logged, and the LoRa downlink and BLE telemetry freeze at their
// last values until a power cycle. On a rocket that is the whole point of the
// OC, so the recovery must be automatic and must not depend on the app, on the
// FC, or on the power rail being on.
//
// The imperative half (which IDF calls, under which mutex) stays in main.cpp.
// What lives here is the timing, because that is what is worth testing off-target.

namespace OtaRelayPolicy {

// Retry cadence for re-establishing slave RX after a failed begin. Fast enough
// that a transient DMA-allocation failure costs about a second of telemetry,
// slow enough that a hard failure does not spin the loop task.
static constexpr uint32_t kRxRetryIntervalMs = 1000;

// How long the relay may sit flipped to master TX with no chunk arriving before
// we give up and revert. The app pumps continuously once it is told "ready", so
// any real gap is a dead link (BLE supervision timeout is ~2-6 s; a stalled or
// killed app never resumes). Long enough not to trip on a slow phone, short
// enough that a walked-away operator gets telemetry back while still on site.
static constexpr uint32_t kRelayStallTimeoutMs = 10000;

// Should we (re)attempt beginSlaveRx this pass?
//
// `rx_broken` is the honest state — "we have no working slave RX" — as opposed
// to oc_ota_tx_mode, which means "we are deliberately in master TX". Keeping
// them separate is the fix for the original bug: the old code overloaded
// oc_ota_tx_mode with both meanings, so clearing it to stop the feeder also
// disabled the only route back.
inline bool shouldRetryRx(bool rx_broken, uint32_t now_ms, uint32_t last_try_ms)
{
    if (!rx_broken) return false;
    return (uint32_t)(now_ms - last_try_ms) >= kRxRetryIntervalMs;
}

// Has a flipped relay stalled long enough to abandon?
//
// Only armed once the app has been told "ready" (last_chunk_ms != 0): before
// that the silence is ours — the warmup window deliberately idle-fills so the
// FC's slave RX can lock onto BCLK — and timing it out would abort every OTA.
inline bool relayStalled(bool tx_mode, uint32_t last_chunk_ms, uint32_t now_ms)
{
    if (!tx_mode || last_chunk_ms == 0) return false;
    return (uint32_t)(now_ms - last_chunk_ms) >= kRelayStallTimeoutMs;
}

}  // namespace OtaRelayPolicy
