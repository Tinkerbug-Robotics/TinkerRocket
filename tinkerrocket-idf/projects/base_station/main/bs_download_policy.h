#pragma once

// Base-station BLE file-download chunking policy (#380).
//
// Pure helpers extracted from main.cpp's cooperative download state machine so
// the host-side gtest can drive them without SD/FILE* or the BLE stack.  Must
// stay free of ESP-IDF / FreeRTOS / hardware dependencies.
//
// Background: the download used to run as a synchronous while-loop on the
// single bs_loop task (fread + sendFileChunk + delay(15 ms) per 170 B chunk),
// blocking telemetry RX, uplink retries, heartbeats, and CSV flushes for the
// whole transfer (~11 s per 120 KB; minutes for a multi-MB log). It is now a
// per-loop-iteration state machine; these predicates carry the arithmetic.

#include <cstdint>

namespace bs_download_policy {

struct ChunkPlan {
    uint32_t read_len;  // bytes to read/send this chunk (0 => empty-EOF chunk)
    bool     eof;       // set the EOF flag on this chunk (terminal)
};

// Plan the next chunk of a transfer bounded by the file size measured at open.
// Bounding by the open-time size keeps a download of the ACTIVE (still-growing)
// log finite; growth past `file_size` is simply not part of this transfer.
//
// The empty-file case (file_size == 0) falls out naturally: read_len 0 +
// eof true == the single empty EOF chunk the old code special-cased.
static inline ChunkPlan nextChunk(uint32_t file_size, uint32_t offset,
                                  uint32_t chunk_size)
{
    ChunkPlan p{};
    const uint32_t remaining = (offset < file_size) ? (file_size - offset) : 0;
    p.read_len = (remaining < chunk_size) ? remaining : chunk_size;
    p.eof      = (offset + p.read_len >= file_size);
    return p;
}

// Inter-chunk pacing gate. The 15 ms spacing is NOT arbitrary — it mirrors
// notify_data()'s mbuf drain budget (TR_BLE_To_APP), giving the controller
// time to flush notifications so the pool doesn't exhaust. The bs_loop
// iterates every ~1 ms, so the state machine must gate on wall time, not on
// loop cadence. last_chunk_ms == 0 means "no chunk sent yet" -> send now.
// Unsigned subtraction keeps the comparison millis()-wraparound-safe.
static inline bool mayEmitChunk(uint32_t now_ms, uint32_t last_chunk_ms,
                                uint32_t min_interval_ms)
{
    if (last_chunk_ms == 0) return true;
    return (now_ms - last_chunk_ms) >= min_interval_ms;
}

}  // namespace bs_download_policy
