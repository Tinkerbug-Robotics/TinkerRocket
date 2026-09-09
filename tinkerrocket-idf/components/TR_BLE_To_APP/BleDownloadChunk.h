#pragma once

#include <stddef.h>

// ============================================================================
// #1160 / #1155 item 2: how a log frame goes into a download chunk.
//
// The download loop stages consecutive log frames into one chunk buffer and
// sends the buffer whenever the next frame would not fit. The original rule was
//
//     if (ble_used > 0 && ble_used + frame_size > chunk_data_size) flush;
//     append(frame);
//
// which bounds a chunk to chunk_data_size ONLY while the stage is non-empty. A
// frame larger than chunk_data_size arriving at an empty stage is appended
// whole and sent whole — an ATT notification bigger than the negotiated MTU.
// NimBLE's ble_att_tx_dflt() calls ble_att_truncate_to_mtu(), drops the
// excess, and returns 0: sendFileChunk() reports success, the loop counts the
// bytes as sent, and the app receives a chunk whose header promises more bytes
// than arrived. The flight log routinely carries such frames — SNAPSHOT_MSG is
// 232 B at 10 Hz through INFLIGHT, FlightSettingsData 228 B — and the common
// iOS MTU of 185 fits 175. Every such flight was undownloadable at that MTU,
// with the OC logging "Download complete".
//
// A chunk is an opaque, contiguous byte range of the .bin: both apps reassemble
// by offset and check contiguity, never frame boundaries. So a frame that does
// not fit a chunk is simply sent as consecutive pieces of at most
// chunk_data_size bytes. This header decides that; it is pure integer
// arithmetic so the rule — and the case that was wrong — is host-tested.
// ============================================================================

namespace tr_ble
{

struct AppendPlan
{
    // Send the staged bytes as their own chunk before touching this frame.
    // Only ever true when there is something staged.
    bool   flush_first;
    // 0: append the whole frame to the stage.
    // N: the frame cannot fit even an empty stage; send it directly from the
    //    read buffer as N consecutive pieces of at most `chunk` bytes.
    size_t direct_pieces;
};

/// Decide what to do with the next complete frame.
/// @param used   bytes already staged in the chunk buffer
/// @param frame  size of the next complete frame
/// @param chunk  chunk payload capacity (getMaxChunkDataSize()); the caller
///               already refuses to start a download when this is 0
inline AppendPlan planAppend(size_t used, size_t frame, size_t chunk)
{
    if (chunk == 0)                 return {false, 0};   // caller-guarded; never split by 0
    if (used + frame <= chunk)      return {false, 0};   // fits behind what is staged
    if (frame <= chunk)             return {used > 0, 0};// fits an empty stage: flush, then append
    // Larger than any chunk: flush whatever is staged, then stream the frame.
    return {used > 0, (frame + chunk - 1) / chunk};
}

/// Length of piece `index` (0-based) when a `frame`-byte frame is streamed in
/// pieces of at most `chunk` bytes. Every piece but the last is exactly
/// `chunk`; the last carries the remainder (or a full `chunk`).
inline size_t pieceLen(size_t frame, size_t chunk, size_t index)
{
    if (chunk == 0) return 0;
    const size_t start = index * chunk;
    if (start >= frame) return 0;
    const size_t left = frame - start;
    return left < chunk ? left : chunk;
}

}  // namespace tr_ble
