#pragma once

#include <stddef.h>
#include <stdint.h>

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

// ============================================================================
// What the download sends next: every stored byte, not only complete frames.
//
// The download loop used to scan the read buffer for frame starts
// (AA 55 AA 55) and send only complete frames. A byte that did not start a
// frame was stepped over (`pos++`) and never sent, and a partial frame left in
// the carry-over at end of file was dropped. But the file listing reports the
// stored size (TR_FlightLog's final_bytes) and both apps reject a download
// that falls short of the listed size, so any such byte made the whole file
// undownloadable from the phone: the V5 nose cone's 2026-10-24 flight listed
// 6,901,811 B, streamed 6,901,781 and the OC logged "Download complete", while
// the app's row went to 100 % and then to its failed state. The bytes are the
// file's — the post-flight tools resync on frame starts and check CRCs — so
// the stream must carry them as stored.
//
// A segment is either one whole frame (kept in one chunk where it fits, as
// before, via planAppend) or the bytes up to the next frame start, sent as they
// are. Only the front of the buffer is examined; the caller sends the segment,
// advances by `len`, and asks again. `len` 0 means the rest must wait for the
// next read: a frame whose length runs past the buffer, a frame start whose
// length byte has not arrived, or 1-3 trailing bytes that could be the start
// of a frame start split across reads. At end of file nothing waits: whatever
// is left goes out as it is.
// ============================================================================

struct Segment
{
    size_t len;     // bytes to send from the front of the buffer; 0 = read more first
    bool   frame;   // a whole log frame (the loop's frames_sent counts these)
};

/// Frame layout: [AA][55][AA][55][type][len][payload(len)][CRC(2)].
static constexpr size_t kFrameOverhead = 8;   // SOF(4) + type(1) + len(1) + CRC(2)

/// The next segment at the front of `buf` (`avail` bytes). `eof`: no more
/// bytes will follow these.
inline Segment nextSegment(const uint8_t* buf, size_t avail, bool eof)
{
    static constexpr uint8_t kSof[4] = {0xAA, 0x55, 0xAA, 0x55};
    if (buf == nullptr || avail == 0) return {0, false};

    const bool at_sof = avail >= 4 && buf[0] == kSof[0] && buf[1] == kSof[1] &&
                        buf[2] == kSof[2] && buf[3] == kSof[3];
    if (at_sof)
    {
        if (avail >= 6)
        {
            const size_t frame = kFrameOverhead + buf[5];
            if (frame <= avail) return {frame, true};
        }
        // A frame start whose length or body runs past the buffer.
        return eof ? Segment{avail, false} : Segment{0, false};
    }

    // Not a frame start: the bytes up to the next one go out as they are.
    for (size_t i = 1; i + 4 <= avail; ++i)
    {
        if (buf[i] == kSof[0] && buf[i + 1] == kSof[1] &&
            buf[i + 2] == kSof[2] && buf[i + 3] == kSof[3])
        {
            return {i, false};
        }
    }
    if (eof) return {avail, false};

    // No frame start in the buffer. Hold back a tail that could be the first
    // 1-3 bytes of one, completed by the next read.
    size_t hold = 0;
    for (size_t k = (avail < 3 ? avail : 3); k > 0; --k)
    {
        bool prefix = true;
        for (size_t j = 0; j < k; ++j)
        {
            if (buf[avail - k + j] != kSof[j]) { prefix = false; break; }
        }
        if (prefix) { hold = k; break; }
    }
    return {avail - hold, false};
}

}  // namespace tr_ble
