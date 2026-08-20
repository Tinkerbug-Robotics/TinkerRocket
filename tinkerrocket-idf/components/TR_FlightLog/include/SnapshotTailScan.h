#pragma once

#include <stdint.h>
#include <string.h>

#include "TR_FlightLog.h"
#include <CRC.h>

// #846: find the LAST valid frame of a given type in the tail of a flight's
// logged frame stream — the mini's snapshotTailScan (rocket_computer_mini
// flight.cpp), lifted into the component so the OC's boot re-seed and the
// host tests share one implementation. The mini keeps its own copy for now;
// unifying it is follow-up.
//
// The stream is the standard wire framing every log consumer already parses:
//   [AA 55 AA 55][type][len u8][payload][CRC16 hi][CRC16 lo]
// with CRC16 (components/CRC, poly 0x8001, init 0, unreflected) over
// type..payload — validated here directly so this header needs nothing
// beyond TR_FlightLog + CRC.
//
// TWO answers, because stream order and payload order can disagree. The
// LAST valid frame is what says whether the flight ENDED (the FC's "clear"
// is a LANDED-state snapshot, and it must beat an earlier INFLIGHT one). But
// stream order is not reliable for picking WHICH in-flight frame is newest:
// the OC logs every received frame before the #383 stale-replay guard runs,
// so an I2S DMA replay can append a snapshot a few hundred ms OLDER than the
// one before it. Restoring from that mis-phases the flight (an apogee charge
// that already fired reads as unfired).
//
// So `last_out` gets the stream's final word, and — when the caller names an
// ordering field — `best_out` gets the frame with the highest value of that
// u32. A snapshot caller uses both: decline if the last frame is not
// INFLIGHT, otherwise restore from the highest flight_elapsed_ms.
// This function checks FRAMING only; state/magic/CRC32/sim are the caller's.
//
// Reads via readFlightPage (byte-addressed, at most one page-remainder per
// call, hops PageHeaders and bad-block gaps), with a carry of up to one
// frame across read windows so a frame straddling a page boundary still
// parses. The window is a BYTE budget on purpose — the per-page payload is
// runtime chip geometry (#671: 4080 legacy, 2032 GD5F).

namespace tr_flightlog {

// Scan the last `window_bytes` of `filename` (logical length `final_bytes`)
// for frames of `frame_type` whose payload is exactly `payload_len` bytes.
// Returns true if any valid frame was found, copying the stream's LAST one
// into `last_out` (4+1+1+payload_len+2 bytes).
//
// When `best_out` is non-null it additionally receives the frame with the
// highest u32 at payload offset `order_off` (see the header note); pass
// nullptr for pure last-wins.
//
// `scratch`/`scratch_len` is the caller-provided read window and must hold at
// least (max per-page payload + frame length) bytes — sizing it to
// (NAND_PAGE_SIZE_MAX - 16) + frame keeps it geometry-proof.
inline bool tailScanForFrame(TR_FlightLog& fl, const char* filename,
                             uint32_t final_bytes, uint8_t frame_type,
                             uint8_t payload_len, uint32_t window_bytes,
                             uint8_t* last_out,
                             uint8_t* scratch, size_t scratch_len,
                             uint8_t* best_out = nullptr,
                             uint16_t order_off = 0)
{
    const size_t frame_len = 4u + 1u + 1u + (size_t)payload_len + 2u;
    if (scratch_len < frame_len) return false;
    uint32_t best_order = 0;
    bool     have_best  = false;

    const uint32_t scan_start =
        (final_bytes > window_bytes) ? final_bytes - window_bytes : 0u;
    uint32_t offset = scan_start;
    size_t   carry  = 0;
    bool     found  = false;

    while (offset < final_bytes)
    {
        size_t got = 0;
        const Status st = fl.readFlightPage(filename, offset,
                                            scratch + carry,
                                            scratch_len - carry, got);
        if (st != Status::Ok)
        {
            // A read error means we never reached the stream's real end, so
            // "last valid frame" is unknowable — anything found so far could
            // be superseded by a frame past the unreadable page (including
            // the FC's LANDED clear, whose whole job is to say the flight is
            // over). Fail rather than hand back a frame that might be stale:
            // for recovery, a missed restore is recoverable and a wrong one
            // re-arms a grounded vehicle.
            return false;
        }
        if (got == 0) break;   // clean end of stream

        const size_t avail = carry + got;
        size_t pos = 0;
        while (pos + frame_len <= avail)
        {
            if (scratch[pos] == 0xAA && scratch[pos + 1] == 0x55 &&
                scratch[pos + 2] == 0xAA && scratch[pos + 3] == 0x55 &&
                scratch[pos + 4] == frame_type &&
                scratch[pos + 5] == payload_len)
            {
                const uint16_t want =
                    (uint16_t)((scratch[pos + 6 + payload_len] << 8) |
                               scratch[pos + 7 + payload_len]);
                const uint16_t got_crc =
                    calcCRC16(scratch + pos + 4, (int)(2u + payload_len));
                if (want == got_crc)
                {
                    memcpy(last_out, scratch + pos, frame_len);
                    found = true;              // stream's final word so far
                    if (best_out != nullptr &&
                        (size_t)order_off + 4u <= (size_t)payload_len)
                    {
                        uint32_t ord = 0;
                        memcpy(&ord, scratch + pos + 6 + order_off, sizeof(ord));
                        if (!have_best || ord > best_order)
                        {
                            best_order = ord;
                            have_best  = true;
                            memcpy(best_out, scratch + pos, frame_len);
                        }
                    }
                    pos += frame_len;
                    continue;
                }
            }
            ++pos;
        }
        carry = avail - pos;                   // < frame_len by construction
        if (carry > 0) memmove(scratch, scratch + pos, carry);
        offset += got;
    }
    return found;
}

}  // namespace tr_flightlog
