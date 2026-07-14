#pragma once

#include <stddef.h>
#include <stdint.h>

// ============================================================================
// #524: how big should a file-transfer chunk be?
//
// The obvious answer — "as big as the ATT MTU allows" — is the wrong one, and it
// is what capped downloads at ~17 kB/s.
//
// A chunk notification goes on the wire like this:
//
//   [ L2CAP hdr 4 ][ ATT opcode+handle 3 ][ chunk hdr 7 ][ data ... ]
//   \______________________ one L2CAP PDU _________________________/
//
// The link layer then splits that PDU into LL data packets of at most
// ll_tx_octets — 27 before Data Length Extension, 251 after.
//
// Throughput on this link is bounded by PACKETS PER CONNECTION EVENT, not by
// bytes: iOS grants a 30 ms interval (Apple's envelope makes 15-30 the tightest
// legal request, and it serves the top), and only a handful of packets move per
// event. So the figure of merit is USEFUL BYTES PER PACKET.
//
// MTU-maximal is pathological for that metric. At MTU 512 it yields 502 data
// bytes, so the L2CAP PDU is 502 + 14 = 516 octets, which fragments as:
//
//       251 + 251 + 14        <-- a whole third packet to carry FOURTEEN bytes
//
// Three packets for 502 useful bytes = 167 B/packet. Giving up 14 bytes of
// payload (502 -> 488) makes the PDU exactly 2 x 251 = 502 octets:
//
//       251 + 251             <-- both full
//
// Two packets for 488 useful bytes = 244 B/packet. Same airtime, 1.46x the data.
// It also costs one fewer HCI credit per chunk, so more chunks fit in the
// controller's outbound queue and more of them ride each event.
//
// Kept as a free function on plain integers so it is host-testable — see
// tests_cpp/test_ble_chunk_size.cpp.
// ============================================================================

namespace tr_ble
{

static constexpr size_t kL2capHeader   = 4;  // L2CAP basic header
static constexpr size_t kAttNotifyHdr  = 3;  // ATT opcode(1) + attribute handle(2)
static constexpr size_t kChunkHeader   = 7;  // offset(4) + length(2) + flags(1)

// Bytes the link layer must carry in front of every chunk payload.
static constexpr size_t kWireOverhead = kL2capHeader + kAttNotifyHdr + kChunkHeader;  // 14

// Smallest ATT MTU worth sizing a real chunk against. At or below this we cannot
// know the true MTU and fall back (see below).
static constexpr uint16_t kMinUsableMtu = kAttNotifyHdr + kChunkHeader + 20;  // 30

// Chunk size used before the MTU has been negotiated (fits iOS's 185-byte default).
static constexpr size_t kFallbackChunkData = 170;

/// Largest chunk payload that fits the ATT MTU *and* leaves no stub link-layer
/// packet behind.
///
/// @param att_mtu       negotiated ATT MTU (23 until the exchange completes)
/// @param ll_tx_octets  LL max TX payload from BLE_GAP_EVENT_DATA_LEN_CHG;
///                      27 (the pre-DLE minimum) if DLE has not been negotiated
inline size_t maxChunkDataSize(uint16_t att_mtu, uint16_t ll_tx_octets)
{
    // Before the MTU exchange lands there is nothing to optimise against.
    if (att_mtu <= kMinUsableMtu)
    {
        return kFallbackChunkData;
    }

    // Ceiling imposed by the MTU: one ATT notification cannot exceed it.
    const size_t mtu_limit = static_cast<size_t>(att_mtu) - kAttNotifyHdr - kChunkHeader;

    // A fragment smaller than the header we must prepend is not a real link.
    // Take the MTU-maximal size and let the LL fragment it however it must.
    if (ll_tx_octets <= kWireOverhead)
    {
        return mtu_limit;
    }

    // How many WHOLE link-layer packets can we fill without exceeding the MTU?
    const size_t ll        = static_cast<size_t>(ll_tx_octets);
    const size_t fragments = (mtu_limit + kWireOverhead) / ll;

    // MTU too small to fill even one fragment — nothing to align to.
    if (fragments == 0)
    {
        return mtu_limit;
    }

    // Fill exactly that many. <= mtu_limit by construction.
    return fragments * ll - kWireOverhead;
}

}  // namespace tr_ble
