// Host tests for BleChunkSize.h — the #524 file-transfer chunk sizing.
//
// The point of the module is one non-obvious claim: the FASTEST chunk is not the
// BIGGEST chunk. Downloads are bounded by packets per connection event, so a
// chunk that overflows into a nearly-empty third link-layer packet is slower than
// a slightly smaller one that fits in two full packets. These tests pin that.

#include <gtest/gtest.h>

#include "BleChunkSize.h"

using tr_ble::kWireOverhead;
using tr_ble::maxChunkDataSize;

namespace
{

// Bytes the link layer actually puts on air for a chunk of `data` bytes:
// L2CAP(4) + ATT(3) + chunk header(7) + data, split into ll-octet packets.
int llPacketsFor(size_t data, uint16_t ll)
{
    const size_t pdu = data + kWireOverhead;
    return static_cast<int>((pdu + ll - 1) / ll);  // ceil
}

}  // namespace

// ---------------------------------------------------------------------------
// The regression this module exists to fix.
// ---------------------------------------------------------------------------

// MTU 512 + DLE 251 is the negotiated state on the bench (iOS + ESP32-S3).
TEST(BleChunkSize, MtuMaximalChunkWastesAThirdPacket)
{
    // What the OLD code did: mtu - ATT(3) - header(7).
    const size_t mtu_maximal = 512 - 3 - 7;
    ASSERT_EQ(mtu_maximal, 502u);

    // 502 + 14 = 516 octets on the wire -> 251 + 251 + 14. Three packets, and the
    // third carries fourteen bytes.
    EXPECT_EQ(llPacketsFor(mtu_maximal, 251), 3);
}

TEST(BleChunkSize, AlignsToWholeLinkLayerPackets)
{
    const size_t chunk = maxChunkDataSize(512, 251);

    EXPECT_EQ(chunk, 488u);                    // 2 * 251 - 14
    EXPECT_EQ(llPacketsFor(chunk, 251), 2);    // both packets full
    EXPECT_EQ(chunk + kWireOverhead, 2u * 251);  // exactly, no stub
}

// The whole justification for giving up payload: useful bytes per packet is the
// figure of merit, not bytes per chunk.
TEST(BleChunkSize, TradesPayloadForThroughput)
{
    const size_t old_chunk = 502;                 // MTU-maximal
    const size_t new_chunk = maxChunkDataSize(512, 251);

    EXPECT_LT(new_chunk, old_chunk);              // 2.8% smaller chunk...

    const double old_per_pkt = double(old_chunk) / llPacketsFor(old_chunk, 251);
    const double new_per_pkt = double(new_chunk) / llPacketsFor(new_chunk, 251);

    EXPECT_NEAR(old_per_pkt, 167.3, 0.1);
    EXPECT_NEAR(new_per_pkt, 244.0, 0.1);
    EXPECT_GT(new_per_pkt / old_per_pkt, 1.45);   // ...for ~1.46x the throughput
}

// ---------------------------------------------------------------------------
// Invariants that must hold for every link we could end up on.
// ---------------------------------------------------------------------------

// Smallest MTU we will actually size a chunk against; below this we cannot know
// the real MTU and fall back (see UnnegotiatedMtuAssumesIosDefault).
constexpr uint16_t kFirstRealMtu = tr_ble::kMinUsableMtu + 1;

TEST(BleChunkSize, NeverExceedsTheMtu)
{
    // An oversized notification is not slow, it is broken — the peer drops it.
    for (uint16_t mtu = kFirstRealMtu; mtu <= 517; ++mtu)
    {
        for (uint16_t ll : {uint16_t(27), uint16_t(100), uint16_t(251)})
        {
            const size_t chunk = maxChunkDataSize(mtu, ll);
            EXPECT_LE(chunk + 3u + 7u, mtu) << "mtu=" << mtu << " ll=" << ll;
        }
    }
}

TEST(BleChunkSize, NeverReturnsZero)
{
    for (uint16_t mtu = kFirstRealMtu; mtu <= 517; ++mtu)
    {
        for (uint16_t ll : {uint16_t(27), uint16_t(100), uint16_t(251)})
        {
            EXPECT_GT(maxChunkDataSize(mtu, ll), 0u) << "mtu=" << mtu << " ll=" << ll;
        }
    }
}

// DLE is negotiated per connection and may simply not happen. 27 octets is the
// floor every BLE link starts at, and we must still produce a sane chunk.
TEST(BleChunkSize, HandlesLinkWithoutDataLengthExtension)
{
    const size_t chunk = maxChunkDataSize(512, 27);

    EXPECT_EQ(chunk, 499u);                   // 19 * 27 - 14
    EXPECT_EQ(llPacketsFor(chunk, 27), 19);   // all 19 full
    EXPECT_EQ((chunk + kWireOverhead) % 27, 0u);
    EXPECT_LE(chunk, 502u);                   // still within MTU 512
}

// ---------------------------------------------------------------------------
// Degenerate links: alignment must degrade to "as big as the MTU allows", never
// to something nonsensical.
// ---------------------------------------------------------------------------

// iOS's pre-negotiation default MTU is 185: one 251-octet fragment cannot even be
// filled, so there is nothing to align to.
TEST(BleChunkSize, SmallMtuFallsBackToMtuMaximal)
{
    EXPECT_EQ(maxChunkDataSize(185, 251), 185u - 3 - 7);
}

// Below kFirstRealMtu we do not have a usable MTU to size against, and the
// fallback deliberately does NOT fit it: 170 + 10 = 180 bytes needs an MTU of
// 180, not 23. That is on purpose and predates #524.
//
// The reason is #283: effectiveMtu() can report a STALE 0/23 on some iOS
// reconnect paths that reuse the prior MTU without re-firing BLE_GAP_EVENT_MTU,
// while the link is really running at 185 or 512. Honouring 23 literally would
// pin those reconnects to 13-byte chunks — far worse than assuming iOS's 185-byte
// default and being right almost every time. So: when the MTU is unknown, guess
// iOS; do not clamp.
TEST(BleChunkSize, UnnegotiatedMtuAssumesIosDefault)
{
    EXPECT_EQ(maxChunkDataSize(23, 251), tr_ble::kFallbackChunkData);  // pre-exchange
    EXPECT_EQ(maxChunkDataSize(0, 251), tr_ble::kFallbackChunkData);

    // 170 fits iOS's 185-byte default with room to spare — which is the bet.
    EXPECT_LE(tr_ble::kFallbackChunkData + 3u + 7u, 185u);
}

// A fragment smaller than the header we must prepend would underflow the
// alignment arithmetic. Guard it rather than wrap to a huge size_t.
TEST(BleChunkSize, PathologicalFragmentSizeDoesNotUnderflow)
{
    for (uint16_t ll = 0; ll <= kWireOverhead; ++ll)
    {
        const size_t chunk = maxChunkDataSize(512, ll);
        EXPECT_EQ(chunk, 502u) << "ll=" << ll;   // MTU-maximal, not garbage
        EXPECT_LT(chunk, 1024u);                 // definitely not a wrapped size_t
    }
}
