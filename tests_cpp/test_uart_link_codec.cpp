// Tests for the UART radio-modem link codec (#409):
//  - tr_msg::pack produces the exact TR_I2C_Interface::packMessage wire
//    format (golden-byte check so the two codecs can't silently drift)
//  - MsgDeframer round-trips frames from a byte stream, resynchronizes
//    through garbage/corruption, and counts drops
//  - RadioModemProtocol struct layouts stay wire-stable

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include <CRC.h>

#include "../tinkerrocket-idf/components/TR_UART_Link/TR_MsgCodec.h"
#include "../tinkerrocket-idf/components/TR_UART_Link/RadioModemProtocol.h"

using tr_msg::MsgDeframer;

namespace
{

std::vector<uint8_t> packVec(uint8_t type, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> out(tr_msg::MAX_FRAME);
    size_t frame_len = 0;
    EXPECT_TRUE(tr_msg::pack(type, payload.empty() ? nullptr : payload.data(),
                             payload.size(), out.data(), out.size(), frame_len));
    out.resize(frame_len);
    return out;
}

// Feed a byte vector through a deframer, collecting completed frames.
struct Collected
{
    uint8_t type;
    std::vector<uint8_t> payload;
};

std::vector<Collected> run(MsgDeframer& d, const std::vector<uint8_t>& bytes)
{
    std::vector<Collected> got;
    for (uint8_t b : bytes)
    {
        if (d.feed(b))
        {
            got.push_back({d.type(),
                           {d.payload(), d.payload() + d.payloadLen()}});
        }
    }
    return got;
}

}  // namespace

// ---------------------------------------------------------------------------
// pack: wire format
// ---------------------------------------------------------------------------

TEST(UartLinkCodec, PackGoldenFrame)
{
    // Golden layout: SOF(4) + type + len + payload + CRC16be(type+len+payload).
    // CRC computed with the same library call TR_I2C_Interface::packMessage
    // uses — if either side changes polynomial/params, this fails.
    const std::vector<uint8_t> payload = {0x01, 0x02, 0x03};
    const auto frame = packVec(0x42, payload);

    ASSERT_EQ(frame.size(), 4u + 1 + 1 + 3 + 2);
    EXPECT_EQ(frame[0], 0xAA);
    EXPECT_EQ(frame[1], 0x55);
    EXPECT_EQ(frame[2], 0xAA);
    EXPECT_EQ(frame[3], 0x55);
    EXPECT_EQ(frame[4], 0x42);
    EXPECT_EQ(frame[5], 0x03);
    EXPECT_EQ(frame[6], 0x01);
    EXPECT_EQ(frame[7], 0x02);
    EXPECT_EQ(frame[8], 0x03);

    const uint8_t crc_input[] = {0x42, 0x03, 0x01, 0x02, 0x03};
    const uint16_t crc = calcCRC16(crc_input, sizeof(crc_input));
    EXPECT_EQ(frame[9], static_cast<uint8_t>(crc >> 8));
    EXPECT_EQ(frame[10], static_cast<uint8_t>(crc & 0xFF));

    // The check above proves pack() calls calcCRC16 — it cannot catch a change
    // to the CRC *parameters*, because the expectation moves with them. Pin
    // the literal bytes too. These same constants are carried by the bench
    // host (tools/bench_radio_modem.py, GOLDEN), which reimplements the CRC
    // rather than linking this one, so the two implementations cannot drift
    // apart silently — which is the entire point of a golden-byte test.
    const std::vector<uint8_t> golden = {0xAA, 0x55, 0xAA, 0x55, 0x42, 0x03,
                                         0x01, 0x02, 0x03, 0x00, 0x66};
    EXPECT_EQ(frame, golden);
}

TEST(UartLinkCodec, PackGoldenEmptyPayload)
{
    // len = 0 is the common case on this link (GET_STATUS / GET_IDENTITY /
    // START_RX are all payload-free), and it is the edge a length-handling
    // regression breaks first.
    using namespace radio_modem;
    const std::vector<uint8_t> golden_identity = {0xAA, 0x55, 0xAA, 0x55,
                                                  MSG_GET_IDENTITY, 0x00,
                                                  0x04, 0x00};
    const std::vector<uint8_t> golden_status = {0xAA, 0x55, 0xAA, 0x55,
                                                MSG_GET_STATUS, 0x00,
                                                0x06, 0x00};
    EXPECT_EQ(packVec(MSG_GET_IDENTITY, {}), golden_identity);
    EXPECT_EQ(packVec(MSG_GET_STATUS, {}), golden_status);
}

TEST(UartLinkCodec, PackRejectsBadArgs)
{
    uint8_t out[tr_msg::MAX_FRAME];
    size_t frame_len = 0;

    // Null output
    EXPECT_FALSE(tr_msg::pack(1, nullptr, 0, nullptr, sizeof(out), frame_len));
    // Payload bytes promised but null pointer
    EXPECT_FALSE(tr_msg::pack(1, nullptr, 3, out, sizeof(out), frame_len));
    // Capacity too small
    EXPECT_FALSE(tr_msg::pack(1, nullptr, 0, out, 7, frame_len));
    // Zero-length payload is legal
    EXPECT_TRUE(tr_msg::pack(1, nullptr, 0, out, sizeof(out), frame_len));
    EXPECT_EQ(frame_len, tr_msg::FRAME_OVERHEAD);
}

// ---------------------------------------------------------------------------
// Deframer: round trips
// ---------------------------------------------------------------------------

TEST(UartLinkCodec, RoundTripSingleFrame)
{
    MsgDeframer d;
    const std::vector<uint8_t> payload = {9, 8, 7, 6, 5};
    const auto got = run(d, packVec(0x11, payload));

    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].type, 0x11);
    EXPECT_EQ(got[0].payload, payload);
    EXPECT_EQ(d.stats().frames, 1u);
    EXPECT_EQ(d.stats().crc_fails, 0u);
    EXPECT_EQ(d.stats().resync_bytes, 0u);
}

TEST(UartLinkCodec, RoundTripZeroAndMaxPayload)
{
    MsgDeframer d;
    std::vector<uint8_t> maxp(tr_msg::MAX_PAYLOAD);
    for (size_t i = 0; i < maxp.size(); i++)
    {
        maxp[i] = static_cast<uint8_t>(i);
    }

    std::vector<uint8_t> stream = packVec(0x01, {});
    const auto f2 = packVec(0x02, maxp);
    stream.insert(stream.end(), f2.begin(), f2.end());

    const auto got = run(d, stream);
    ASSERT_EQ(got.size(), 2u);
    EXPECT_EQ(got[0].type, 0x01);
    EXPECT_TRUE(got[0].payload.empty());
    EXPECT_EQ(got[1].type, 0x02);
    EXPECT_EQ(got[1].payload, maxp);
}

TEST(UartLinkCodec, ResyncThroughLeadingGarbage)
{
    MsgDeframer d;
    std::vector<uint8_t> stream = {0x00, 0xFF, 0xAA, 0xAA, 0x13, 0x55};
    const auto frame = packVec(0x33, {1, 2});
    stream.insert(stream.end(), frame.begin(), frame.end());

    const auto got = run(d, stream);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].type, 0x33);

    // Exactly 5, not "> 0" and not 6. Six bytes of garbage go in, but the
    // counter tracks what the state machine DISCARDS, and one of the two
    // 0xAAs is consumed as a SOF candidate rather than dropped (see rehunt).
    // This is pinned rather than loose because the value is reported to hosts
    // as STATUS.uart_rx_resync_bytes, and anything that recomputes it must
    // agree — tools/bench_radio_modem.py got 6 with a scan-for-SOF deframer,
    // and only the on-hardware self-test caught it.
    EXPECT_EQ(d.stats().resync_bytes, 5u);
}

TEST(UartLinkCodec, CorruptedFrameDroppedNextFrameSurvives)
{
    MsgDeframer d;
    auto bad = packVec(0x44, {10, 20, 30});
    bad[7] ^= 0xFF;  // corrupt a payload byte -> CRC fail
    const auto good = packVec(0x55, {42});

    std::vector<uint8_t> stream = bad;
    stream.insert(stream.end(), good.begin(), good.end());

    const auto got = run(d, stream);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].type, 0x55);
    ASSERT_EQ(got[0].payload.size(), 1u);
    EXPECT_EQ(got[0].payload[0], 42);
    EXPECT_EQ(d.stats().crc_fails, 1u);
}

TEST(UartLinkCodec, TruncatedFrameThenNewSofRecovers)
{
    MsgDeframer d;
    auto truncated = packVec(0x66, {1, 2, 3, 4, 5, 6, 7, 8});
    truncated.resize(truncated.size() - 6);  // cut mid-payload
    const auto good = packVec(0x77, {0xEE});

    std::vector<uint8_t> stream = truncated;
    stream.insert(stream.end(), good.begin(), good.end());

    // A truncation costs EXACTLY TWO frames. This framer is length-driven, so
    // the partial frame keeps consuming: it swallows the next frame's SOF as
    // its own payload/CRC, fails the CRC, and recovers on the frame after.
    // The second frame is given a different type so the test can prove WHICH
    // one was eaten rather than just that something got through.
    const auto survivor = packVec(0x78, {0xEF});
    stream.insert(stream.end(), survivor.begin(), survivor.end());

    const auto got = run(d, stream);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].type, 0x78);  // 0x77 is the expected casualty
    ASSERT_EQ(got[0].payload.size(), 1u);
    EXPECT_EQ(got[0].payload[0], 0xEF);
    EXPECT_EQ(d.stats().crc_fails, 1u);
}

TEST(UartLinkCodec, BackToBackFramesNoLoss)
{
    MsgDeframer d;
    std::vector<uint8_t> stream;
    for (int i = 0; i < 50; i++)
    {
        const auto f = packVec(static_cast<uint8_t>(i),
                               {static_cast<uint8_t>(i), static_cast<uint8_t>(i + 1)});
        stream.insert(stream.end(), f.begin(), f.end());
    }
    const auto got = run(d, stream);
    ASSERT_EQ(got.size(), 50u);
    for (int i = 0; i < 50; i++)
    {
        EXPECT_EQ(got[i].type, static_cast<uint8_t>(i));
    }
}

// ---------------------------------------------------------------------------
// RadioModemProtocol: wire-stable layouts
// ---------------------------------------------------------------------------

TEST(RadioModemProtocol, StructSizesAreWireStable)
{
    using namespace radio_modem;
    EXPECT_EQ(sizeof(TxFrameHeader), 1u);
    EXPECT_EQ(sizeof(RadioConfigData), 16u);
    EXPECT_EQ(sizeof(HopFreqData), 4u);
    EXPECT_EQ(sizeof(RxFrameHeader), 8u);
    EXPECT_EQ(sizeof(TxResultData), 2u);
    EXPECT_EQ(sizeof(ModemIdentityData), 44u);
    EXPECT_EQ(sizeof(ModemStatusData), 52u);
    EXPECT_EQ(sizeof(ScanRequestData), 12u);
    EXPECT_EQ(sizeof(ScanResultHeader), 12u);
}

TEST(RadioModemProtocol, ScanResultFitsOneFrame)
{
    // SCAN_MAX_SAMPLES int8 samples + header must fit a single tr_msg frame
    // (the protocol relies on this to avoid chunking).
    EXPECT_LE(sizeof(radio_modem::ScanResultHeader) + 128, tr_msg::MAX_PAYLOAD);
}

TEST(RadioModemProtocol, MaxAirFrameFitsWithHeaders)
{
    // MAX_AIR_FRAME must fit alongside BOTH per-frame headers so anything a
    // host may transmit is also deliverable on the receive side.
    EXPECT_LE(radio_modem::MAX_AIR_FRAME + sizeof(radio_modem::TxFrameHeader),
              tr_msg::MAX_PAYLOAD);
    EXPECT_LE(radio_modem::MAX_AIR_FRAME + sizeof(radio_modem::RxFrameHeader),
              tr_msg::MAX_PAYLOAD);
}
