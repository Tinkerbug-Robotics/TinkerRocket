// Host tests for the gnss-sdr-dev board's RTCM3 stream on the flight
// computer (TR_GNSSReceiverSdr_Serial/SdrRtcm.cpp): the frames pulled out of
// the byte stream in any slicing, a lost or corrupted byte costing only the
// frame it was in, and the few fields of TinkerRocket's message 4001 the
// flight computer reads. On a recorded stream (fixtures/sdr_rtcm_stream.h).
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "Rtcm3Msm.h"
#include "SdrRtcm.h"
#include "fixtures/sdr_rtcm_stream.h"

using namespace sdr_rtcm;

namespace {

struct Frame
{
    std::vector<uint8_t> bytes;
    size_t after;
};

struct Collect : Rtcm3Framer::Sink
{
    std::vector<Frame> frames;
    void onFrame(const uint8_t* f, size_t len, size_t after) override
    {
        frames.push_back({std::vector<uint8_t>(f, f + len), after});
    }
};

std::vector<uint8_t> stream() { return std::vector<uint8_t>(kSdrStream, kSdrStream + sizeof(kSdrStream)); }

// The recorded stream's frames, cut by their own length fields.
std::vector<std::vector<uint8_t>> cut(const std::vector<uint8_t>& s)
{
    std::vector<std::vector<uint8_t>> out;
    size_t i = 0;
    while (i + 6 <= s.size())
    {
        const size_t len = 6 + (((s[i + 1] & 3) << 8) | s[i + 2]);
        out.emplace_back(s.begin() + i, s.begin() + i + len);
        i += len;
    }
    return out;
}

Collect feedIn(Rtcm3Framer& f, const std::vector<uint8_t>& s, size_t piece)
{
    Collect c;
    for (size_t i = 0; i < s.size(); i += piece)
        f.feed(s.data() + i, std::min(piece, s.size() - i), c);
    return c;
}

// An RTCM3 frame around `payload`, its CRC-24Q computed.
std::vector<uint8_t> frameOf(const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> f = {0xD3, static_cast<uint8_t>(payload.size() >> 8), static_cast<uint8_t>(payload.size())};
    f.insert(f.end(), payload.begin(), payload.end());
    const uint32_t crc = rtcm3::crc24q(f.data(), f.size());
    f.push_back(static_cast<uint8_t>(crc >> 16));
    f.push_back(static_cast<uint8_t>(crc >> 8));
    f.push_back(static_cast<uint8_t>(crc));
    return f;
}

}  // namespace

TEST(SdrRtcmFixture, IsWholeFramesEveryOneGood)
{
    const auto frames = cut(stream());
    ASSERT_EQ(frames.size(), kSdrStreamFrames);
    size_t total = 0;
    for (const auto& f : frames)
    {
        total += f.size();
        const uint32_t want = (f[f.size() - 3] << 16) | (f[f.size() - 2] << 8) | f[f.size() - 1];
        EXPECT_EQ(rtcm3::crc24q(f.data(), f.size() - 3), want);
    }
    EXPECT_EQ(total, sizeof(kSdrStream));
}

TEST(SdrRtcmFramer, FindsEveryFrameInAnySlicing)
{
    const auto want = cut(stream());
    for (size_t piece : {1u, 2u, 7u, 64u, 128u, 1000u, 5000u})
    {
        Rtcm3Framer f;
        const Collect c = feedIn(f, stream(), piece);
        ASSERT_EQ(c.frames.size(), want.size()) << piece;
        for (size_t i = 0; i < want.size(); i++) EXPECT_EQ(c.frames[i].bytes, want[i]) << piece << " #" << i;
        EXPECT_EQ(f.frames(), want.size());
        EXPECT_EQ(f.crcFails(), 0u) << piece;
        EXPECT_EQ(f.skipped(), 0u) << piece;
    }
}

TEST(SdrRtcmFramer, SaysHowManyBytesCameAfterEachFrame)
{
    // What the driver times each frame by: in one read of the whole stream,
    // the bytes after a frame's last byte.
    Rtcm3Framer f;
    const Collect c = feedIn(f, stream(), sizeof(kSdrStream));
    size_t end = 0;
    for (const Frame& fr : c.frames)
    {
        end += fr.bytes.size();
        EXPECT_EQ(fr.after, sizeof(kSdrStream) - end);
    }
    // Byte by byte, each frame comes out on its own last byte.
    Rtcm3Framer g;
    for (const Frame& fr : feedIn(g, stream(), 1).frames) EXPECT_EQ(fr.after, 0u);
}

TEST(SdrRtcmFramer, ALostByteCostsOnlyItsFrame)
{
    // A byte the UART lost, anywhere in each frame in turn: that frame fails
    // its CRC (or is never whole), every other frame still comes out.
    const auto frames = cut(stream());
    size_t start = 0;
    for (size_t k = 0; k < frames.size(); k++)
    {
        for (size_t at : {size_t{0}, size_t{1}, size_t{2}, frames[k].size() / 2, frames[k].size() - 1})
        {
            std::vector<uint8_t> s = stream();
            s.erase(s.begin() + static_cast<long>(start + at));
            Rtcm3Framer f;
            const Collect c = feedIn(f, s, 97);
            EXPECT_EQ(c.frames.size(), frames.size() - 1) << "frame " << k << " byte " << at;
            for (const Frame& fr : c.frames) EXPECT_NE(fr.bytes, frames[k]);
        }
        start += frames[k].size();
    }
}

TEST(SdrRtcmFramer, AFlippedBitCostsOnlyItsFrame)
{
    const auto frames = cut(stream());
    size_t start = 0;
    for (size_t k = 0; k < frames.size(); k++)
    {
        std::vector<uint8_t> s = stream();
        s[start + frames[k].size() / 2] ^= 0x10;
        Rtcm3Framer f;
        const Collect c = feedIn(f, s, 64);
        EXPECT_EQ(c.frames.size(), frames.size() - 1) << k;
        EXPECT_GE(f.crcFails(), 1u);
        start += frames[k].size();
    }
}

TEST(SdrRtcmFramer, ALengthTurnedLongDoesNotHideTheFramesAfterIt)
{
    // The first frame's length field hit so it claims ~1 kB: the frames
    // inside that span are found again once its CRC fails.
    std::vector<uint8_t> s = stream();
    s[1] = 0x03;
    s[2] = 0xF0;
    Rtcm3Framer f;
    const Collect c = feedIn(f, s, 200);
    EXPECT_EQ(c.frames.size(), kSdrStreamFrames - 1);
    EXPECT_EQ(f.crcFails(), 1u);
}

TEST(SdrRtcmFramer, NoiseBeforeTheStreamIsSkipped)
{
    // A probe opens the UART in the middle of a frame, or hears the line at
    // another baud first: what is no frame is counted and passed over.
    std::vector<uint8_t> s = {0x00, 0xD3, 0x55, 0xD3, 0x00, 0x13, 0x42, 0xFF, 0xD3};
    s.insert(s.end(), kSdrStream + 1500, kSdrStream + 1700);   // a partial frame, starting mid-frame
    const std::vector<uint8_t> whole = stream();
    s.insert(s.end(), whole.begin(), whole.end());
    Rtcm3Framer f;
    const Collect c = feedIn(f, s, 33);
    ASSERT_GE(c.frames.size(), kSdrStreamFrames);
    // The last 45 are the stream, whole.
    const auto want = cut(whole);
    for (size_t i = 0; i < want.size(); i++)
        EXPECT_EQ(c.frames[c.frames.size() - want.size() + i].bytes, want[i]);
    EXPECT_GT(f.skipped(), 0u);
}

TEST(SdrRtcmFramer, ReservedBitsSetIsNoFrame)
{
    std::vector<uint8_t> f = frameOf({0xFA, 0x11, 0x22});   // 4001 sub-type 1, short
    f[1] |= 0x04;
    Rtcm3Framer r;
    Collect c;
    r.feed(f.data(), f.size(), c);
    EXPECT_TRUE(c.frames.empty());
}

TEST(SdrRtcmFramer, TheLongestFrameFits)
{
    std::vector<uint8_t> payload(1023, 0x5A);
    payload[0] = 0x43;   // 1077 in the first 12 bits
    payload[1] = 0x50;
    const std::vector<uint8_t> fr = frameOf(payload);
    ASSERT_EQ(fr.size(), kMaxFrame);
    std::vector<uint8_t> s = fr;
    s.insert(s.end(), fr.begin(), fr.end());
    s.insert(s.end(), fr.begin(), fr.end());
    for (size_t piece : {1u, 500u, 1029u, 4000u})
    {
        Rtcm3Framer r;
        const Collect c = feedIn(r, s, piece);
        ASSERT_EQ(c.frames.size(), 3u) << piece;
        EXPECT_EQ(c.frames[2].bytes, fr);
        EXPECT_EQ(messageNumber(fr.data(), fr.size()), 1077);
    }
}

// ── The messages ────────────────────────────────────────────────────────

TEST(SdrRtcmMessages, NumbersAndSubTypes)
{
    const auto frames = cut(stream());
    EXPECT_EQ(messageNumber(frames[0].data(), frames[0].size()), kTrMessage);
    EXPECT_EQ(trSubType(frames[0].data(), frames[0].size()), kTrFix);
    EXPECT_EQ(messageNumber(frames[1].data(), frames[1].size()), 1019);
    EXPECT_EQ(trSubType(frames[1].data(), frames[1].size()), 0);
    for (int k : {2, 3, 4}) EXPECT_EQ(messageNumber(frames[k].data(), frames[k].size()), 1046);
    size_t epochs = 0;
    for (size_t i = 5; i < frames.size(); i += 4)
    {
        EXPECT_EQ(messageNumber(frames[i].data(), frames[i].size()), 1077);
        EXPECT_EQ(messageNumber(frames[i + 1].data(), frames[i + 1].size()), 1097);
        EXPECT_EQ(messageNumber(frames[i + 2].data(), frames[i + 2].size()), 1127);
        EXPECT_EQ(trSubType(frames[i + 3].data(), frames[i + 3].size()), kTrEpoch);
        epochs++;
    }
    EXPECT_EQ(epochs, kSdrStreamEpochs);
    EXPECT_EQ(messageNumber(nullptr, 0), 0);
    const uint8_t tiny[6] = {0xD3, 0, 0, 0, 0, 0};
    EXPECT_EQ(messageNumber(tiny, sizeof(tiny)), 0);
}

// The status the board sends once a second before its first fix (sub-type 3), as its own encoder made it
// (receiver/core/nav/rtcm_msm.c rtcm_tr_status_encode in the gnss repo): a60b606a-dirty, up 73 s, 13 channels
// tracking, 11 locked, no time.
TEST(SdrRtcmMessages, TheBoardsStatusBeforeItsFirstFix)
{
    const std::vector<uint8_t> f = {0xD3, 0x00, 0x0B, 0xFA, 0x13, 0xA6, 0x0B, 0x60, 0x6A, 0x80, 0x02, 0x49,
                                    0xA5, 0x80, 0x85, 0xF8, 0x88};
    EXPECT_EQ(rtcm3::crc24q(f.data(), f.size() - 3), (uint32_t(0x85) << 16) | (uint32_t(0xF8) << 8) | 0x88);
    EXPECT_EQ(messageNumber(f.data(), f.size()), kTrMessage);
    EXPECT_EQ(trSubType(f.data(), f.size()), kTrStatus);
    TrStatus st;
    ASSERT_TRUE(decodeTrStatus(f.data(), f.size(), st));
    EXPECT_EQ(st.fw, 0xa60b606au);
    EXPECT_TRUE(st.fw_dirty);
    EXPECT_EQ(st.uptime_s, 73u);
    EXPECT_EQ(st.ntrack, 13);
    EXPECT_EQ(st.nlock, 11);
    EXPECT_FALSE(st.time_valid);
    EXPECT_FALSE(st.fix_valid);
    TrEpoch e;
    TrFix x;
    EXPECT_FALSE(decodeTrEpoch(f.data(), f.size(), e));
    EXPECT_FALSE(decodeTrFix(f.data(), f.size(), x));
    // and the framer passes it through whole
    Rtcm3Framer fr;
    const Collect c = feedIn(fr, f, 5);
    ASSERT_EQ(c.frames.size(), 1u);
    EXPECT_EQ(c.frames[0].bytes, f);
}

TEST(SdrRtcmMessages, TheEpochsAre10msApartIn100HzMode)
{
    // As the board's own decoder (receiver/host/rtcmdump) reads them.
    const auto frames = cut(stream());
    uint32_t tow = kSdrStreamFirstEpochTow;
    for (size_t i = 8; i < frames.size(); i += 4)
    {
        TrEpoch e;
        ASSERT_TRUE(decodeTrEpoch(frames[i].data(), frames[i].size(), e)) << i;
        EXPECT_EQ(e.week, 2443);
        EXPECT_EQ(e.tow_ms, tow);
        EXPECT_EQ(e.mode, 1);
        EXPECT_EQ(e.nobs, 18);
        EXPECT_EQ(e.clk_steps, 18);
        EXPECT_NEAR(e.clk_step_m, 2591783.819, 1e-6);   // to the millimetre
        EXPECT_EQ(e.shift_ns == 0, false);
        EXPECT_EQ(e.sample % 67500, 0u);                // 10 ms of the 6.75 MS/s correlator clock a step
        EXPECT_FALSE(e.has_tail);                       // recorded before the board's newer tail
        tow += 10;
        TrFix not_a_fix;
        EXPECT_FALSE(decodeTrFix(frames[i].data(), frames[i].size(), not_a_fix));
    }
}

TEST(SdrRtcmMessages, AnEpochRecordWithItsNewerTailReadsTheSame)
{
    // Since 2026-11-02 the board appends to sub-type 1, after the sample: the
    // ranges its checks left out (6 bits), the satellites it marked (6) and
    // each mark (system 2, PRN 6, flags 2). The fields before stay where they
    // were: such a frame is found, proves the board and decodes the same.
    const auto frames = cut(stream());
    const std::vector<uint8_t>& old_epoch = frames[8];
    TrEpoch before;
    ASSERT_TRUE(decodeTrEpoch(old_epoch.data(), old_epoch.size(), before));
    std::vector<uint8_t> payload(old_epoch.begin() + 3, old_epoch.end() - 3);   // 202 bits in 26 bytes
    size_t pos = 202;
    auto put = [&](uint32_t v, int n) {
        for (int i = n - 1; i >= 0; i--, pos++)
        {
            if (pos / 8 >= payload.size()) payload.push_back(0);
            if ((v >> i) & 1U) payload[pos / 8] |= static_cast<uint8_t>(0x80 >> (pos % 8));
        }
    };
    put(2, 6);    // two ranges left out
    put(36, 6);   // the most marks a 72-byte payload holds
    for (int k = 0; k < 36; k++)
    {
        put(static_cast<uint32_t>(k % 3), 2);
        put(static_cast<uint32_t>(k + 1), 6);
        put(1, 2);
    }
    const std::vector<uint8_t> longer = frameOf(payload);
    ASSERT_EQ(longer.size(), 3u + 72u + 3u);
    TrEpoch after;
    ASSERT_TRUE(decodeTrEpoch(longer.data(), longer.size(), after));
    EXPECT_EQ(after.week, before.week);
    EXPECT_EQ(after.tow_ms, before.tow_ms);
    EXPECT_EQ(after.mode, before.mode);
    EXPECT_EQ(after.nobs, before.nobs);
    std::vector<uint8_t> s = stream();
    s.insert(s.end(), longer.begin(), longer.end());
    Rtcm3Framer f;
    const Collect c = feedIn(f, s, 50);
    ASSERT_EQ(c.frames.size(), kSdrStreamFrames + 1);
    EXPECT_EQ(c.frames.back().bytes, longer);
    EXPECT_EQ(trSubType(longer.data(), longer.size()), kTrEpoch);
}

TEST(SdrRtcmMessages, TheFix)
{
    const auto frames = cut(stream());
    TrFix f;
    ASSERT_TRUE(decodeTrFix(frames[0].data(), frames[0].size(), f));
    EXPECT_EQ(f.week, 2443);
    EXPECT_EQ(f.tow_ms, 156619277u);
    EXPECT_TRUE(f.valid);
    EXPECT_EQ(f.nsat, 18);
    // Its ECEF position, in the field's 0.1 mm steps (the board's
    // rtcm_tr_fix_encode: three signed 38-bit fields from bit 90): the bench
    // antenna at 39.97 N 74.94 W, 6369.3 km from the Earth's centre
    EXPECT_EQ(std::llround(f.pos[0] * 1e4), 12721035908LL);
    EXPECT_EQ(std::llround(f.pos[1] * 1e4), -47265285163LL);
    EXPECT_EQ(std::llround(f.pos[2] * 1e4), 40755638196LL);
    EXPECT_NEAR(std::sqrt(f.pos[0] * f.pos[0] + f.pos[1] * f.pos[1] + f.pos[2] * f.pos[2]), 6369343.750, 0.001);
    // and the rest of the fix: velocity, the clock, PDOP and the residuals
    EXPECT_TRUE(f.vel_valid);
    EXPECT_NEAR(f.vel[0], -0.0040, 1e-9);
    EXPECT_NEAR(f.vel[1], 0.0043, 1e-9);
    EXPECT_NEAR(f.vel[2], 0.0028, 1e-9);
    EXPECT_NEAR(f.clk_bias_m, -92.819, 1e-9);
    EXPECT_NEAR(f.clk_drift_mps, -47.1977, 1e-9);
    EXPECT_NEAR(f.pdop, 1.3, 1e-9);
    EXPECT_NEAR(f.resid_rms_m, 6.14, 1e-9);
    TrEpoch not_an_epoch;
    EXPECT_FALSE(decodeTrEpoch(frames[0].data(), frames[0].size(), not_an_epoch));
    // Cut short, it is refused rather than read past its end.
    std::vector<uint8_t> payload(frames[0].begin() + 3, frames[0].end() - 3);
    payload.resize(20);
    const std::vector<uint8_t> short_fix = frameOf(payload);
    EXPECT_EQ(trSubType(short_fix.data(), short_fix.size()), kTrFix);
    EXPECT_FALSE(decodeTrFix(short_fix.data(), short_fix.size(), f));
}
