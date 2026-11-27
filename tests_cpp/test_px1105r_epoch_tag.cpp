// #1771: the PX1105R's raw epochs on the FC's clock (TR_GNSSReceiverPX1105R_Serial:
// onFrame, rawE5; Px1105rRawEpoch.h arrivalUs, frameStartUs).
//
// With no 1PPS on the carrier, each 0xE5 is placed by when it arrived: a sample
// of lc86::GnssTimeSync's line, the epoch's GPS instant against an arrival time.
// The driver took the arrival of the frame's last byte, so every epoch carried
// the frame's own wire time -- the frame is 21 bytes plus 31 a signal, 7-15 ms
// at 921600 baud -- and that moved with the signal count: a drop stepped the tag after ~3 s and
// a return left it there for the line's ~64 s window. It now takes the frame's
// first byte, which leaves only the receiver's own delay to the start of its
// output (unmeasured; a stand-in here), the same every epoch.
//
// The wire is simulated: each 50 ms epoch the receiver sends its 0xE5 (with the
// signal count's length) and an 0xDF, back to back, from a fixed delay after the
// epoch. The UART is read as the firmware reads it: the GNSS poll task every
// 25 ms (half the 20 Hz period) plus its own work, each poll draining what the
// UART driver has handed over in reads of up to 512 bytes, the rest counted as
// buffered (pollNewPVT, pump). The driver hands bytes over as its ISR takes
// them from the FIFO: at each 64 (kRxFullThreshold), and the rest once the
// line has been idle 10 byte times (ESP-IDF's default timeout); bytes still in
// the FIFO are neither read nor counted. The real SkyTraq frame parser finds
// each frame's last byte, the real arrivalUs dates it, and the real
// GnssTimeSync places the epochs. Both stamps are run: the frame's last byte
// (before #1771) and its first.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "GnssTimeSync.h"
#include "Px1105rRawEpoch.h"
#include "SkyTraqBinary.h"

namespace {

constexpr double kByteUs = 1.0e7 / 921600.0;   // the driver's byte_us_ at kTargetBaud, 8N1
constexpr double kByteS = kByteUs * 1.0e-6;
constexpr double kDelayS = 0.004;     // the receiver's delay from an epoch to its 0xE5's first byte: a stand-in
constexpr double kPeriodS = 0.05;     // 20 Hz
constexpr double kGps0 = 2400 * 604800.0;   // GPS seconds at the run's start
constexpr size_t kDfBody = 81;        // an 0xDF's body (decodeRcvState's length)
constexpr size_t kReadMax = 512;      // the driver's rx_buf_
constexpr size_t kPollMax = 4096;     // the driver's kMaxBytesPerPoll
constexpr size_t kIsrBlock = 64;      // the driver's kRxFullThreshold
constexpr double kIdleS = 10 * kByteS;   // ESP-IDF's RX timeout: 10 byte times

size_t e5Body(int signals) { return 14 + 31 * static_cast<size_t>(signals); }
double wireS(int signals) { return static_cast<double>(e5Body(signals) + 7) * kByteS; }   // the whole frame, 21 + 31n bytes

// The FC's clock: 12 ppm fast, 7 s ahead of the run's start.
int64_t mcuUs(double t) { return std::llround(t * (1.0 + 12.0e-6) * 1.0e6) + 7000000; }

struct Tag
{
    double t;        // the epoch's true time (s into the run)
    double err_ms;   // where the line put it on the FC's clock, less where it is
};

// Signals tracked: 40 (the open sky lists 36-45), 20 from 80 s to 90 s.
int signalsAt(double t) { return (t >= 80.0 && t < 90.0) ? 20 : 40; }

class Wire : public skytraq::FrameSink
{
public:
    explicit Wire(bool first_byte) : first_byte_(first_byte) {}

    std::vector<Tag> run(double t_end)
    {
        std::mt19937 rng(1771);
        std::uniform_real_distribution<double> work(0.0, 0.0015);   // the poll task's own time each pass
        int next_k = 0;
        for (double r = 0.0123; r <= t_end; r += 0.025 + work(rng))
        {
            // the bursts begun by now: the 0xE5, then the 0xDF, back to back from t_k + kDelayS; each byte handed
            // over when its 64-byte block is complete, or 10 byte times after the burst's last byte
            while (next_k * kPeriodS + kDelayS <= r)
            {
                const double t_k = next_k * kPeriodS;
                std::vector<uint8_t> burst;
                appendFrame(burst, e5BodyAt(next_k, signalsAt(t_k)));
                appendFrame(burst, std::vector<uint8_t>(kDfBody, 0x00), skytraq::MSG_RCV_STATE);
                const double start = t_k + kDelayS;
                const size_t n = burst.size();
                for (size_t j = 0; j < n; ++j)
                {
                    const size_t block_end = std::min(n, (j / kIsrBlock + 1) * kIsrBlock);
                    const double handed = block_end % kIsrBlock == 0 && block_end <= n - (n % kIsrBlock)
                                              ? start + static_cast<double>(block_end) * kByteS
                                              : start + static_cast<double>(n) * kByteS + kIdleS;
                    pending_.push_back({burst[j], handed});
                }
                next_k++;
            }
            // pollNewPVT: pump() until nothing is handed over, up to 4 kB
            size_t visible = 0;
            while (visible < pending_.size() && pending_[visible].handed <= r) visible++;
            size_t read = 0, pumps = 0;
            while (read < visible && read < kPollMax)
            {
                const size_t n = std::min({kReadMax, visible - read, kPollMax - read});
                std::vector<uint8_t> buf(n);
                for (size_t i = 0; i < n; ++i) buf[i] = pending_[read + i].b;
                rd_us_ = mcuUs(r + 20.0e-6 * static_cast<double>(++pumps));   // nowUs() after each read
                rd_n_ = n;
                behind_ = visible - read - n;   // uart_get_buffered_data_len: handed over, not yet read
                parser_.feed(buf.data(), n, *this);
                read += n;
            }
            pending_.erase(pending_.begin(), pending_.begin() + static_cast<long>(read));
        }
        return tags_;
    }

    void onFrame(const uint8_t* body, size_t len, size_t index) override
    {
        if (body[0] != skytraq::MSG_EXT_RAW) return;
        const uint32_t tow_ms = (uint32_t(body[5]) << 24) | (uint32_t(body[6]) << 16) | (uint32_t(body[7]) << 8) |
                                uint32_t(body[8]);
        epoch_t_ = tow_ms * 1.0e-3;
        epoch_gps_ = kGps0 + epoch_t_;
        const int64_t end_us = px1105r::arrivalUs(rd_us_, rd_n_, index, behind_, kByteUs);
        const int64_t stamp = first_byte_ ? px1105r::frameStartUs(end_us, len, kByteUs) : end_us;
        sync_.addSample(epoch_gps_, stamp);
        int64_t mcu = 0;
        if (sync_.valid() && sync_.mcuAt(epoch_gps_, mcu))
            tags_.push_back({epoch_t_, static_cast<double>(mcu - mcuUs(epoch_t_)) * 1.0e-3});
    }

private:
    static std::vector<uint8_t> e5BodyAt(int k, int signals)
    {
        std::vector<uint8_t> b(e5Body(signals), 0x00);
        b[0] = skytraq::MSG_EXT_RAW;
        b[1] = 1;
        const uint32_t tow_ms = static_cast<uint32_t>(k) * 50u;
        b[5] = static_cast<uint8_t>(tow_ms >> 24);
        b[6] = static_cast<uint8_t>(tow_ms >> 16);
        b[7] = static_cast<uint8_t>(tow_ms >> 8);
        b[8] = static_cast<uint8_t>(tow_ms);
        return b;
    }
    static void appendFrame(std::vector<uint8_t>& out, std::vector<uint8_t> body, uint8_t id = 0)
    {
        if (id != 0) body[0] = id;
        std::vector<uint8_t> f(body.size() + 7);
        const size_t n = skytraq::buildFrame(body.data(), body.size(), f.data(), f.size());
        ASSERT_EQ(n, body.size() + 7);
        out.insert(out.end(), f.begin(), f.end());
    }

    struct Byte
    {
        uint8_t b;
        double handed;   // when the UART driver handed it over (true time)
    };

    bool first_byte_;
    std::vector<Byte> pending_;
    skytraq::FrameParser parser_;
    lc86::GnssTimeSync sync_;
    int64_t rd_us_ = 0;
    size_t rd_n_ = 0, behind_ = 0;
    double epoch_t_ = 0.0, epoch_gps_ = 0.0;
    std::vector<Tag> tags_;
};

// The line fits the clock's drift (12 ppm here) over its first ~40 s; until
// then both stamps share the same start-up error, so the comparisons start at
// kSettledS.
constexpr double kSettledS = 40.0;

// The tag error at time t (the epoch nearest it).
double errAt(const std::vector<Tag>& tags, double t)
{
    const Tag* best = nullptr;
    for (const Tag& g : tags)
        if (best == nullptr || std::fabs(g.t - t) < std::fabs(best->t - t)) best = &g;
    return best != nullptr ? best->err_ms : NAN;
}

TEST(Px1105rEpochTag, FrameStartIsTheFramesFirstByte)
{
    // a frame is its body and 7 bytes more; its first byte arrived len + 6 byte times before its last
    for (size_t len : {e5Body(20), e5Body(40), kDfBody})
    {
        std::vector<uint8_t> body(len, 0x11), frame(len + 16);
        body[0] = skytraq::MSG_EXT_RAW;
        EXPECT_EQ(skytraq::buildFrame(body.data(), len, frame.data(), frame.size()), len + 7);
        const int64_t end = 50000000;
        EXPECT_EQ(px1105r::frameStartUs(end, len, kByteUs),
                  end - static_cast<int64_t>(static_cast<double>(len + 6) * kByteUs)) << len;
        EXPECT_EQ(px1105r::frameStartUs(end, len, 0.0), end) << "byte_us 0: the stamp as given";
    }
    // 40 signals: 13.67 ms of frame before its last byte, 20 signals 6.94 ms; never rounded up (early)
    EXPECT_EQ(50000000 - px1105r::frameStartUs(50000000, e5Body(40), kByteUs), 13671);
    EXPECT_EQ(50000000 - px1105r::frameStartUs(50000000, e5Body(20), kByteUs), 6944);
}

TEST(Px1105rEpochTag, ArrivalIsTheReadLessTheBytesAfterIt)
{
    // the driver's rule, as its member had it: the read's time less the bytes after
    // this one in the read and those still buffered
    const int64_t rd = 9000000;
    EXPECT_EQ(px1105r::arrivalUs(rd, 100, 99, 0, kByteUs), rd);
    EXPECT_EQ(px1105r::arrivalUs(rd, 100, 49, 0, kByteUs), rd - static_cast<int64_t>(50 * kByteUs));
    EXPECT_EQ(px1105r::arrivalUs(rd, 100, 49, 200, kByteUs), rd - static_cast<int64_t>(250 * kByteUs));
}

// Through the real parser, read while the receiver is still sending (every
// byte after the frame's end already in the read): the first byte's arrival
// comes out within 2 us of the truth (the read's clock, arrivalUs and
// frameStartUs each truncate to whole microseconds).
TEST(Px1105rEpochTag, ReadMidStreamTheFirstByteIsExact)
{
    for (int signals : {20, 40, 45})
    {
        std::vector<uint8_t> body(e5Body(signals), 0x00), stream(body.size() + 7 + 50, 0x55);
        body[0] = skytraq::MSG_EXT_RAW;
        ASSERT_EQ(skytraq::buildFrame(body.data(), body.size(), stream.data(), stream.size()), body.size() + 7);
        const double start_us = 1.0e6;   // the A0 starts here; byte j has arrived at start + (j + 1) byte times
        const size_t m = stream.size() - 1;   // read just as its last byte arrived
        struct Sink : skytraq::FrameSink
        {
            int64_t rd_us = 0;
            size_t rd_n = 0, index = 0, len = 0;
            void onFrame(const uint8_t*, size_t l, size_t i) override { index = i; len = l; }
        } sink;
        sink.rd_n = stream.size();
        sink.rd_us = static_cast<int64_t>(start_us + static_cast<double>(m + 1) * kByteUs);
        skytraq::FrameParser parser;
        parser.feed(stream.data(), stream.size(), sink);
        ASSERT_EQ(sink.len, body.size());
        ASSERT_EQ(sink.index, body.size() + 6);   // its 0A
        const int64_t end = px1105r::arrivalUs(sink.rd_us, sink.rd_n, sink.index, 0, kByteUs);
        EXPECT_NEAR(static_cast<double>(px1105r::frameStartUs(end, sink.len, kByteUs)), start_us + kByteUs, 2.0)
            << signals;
    }
}

TEST(Px1105rEpochTag, TheFirstByteTagsEveryEpochAtTheReceiversOwnDelay)
{
    const std::vector<Tag> tags = Wire(true).run(170.0);
    ASSERT_FALSE(tags.empty());
    const double want = kDelayS * 1e3 + kByteUs * 1e-3;   // the delay, and the first byte's own time
    double lo = 1e9, hi = -1e9, lo40 = 1e9, lo20 = 1e9;
    for (const Tag& g : tags)
        if (g.t >= kSettledS)
        {
            lo = std::min(lo, g.err_ms);
            hi = std::max(hi, g.err_ms);
            (signalsAt(g.t) == 20 ? lo20 : lo40) = std::min(signalsAt(g.t) == 20 ? lo20 : lo40, g.err_ms);
        }
    std::printf("first byte: tags %.3f-%.3f ms late (the receiver's delay %.1f ms), 40 -> 20 -> 40 signals\n", lo, hi,
                kDelayS * 1e3);
    // late only (the reads' lateness, never early), and the same with 20 signals as with 40
    EXPECT_GE(lo, want - 0.02);
    EXPECT_LE(hi, want + 0.3);
    EXPECT_LT(std::fabs(lo40 - lo20), 0.1);
}

TEST(Px1105rEpochTag, BeforeTheFixTheLastByteTaggedTheFrameAndMovedWithTheSignals)
{
    const std::vector<Tag> tags = Wire(false).run(170.0);
    const double d = kDelayS * 1e3;
    const double w40 = wireS(40) * 1e3, w20 = wireS(20) * 1e3;
    const double at79 = errAt(tags, 79.0), at89 = errAt(tags, 89.0), at120 = errAt(tags, 120.0),
                 at165 = errAt(tags, 165.0);
    std::printf("last byte: %.2f ms late at 40 signals (79 s), %.2f 9 s into the 20 (89 s), still %.2f 30 s after "
                "40 are back (120 s), %.2f at 165 s\n", at79, at89, at120, at165);
    // the frame on top of the delay, and the reads' own lateness (0.2-0.4 ms here, itself moving with the length)
    EXPECT_NEAR(at79, d + w40, 0.5);    // 13.7 ms of frame
    EXPECT_NEAR(at89, d + w20, 0.5);    // 7.0 ms: the shorter frames' step
    EXPECT_GT(at79 - at89, 6.0);
    EXPECT_NEAR(at120, at89, 0.3);      // held by the line's window after the signals returned
    EXPECT_NEAR(at165, at79, 0.3);
}

}  // namespace
