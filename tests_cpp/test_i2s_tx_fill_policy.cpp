#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>
#include "i2s_tx_fill_policy.h"

// The FC's I2S sender keeps the TX DMA ring full (i2s_tx_fill_policy.h).
//
// Part 1 pins the policy's decisions. Part 2 runs it against a model of the
// ESP-IDF v6.0 TX path (esp_driver_i2s/i2s_common.c) on a DMA that reads
// memory ahead of the EOF that releases each buffer, and measures how far
// ahead of the DMA's read point every frame lands. On the Beetle, frames that
// landed within about a millisecond of it (by the EOF clock) were lost on the
// wire; the old sender put frames there, the policy keeps them a ring away.

using Step = I2sTxFillPolicy::Step;

namespace {

constexpr uint32_t RING_US = 23219;   // 8 x 1024 B at 88200 x 4 B/s

}  // namespace

// ---------------------------------------------------------------------------
// Part 1: the decisions
// ---------------------------------------------------------------------------

TEST(I2sTxFillPolicy, StartsWithARefill)
{
    I2sTxFillPolicy p(RING_US);
    EXPECT_EQ(p.next(true, 1000), Step::Refill);
    EXPECT_EQ(p.next(false, 1000), Step::Refill);
}

TEST(I2sTxFillPolicy, SendsQueuedFramesAndFillsIdleTimeWithZeros)
{
    I2sTxFillPolicy p(RING_US);
    p.wrote(true, 1000);
    EXPECT_EQ(p.next(true, 1010), Step::SendFrame);
    EXPECT_EQ(p.next(false, 1010), Step::IdleFill);
}

TEST(I2sTxFillPolicy, RefillsAfterBeingAwayForMoreThanAQuarterOfTheRing)
{
    I2sTxFillPolicy p(RING_US);
    p.wrote(true, 50000);
    EXPECT_EQ(p.next(true, 50000 + RING_US / 4), Step::SendFrame);
    EXPECT_EQ(p.next(true, 50000 + RING_US / 4 + 1), Step::Refill);
    p.wrote(true, 50000 + RING_US);
    EXPECT_EQ(p.next(true, 50000 + RING_US + 5), Step::SendFrame);
}

TEST(I2sTxFillPolicy, TheAwayTimeIsWrapSafe)
{
    I2sTxFillPolicy p(RING_US);
    p.wrote(true, 0xFFFFF000u);
    EXPECT_EQ(p.next(true, 0x00000100u), Step::SendFrame);   // 0x1100 us later
    EXPECT_EQ(p.next(true, 0xFFFFF000u + RING_US / 2), Step::Refill);
}

TEST(I2sTxFillPolicy, NoChannelBacksOffUntilAWriteGoesThrough)
{
    I2sTxFillPolicy p(RING_US);
    p.wrote(true, 1000);
    p.wrote(false, 2000);                        // the channel is gone
    EXPECT_EQ(p.next(true, 2010), Step::Backoff);
    EXPECT_EQ(p.next(false, 2010), Step::Backoff);
    p.retry();
    EXPECT_EQ(p.next(true, 3010), Step::Refill);  // the ring may be new or empty
    p.wrote(false, 3020);                        // still gone
    EXPECT_EQ(p.next(true, 3030), Step::Backoff);
    p.retry();
    p.wrote(true, 4020);
    EXPECT_EQ(p.next(true, 4030), Step::SendFrame);
}

TEST(I2sTxFillPolicy, ResetStartsAgainWithARefill)
{
    I2sTxFillPolicy p(RING_US);
    p.wrote(true, 1000);
    p.reset();
    EXPECT_EQ(p.next(true, 1010), Step::Refill);
}

// ---------------------------------------------------------------------------
// Part 2: against a model of the IDF TX driver
// ---------------------------------------------------------------------------

namespace {

// Fixed LCG: std:: distributions differ between standard libraries.
struct Lcg
{
    uint32_t s;
    explicit Lcg(uint32_t seed) : s(seed) {}
    uint32_t next() { s = s * 1664525u + 1013904223u; return s; }
    double unit() { return (next() >> 8) * (1.0 / 16777216.0); }
    double uniform(double a, double b) { return a + (b - a) * unit(); }
};

// The TX side of esp_driver_i2s (v6.0) on its DMA ring.
//  - The DMA reads buffer g % N during [g*T, (g+1)*T), byte o at g*T + o*dt,
//    forever, whatever the buffers hold.
//  - The EOF ISR for pass g runs at (g+1)*T + eof_lag: the GDMA has already
//    been reading pass g+1 for that long (its FIFO lead plus ISR latency).
//    It drops the oldest entry if msg_queue is full, clears the buffer
//    (auto_clear) and pushes it.
//  - i2s_channel_write takes a new buffer when the current one is full,
//    there is none, or msg_queue has at most one space left; then copies.
struct TxModel
{
    int    N;
    int    B;
    double dt;        // us per byte
    double T;         // us per buffer
    double eof_lag;

    std::deque<int> q;            // free buffers, oldest first; capacity N-1
    int curr = -1;
    int rw = 0;
    long next_isr = 0;            // first pass whose ISR has not run
    std::vector<double> cleared;  // per buffer: time of its last auto_clear

    TxModel(int n, int b, double bytes_per_us, double lag)
        : N(n), B(b), dt(1.0 / bytes_per_us), T(b / bytes_per_us), eof_lag(lag),
          cleared(n, -1e18) {}

    double isrTime(long g) const { return (double)(g + 1) * T + eof_lag; }

    void runIsrs(double t)
    {
        while (isrTime(next_isr) <= t)
        {
            const int b = (int)(next_isr % N);
            if ((int)q.size() == N - 1) q.pop_front();
            cleared[b] = isrTime(next_isr);
            q.push_back(b);
            ++next_isr;
        }
    }

    // When the DMA reads byte (b, o) next after time tc, and whether the
    // byte is still there by then.
    struct Fate { double read_us; bool wiped; };
    Fate fate(int b, int o, double tc) const
    {
        // the first pass of b that reads offset o at or after the copy
        long g = (long)std::ceil((tc - o * dt) / T);
        if (g < 0) g = 0;
        while ((g % N) != b) ++g;
        // The pass of b before it read offset o before the copy, and its ISR
        // clears the whole buffer: if that runs after the copy, the copy is
        // wiped before the DMA comes round again.
        const long prev = g - N;
        const bool wiped = prev >= 0 && isrTime(prev) > tc;
        return { (double)g * T + o * dt, wiped };
    }

    struct Placement { int b; int o; double tc; };

    // i2s_channel_write: returns the time the call returned; *timed_out set
    // when it ran out of free buffers. copy_delay: the writer preempted
    // between taking the decision and copying (first chunk only).
    double write(double t, int len, double timeout_us, double copy_delay,
                 std::vector<Placement>* where, bool* timed_out)
    {
        *timed_out = false;
        bool first = true;
        while (len > 0)
        {
            runIsrs(t);
            const int spaces = (N - 1) - (int)q.size();
            if (curr < 0 || rw == B || spaces <= 1)
            {
                double tw = t;
                while (q.empty())
                {
                    const double ti = isrTime(next_isr);
                    if (timeout_us >= 0 && ti > t + timeout_us)
                    {
                        *timed_out = true;
                        return t + timeout_us;
                    }
                    tw = ti;
                    runIsrs(tw);
                }
                t = tw;
                curr = q.front();
                q.pop_front();
                rw = 0;
            }
            const int c = std::min(B - rw, len);
            const double tc = t + (first ? copy_delay : 0.0);
            if (where) where->push_back({ curr, rw, tc });
            first = false;
            rw += c;
            len -= c;
            t = tc + 0.2;   // the copy itself
        }
        return t;
    }
};

struct Frame { double ready_us; int len; };

// The Beetle's link at 7680 Hz: a FIFO burst every ~2.19 ms leaves as a
// 10-sample and a 5-sample ISM6 batch, plus BMP585, NonSensor and mag frames.
std::vector<Frame> traffic(double duration_us, Lcg& rng)
{
    std::vector<Frame> f;
    for (double t = 1000; t < duration_us; t += 2192.0)
    {
        const double r = t + rng.uniform(300, 500);   // after the IMU task's drain
        f.push_back({ r, 8 + 1 + 22 * 10 });
        f.push_back({ r + 5, 8 + 1 + 22 * 5 });
    }
    const struct { double hz; int len; } other[] = { {260, 20}, {240, 60}, {96, 24} };
    for (const auto& o : other)
        for (double t = rng.uniform(0, 1e6 / o.hz); t < duration_us; t += 1e6 / o.hz)
            f.push_back({ t + rng.uniform(0, 300), o.len });
    std::sort(f.begin(), f.end(), [](const Frame& a, const Frame& b) { return a.ready_us < b.ready_us; });
    return f;
}

struct Disturbance
{
    double preempt_p = 0.0;      // chance a step starts late ...
    double preempt_max = 0.0;    // ... by up to this much
    double in_write_p = 0.0;     // chance the copy comes late after the decision ...
    double in_write_max = 0.0;   // ... by up to this much
    double stall_at = -1.0;      // one long absence
    double stall_us = 0.0;
};

struct Result
{
    double min_margin = 1e18;    // over every chunk of every frame
    double first_after_stall = 1e18;
    int wiped = 0;
    int frames = 0;
    bool in_order = true;
};

void score(const TxModel& m, const std::vector<TxModel::Placement>& where,
           Result& r, double& last_read)
{
    double first_read = 1e18;
    for (const auto& p : where)
    {
        const TxModel::Fate f = m.fate(p.b, p.o, p.tc);
        if (f.wiped) { r.wiped++; continue; }
        r.min_margin = std::min(r.min_margin, f.read_us - p.tc);
        first_read = std::min(first_read, f.read_us);
    }
    if (first_read < last_read) r.in_order = false;
    last_read = first_read;
    r.frames++;
}

// The sender as main.cpp runs it.
Result runPolicy(TxModel& m, const std::vector<Frame>& frames, const Disturbance& d, Lcg& rng)
{
    I2sTxFillPolicy p((uint32_t)(m.N * m.T));
    const double idle_timeout = m.N * m.T + 2000;
    Result r;
    double t = 0, last_read = -1e18;
    size_t i = 0;
    bool stalled = false;
    bool after_stall = false;
    while (i < frames.size())
    {
        if (d.preempt_p > 0 && rng.unit() < d.preempt_p) t += rng.uniform(0, d.preempt_max);
        if (!stalled && d.stall_at >= 0 && t >= d.stall_at) { t += d.stall_us; stalled = true; after_stall = true; }
        const double cd = (d.in_write_p > 0 && rng.unit() < d.in_write_p) ? rng.uniform(0, d.in_write_max) : 0.0;
        bool timed_out = false;
        switch (p.next(frames[i].ready_us <= t, (uint32_t)t))
        {
            case Step::Refill:
                t = m.write(t, m.N * m.B, 0.0, 0.0, nullptr, &timed_out);
                break;
            case Step::SendFrame:
            {
                std::vector<TxModel::Placement> where;
                t = m.write(t, frames[i].len, 100000.0, cd, &where, &timed_out);
                score(m, where, r, last_read);
                if (after_stall)
                {
                    for (const auto& pl : where)
                        r.first_after_stall = std::min(r.first_after_stall, m.fate(pl.b, pl.o, pl.tc).read_us - pl.tc);
                    after_stall = false;
                }
                ++i;
                break;
            }
            case Step::IdleFill:
                t = m.write(t, (int)I2sTxFillPolicy::IDLE_FILL_BYTES, idle_timeout, cd, nullptr, &timed_out);
                // a frame that came while the write waited goes next
                break;
            case Step::Backoff:
                ADD_FAILURE() << "the model's channel never goes away";
                return r;
        }
        p.wrote(true, (uint32_t)t);
        t += 2.0;   // loop overhead
    }
    return r;
}

// The sender before the fix: a frame if one came within a tick, else 64 B of
// zeros at the tick (xQueueReceive(1 tick) + writeIdleFill(64, 1)).
Result runLegacy(TxModel& m, const std::vector<Frame>& frames, const Disturbance& d, Lcg& rng)
{
    Result r;
    double t = 0, last_read = -1e18;
    size_t i = 0;
    bool stalled = false;
    bool after_stall = false;
    while (i < frames.size())
    {
        if (d.preempt_p > 0 && rng.unit() < d.preempt_p) t += rng.uniform(0, d.preempt_max);
        if (!stalled && d.stall_at >= 0 && t >= d.stall_at) { t += d.stall_us; stalled = true; after_stall = true; }
        const double cd = (d.in_write_p > 0 && rng.unit() < d.in_write_p) ? rng.uniform(0, d.in_write_max) : 0.0;
        bool timed_out = false;
        const double tick = (std::floor(t / 1000.0) + 1.0) * 1000.0;
        if (frames[i].ready_us <= t || frames[i].ready_us < tick)
        {
            t = std::max(t, frames[i].ready_us);
            std::vector<TxModel::Placement> where;
            t = m.write(t, frames[i].len, 100000.0, cd, &where, &timed_out);
            score(m, where, r, last_read);
            if (after_stall)
            {
                for (const auto& pl : where)
                    r.first_after_stall = std::min(r.first_after_stall, m.fate(pl.b, pl.o, pl.tc).read_us - pl.tc);
                after_stall = false;
            }
            ++i;
        }
        else
        {
            t = m.write(tick, 64, 1000.0, cd, nullptr, &timed_out);
        }
        t += 2.0;
    }
    return r;
}

constexpr int    N_DESC = 8;
constexpr int    BUF = 1024;
constexpr double BYTES_PER_US = 88200.0 * 4.0 / 1e6;
constexpr double T_BUF = BUF / BYTES_PER_US;   // ~2902 us

}  // namespace

TEST(I2sTxFillModel, EveryFrameLandsMostOfARingAheadOfTheDma)
{
    // Read-ahead plus ISR latency up to 1.5 ms; the sender preempted often
    // (the IMU task, the GNSS and mag tasks) and sometimes between the
    // driver's decision and its copy.
    for (double lag : { 50.0, 600.0, 1500.0 })
    {
        Lcg rng(0xC0FFEEu + (uint32_t)lag);
        TxModel m(N_DESC, BUF, BYTES_PER_US, lag);
        const auto frames = traffic(5e6, rng);
        Disturbance d;
        d.preempt_p = 0.05;  d.preempt_max = 500;
        d.in_write_p = 0.01; d.in_write_max = 2000;
        const Result r = runPolicy(m, frames, d, rng);
        EXPECT_EQ(r.frames, (int)frames.size());
        EXPECT_EQ(r.wiped, 0) << "lag " << lag;
        EXPECT_TRUE(r.in_order) << "lag " << lag;
        // Each frame goes into the buffer the DMA released last, read again
        // only after the other N-1; the floor allows for filling it, the
        // read-ahead and a 2 ms preemption.
        EXPECT_GT(r.min_margin, (N_DESC - 4) * T_BUF) << "lag " << lag;
    }
}

TEST(I2sTxFillModel, TheOldSenderWritesWithinABufferOfTheReadPoint)
{
    // The same link and disturbances with the old sender: its frames land a
    // buffer or less ahead of the DMA — the margin the Beetle lost frames in.
    Lcg rng(0xC0FFEEu);
    TxModel m(N_DESC, BUF, BYTES_PER_US, 600.0);
    const auto frames = traffic(5e6, rng);
    Disturbance d;
    d.preempt_p = 0.05;  d.preempt_max = 500;
    d.in_write_p = 0.01; d.in_write_max = 2000;
    const Result r = runLegacy(m, frames, d, rng);
    EXPECT_LT(r.min_margin, T_BUF);
}

TEST(I2sTxFillModel, AStallLongerThanTheRingIsRefilledBeforeTheNextFrame)
{
    // 40 ms away (the ring is 23 ms): the DMA has sent everything, so the
    // oldest free buffer is the one it reads next.
    for (double lag : { 50.0, 600.0, 1500.0 })
    {
        Lcg rng(0xBEEFu + (uint32_t)lag);
        TxModel m(N_DESC, BUF, BYTES_PER_US, lag);
        const auto frames = traffic(1e6, rng);
        Disturbance d;
        d.stall_at = 400000; d.stall_us = 40000;
        const Result r = runPolicy(m, frames, d, rng);
        EXPECT_EQ(r.wiped, 0) << "lag " << lag;
        EXPECT_TRUE(r.in_order) << "lag " << lag;
        EXPECT_GT(r.first_after_stall, (N_DESC - 3) * T_BUF) << "lag " << lag;
        EXPECT_GT(r.min_margin, (N_DESC - 4) * T_BUF) << "lag " << lag;
    }
    // Without the refill the first frame after the stall lands at the read point.
    Lcg rng(0xBEEFu);
    TxModel m(N_DESC, BUF, BYTES_PER_US, 600.0);
    const auto frames = traffic(1e6, rng);
    Disturbance d;
    d.stall_at = 400000; d.stall_us = 40000;
    const Result r = runLegacy(m, frames, d, rng);
    EXPECT_LT(r.first_after_stall, T_BUF);
}
