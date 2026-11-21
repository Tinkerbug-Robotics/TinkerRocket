// Host tests for the gnss-sdr-dev board's epochs as the raw-GNSS filter takes
// them (TR_GNSSReceiverSdr_Serial/SdrRawEpoch.h): each epoch made whole by
// its 4001, the board's clock steps put back into every range and the time of
// week, its marks obeyed, one signal a system kept, the fused grid, and what a
// lost or lone record does. On the board's recorded stream
// (fixtures/sdr_rtcm_stream.h), with records re-made where a test needs
// another step sum or marks.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "Rtcm3Msm.h"
#include "SdrRawEpoch.h"
#include "fixtures/sdr_rtcm_stream.h"

using namespace sdr_rtcm;

namespace {

std::vector<std::vector<uint8_t>> frames()
{
    std::vector<std::vector<uint8_t>> out;
    size_t i = 0;
    while (i + 6 <= sizeof(kSdrStream))
    {
        const size_t len = 6 + (((kSdrStream[i + 1] & 3) << 8) | kSdrStream[i + 2]);
        out.emplace_back(kSdrStream + i, kSdrStream + i + len);
        i += len;
    }
    return out;
}

// A 4001 sub-type 1 written to its layout (receiver/core/nav/rtcm_msm.c
// rtcm_tr_epoch_encode), with the tail when it has marks.
std::vector<uint8_t> record(const TrEpoch& e, bool tail)
{
    std::vector<uint8_t> m(80, 0);
    size_t pos = 0;
    auto put = [&](uint64_t v, int n) {
        for (int i = n - 1; i >= 0; i--, pos++)
            if ((v >> i) & 1U) m[pos / 8] |= static_cast<uint8_t>(0x80 >> (pos % 8));
    };
    put(kTrMessage, 12);
    put(kTrEpoch, 4);
    put(e.week, 16);
    put(e.tow_ms, 30);
    put(static_cast<uint64_t>(e.shift_ns) & 0xFFFFF, 20);
    put(e.clk_steps, 16);
    put(static_cast<uint64_t>(std::llround(e.clk_step_m * 1000.0)) & 0xFFFFFFFFFFFFull, 48);
    put(e.mode, 2);
    put(e.nobs, 6);
    put(e.sample & 0xFFFFFFFFFFFFull, 48);
    if (tail)
    {
        put(e.ndrop, 6);
        put(e.nmark, 6);
        for (int k = 0; k < e.nmark; k++)
        {
            put(e.mark[k].sys, 2);
            put(e.mark[k].prn, 6);
            put(e.mark[k].flags, 2);
        }
    }
    m.resize((pos + 7) / 8);
    std::vector<uint8_t> f = {0xD3, static_cast<uint8_t>(m.size() >> 8), static_cast<uint8_t>(m.size())};
    f.insert(f.end(), m.begin(), m.end());
    const uint32_t crc = rtcm3::crc24q(f.data(), f.size());
    f.push_back(static_cast<uint8_t>(crc >> 16));
    f.push_back(static_cast<uint8_t>(crc >> 8));
    f.push_back(static_cast<uint8_t>(crc));
    return f;
}

struct Assembled
{
    std::vector<SdrEpoch> epochs;
    SdrEpochAssembler::Counts counts;
};

Assembled feed(const std::vector<std::vector<uint8_t>>& fs, const SdrEpochAssembler::Config& cfg)
{
    SdrEpochAssembler a(cfg);
    Assembled r;
    int64_t t = 1000;
    for (const auto& f : fs)
    {
        SdrEpoch e;
        t += 500;
        if (a.onFrame(f.data(), f.size(), t, e)) r.epochs.push_back(e);
    }
    r.counts = a.counts();
    return r;
}

constexpr double kSum = 2591783.819;   // the fixture's step sum (18 steps)

}  // namespace

TEST(SdrRawEpoch, EveryEpochIsMadeWholeByItsRecord)
{
    const Assembled r = feed(frames(), SdrEpochAssembler::Config());
    ASSERT_EQ(r.epochs.size(), kSdrStreamEpochs);
    EXPECT_EQ(r.counts.epochs, kSdrStreamEpochs);
    EXPECT_EQ(r.counts.msm, 3 * kSdrStreamEpochs);
    EXPECT_EQ(r.counts.no_record, 0u);
    EXPECT_EQ(r.counts.lone_record, 0u);
    EXPECT_EQ(r.counts.short_epochs, 0u);
    EXPECT_EQ(r.counts.other_signal, 0u);
    uint32_t tow = kSdrStreamFirstEpochTow;
    for (const SdrEpoch& e : r.epochs)
    {
        EXPECT_EQ(e.tow_ms, tow);
        EXPECT_EQ(e.week, 2443);
        EXPECT_DOUBLE_EQ(e.gps_s, 2443 * 604800.0 + tow * 1.0e-3);
        EXPECT_EQ(e.nobs, 18);
        EXPECT_EQ(e.mode, 1);
        ASSERT_EQ(e.epoch.n, 18);
        int n[3] = {};
        for (int i = 0; i < e.epoch.n; i++) n[e.epoch.obs[i].sys]++;
        EXPECT_EQ(n[GNSS_SYS_GPS], 5);
        EXPECT_EQ(n[GNSS_SYS_GAL], 6);
        EXPECT_EQ(n[GNSS_SYS_BDS], 7);   // the B1C pilot, MSM signal 31
        tow += 10;
    }
}

TEST(SdrRawEpoch, TheClockStepsGoBackIntoEveryRangeAndTheTimeOfWeek)
{
    SdrEpochAssembler::Config raw;
    raw.restore_steps = false;
    const Assembled as_sent = feed(frames(), raw);
    const Assembled fixed = feed(frames(), SdrEpochAssembler::Config());
    ASSERT_EQ(as_sent.epochs.size(), fixed.epochs.size());
    for (size_t k = 0; k < fixed.epochs.size(); k++)
    {
        const SdrEpoch& a = as_sent.epochs[k];
        const SdrEpoch& b = fixed.epochs[k];
        EXPECT_DOUBLE_EQ(a.step_m, 0.0);
        EXPECT_NEAR(b.step_m, kSum, 1e-6);
        EXPECT_NEAR(b.epoch.tow_s - a.epoch.tow_s, kSum / kC, 1e-10);
        EXPECT_DOUBLE_EQ(b.gps_s, a.gps_s);   // the GPS instant to place is the tag either way
        for (int i = 0; i < b.epoch.n; i++)
        {
            EXPECT_NEAR(b.epoch.obs[i].pr_m - a.epoch.obs[i].pr_m, kSum, 1e-6);
            if (b.epoch.obs[i].has_phase)
                EXPECT_NEAR(b.epoch.obs[i].phase_m - a.epoch.obs[i].phase_m, kSum, 1e-6);
            EXPECT_DOUBLE_EQ(b.epoch.obs[i].rate_mps, a.epoch.obs[i].rate_mps);   // a step moves no rate
            // each satellite's transmit time is the tag's: tow_s - pr / c the same
            EXPECT_NEAR(b.epoch.tow_s - b.epoch.obs[i].pr_m / kC, a.epoch.tow_s - a.epoch.obs[i].pr_m / kC, 1e-10);
        }
    }
}

TEST(SdrRawEpoch, AStepBetweenTwoEpochsLeavesTheRangesContinuous)
{
    // The second epoch's record says the clock stepped by -332.857 m (the V10
    // log's first): whatever its MSMs carried, its ranges come out moved by
    // the new sum, so the board's own jump of +332.857 m cancels.
    auto fs = frames();
    TrEpoch rec;
    ASSERT_TRUE(decodeTrEpoch(fs[12].data(), fs[12].size(), rec));
    rec.clk_steps++;
    rec.clk_step_m -= 332.857;
    fs[12] = record(rec, false);
    const Assembled r = feed(fs, SdrEpochAssembler::Config());
    ASSERT_EQ(r.epochs.size(), kSdrStreamEpochs);
    EXPECT_NEAR(r.epochs[1].step_m - r.epochs[0].step_m, -332.857, 1e-6);
    EXPECT_NEAR(r.epochs[2].step_m, kSum, 1e-6);
}

TEST(SdrRawEpoch, TheBoardsMarksLeaveOutWhatTheyCover)
{
    auto fs = frames();
    TrEpoch rec;
    ASSERT_TRUE(decodeTrEpoch(fs[8].data(), fs[8].size(), rec));
    // the first epoch's GPS 16 (left out of the fix) and BeiDou 20 (a new channel)
    rec.nmark = 2;
    rec.mark[0] = TrMark{GNSS_SYS_GPS, 16, kMarkFix};
    rec.mark[1] = TrMark{GNSS_SYS_BDS, 20, kMarkNew};
    rec.ndrop = 1;
    fs[8] = record(rec, true);
    TrEpoch back;
    ASSERT_TRUE(decodeTrEpoch(fs[8].data(), fs[8].size(), back));
    EXPECT_TRUE(back.has_tail);
    EXPECT_EQ(back.ndrop, 1);
    ASSERT_EQ(back.nmark, 2);
    EXPECT_EQ(back.mark[1].sys, GNSS_SYS_BDS);
    EXPECT_EQ(back.mark[1].prn, 20);
    EXPECT_EQ(back.mark[1].flags, kMarkNew);

    auto find = [](const SdrEpoch& e, uint8_t sys, uint8_t prn) -> const gnss_raw::RawObs* {
        for (int i = 0; i < e.epoch.n; i++)
            if (e.epoch.obs[i].sys == sys && e.epoch.obs[i].prn == prn) return &e.epoch.obs[i];
        return nullptr;
    };
    const Assembled r = feed(fs, SdrEpochAssembler::Config());
    ASSERT_FALSE(r.epochs.empty());
    const gnss_raw::RawObs* g16 = find(r.epochs[0], GNSS_SYS_GPS, 16);
    const gnss_raw::RawObs* c20 = find(r.epochs[0], GNSS_SYS_BDS, 20);
    ASSERT_TRUE(g16 && c20);
    EXPECT_FALSE(g16->has_pr);
    EXPECT_FALSE(g16->has_phase);
    EXPECT_TRUE(g16->has_rate);       // its Doppler stays: the fix's test was on the range
    EXPECT_FALSE(c20->has_pr);
    EXPECT_FALSE(c20->has_rate);      // a channel not settled: nothing of it
    EXPECT_EQ(r.counts.marked_pr, 2u);
    EXPECT_EQ(r.counts.marked_rate, 1u);
    EXPECT_EQ(r.epochs[0].ndrop, 1);
    // the next epoch is unmarked
    EXPECT_TRUE(find(r.epochs[1], GNSS_SYS_GPS, 16)->has_pr);

    SdrEpochAssembler::Config keep;
    keep.drop_fix_marked = false;
    keep.drop_new_marked = false;
    const Assembled k = feed(fs, keep);
    EXPECT_TRUE(find(k.epochs[0], GNSS_SYS_GPS, 16)->has_pr);
    EXPECT_TRUE(find(k.epochs[0], GNSS_SYS_BDS, 20)->has_rate);
    EXPECT_EQ(k.counts.marked_pr, 0u);
}

TEST(SdrRawEpoch, TheFusedGridTakesTheFirstEpochOfEachPeriod)
{
    // The board's epochs sit at their own millisecond phase (the V10 log's end
    // in 7 ms): the grid is each period's first, not a whole multiple
    SdrEpochAssembler::Config c;
    c.fuse_period_ms = 100;
    const Assembled r = feed(frames(), c);
    ASSERT_EQ(r.epochs.size(), 2u);   // 156619330 opens one period, 156619400 the next
    EXPECT_EQ(r.epochs[0].tow_ms, kSdrStreamFirstEpochTow);
    EXPECT_EQ(r.epochs[1].tow_ms, 156619400u);
    EXPECT_EQ(r.counts.off_grid, kSdrStreamEpochs - 2);
    c.fuse_period_ms = 20;   // 330 alone in its period, then 340/350, 360/370, 380/390, 400/410, 420
    EXPECT_EQ(feed(frames(), c).epochs.size(), 6u);
}

TEST(SdrRawEpoch, ALostRecordCostsOnlyItsEpoch)
{
    auto fs = frames();
    fs.erase(fs.begin() + 12);   // the second epoch's 4001
    const Assembled r = feed(fs, SdrEpochAssembler::Config());
    EXPECT_EQ(r.epochs.size(), kSdrStreamEpochs - 1);
    EXPECT_EQ(r.counts.no_record, 1u);
    EXPECT_EQ(r.epochs[1].tow_ms, kSdrStreamFirstEpochTow + 20);
    // and a record with no MSMs of its time is counted, not made into an epoch
    auto lone = frames();
    lone.erase(lone.begin() + 9, lone.begin() + 12);   // the second epoch's three MSMs
    const Assembled l = feed(lone, SdrEpochAssembler::Config());
    EXPECT_EQ(l.epochs.size(), kSdrStreamEpochs - 1);
    EXPECT_EQ(l.counts.lone_record, 1u);
}

TEST(SdrRawEpoch, OtherMessagesPassThrough)
{
    // the fix and the ephemerides make no epoch and leave a pending one alone
    auto fs = frames();
    std::vector<std::vector<uint8_t>> interleaved(fs.begin() + 5, fs.begin() + 8);   // an epoch's MSMs
    interleaved.push_back(fs[0]);                                                  // the fix
    interleaved.push_back(fs[1]);                                                  // an ephemeris
    interleaved.push_back(fs[8]);                                                  // its record
    const Assembled r = feed(interleaved, SdrEpochAssembler::Config());
    ASSERT_EQ(r.epochs.size(), 1u);
    EXPECT_EQ(r.epochs[0].epoch.n, 18);
}
