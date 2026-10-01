#include "sig_gen.h"

extern "C" {
#include "cmd_queue.h"
#include "corr_model.h"
#include "fe_format.h"
#include "gnss/rx.h"
#include "quant.h"
}

#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <memory>
#include <vector>

TEST(CmdQueue, AppliesAtTheTaggedEpochAndFlagsLateOrDropped)
{
    cmdq_t q;
    cmdq_clear(&q);
    int32_t carr = 0;
    uint64_t code = 0;
    cmdq_push(&q, 3, 30, 300);
    cmdq_push(&q, 4, 40, 400);
    EXPECT_EQ(cmdq_epoch(&q, 2, &carr, &code), 0);  // not yet
    EXPECT_EQ(carr, 0);
    EXPECT_EQ(cmdq_epoch(&q, 3, &carr, &code), 0);  // on time
    EXPECT_EQ(carr, 30);
    EXPECT_EQ(code, 300u);
    EXPECT_EQ(cmdq_epoch(&q, 4, &carr, &code), 0);
    EXPECT_EQ(carr, 40);
    // Late: tagged for 5, first seen at the epoch closing 6.
    cmdq_push(&q, 5, 50, 500);
    EXPECT_EQ(cmdq_epoch(&q, 6, &carr, &code), CORR_DUMP_LATE);
    EXPECT_EQ(carr, 50);
    // Two whose tags have both come: the later one wins, both leave the queue.
    cmdq_push(&q, 7, 70, 700);
    cmdq_push(&q, 8, 80, 800);
    EXPECT_EQ(cmdq_epoch(&q, 8, &carr, &code), 0);
    EXPECT_EQ(carr, 80);
    EXPECT_EQ(cmdq_epoch(&q, 9, &carr, &code), 0);
    EXPECT_EQ(carr, 80);
    // Full: a third command replaces the later-tagged one; the next dump says so.
    cmdq_push(&q, 10, 100, 1000);
    cmdq_push(&q, 11, 110, 1100);
    cmdq_push(&q, 12, 120, 1200);
    EXPECT_EQ(cmdq_epoch(&q, 10, &carr, &code), CORR_DUMP_DROPPED);
    EXPECT_EQ(carr, 100);
    EXPECT_EQ(cmdq_epoch(&q, 11, &carr, &code), 0);
    EXPECT_EQ(carr, 100);  // 11 was replaced
    EXPECT_EQ(cmdq_epoch(&q, 12, &carr, &code), 0);
    EXPECT_EQ(carr, 120);
    // Same tag twice: the second replaces the first.
    cmdq_push(&q, 13, 130, 1300);
    cmdq_push(&q, 13, 131, 1301);
    EXPECT_EQ(cmdq_epoch(&q, 13, &carr, &code), 0);
    EXPECT_EQ(carr, 131);
}

namespace {

constexpr double kFs = 6.75e6, kIf = 1.2e6;

// The receiver core on the golden model, with commands delivered `lat` samples after each tick.
std::vector<corr_dump_t> run(const std::vector<uint8_t> &codes, const std::vector<float> &w, uint64_t lat)
{
    const uint64_t spms = 6750;
    auto rx = std::make_unique<rx_t>();
    rx_cfg_t rc;
    rx_default_cfg(&rc, kFs, kIf);
    rx_init(rx.get(), &rc);
    auto m = std::make_unique<corr_model_t>();
    corr_model_cfg_t mc;
    corr_model_default_cfg(&mc);
    corr_model_init(m.get(), &mc);
    std::vector<float> work(acq_work_floats(2048, rc.acq_ms));
    std::vector<corr_dump_t> all;
    std::vector<corr_cmd_t> held;
    corr_dump_t d[64];
    corr_cmd_t c[64];
    for (uint64_t t0 = 0; t0 + spms <= codes.size(); t0 += spms) {
        int nd = 0;
        uint64_t split = held.empty() ? 0 : lat;
        if (split) {
            nd += corr_model_process(m.get(), t0, codes.data() + t0, split, d, 64);
        }
        for (auto &h : held) {
            corr_model_command(m.get(), &h);
        }
        held.clear();
        nd += corr_model_process(m.get(), t0 + split, codes.data() + t0 + split, spms - split, d + nd, 64 - nd);
        all.insert(all.end(), d, d + nd);
        uint64_t t_now = t0 + spms;
        int nc = rx_tick(rx.get(), t_now, d, nd, c, 64);
        held.insert(held.end(), c, c + nc);
        int ms;
        if (rx_wants_snapshot(rx.get(), t_now, &ms) && t_now >= spms * uint64_t(ms)) {
            uint64_t ts = t_now - spms * uint64_t(ms);
            nc = rx_acquire(rx.get(), t_now, ts, w.data() + 2 * ts, spms * size_t(ms), work.data(), c, 64);
            held.insert(held.end(), c, c + nc);
        }
    }
    return all;
}

}  // namespace

TEST(CmdTags, TrackingIsTheSameWhateverTheP4Latency)
{
    siggen::Sat a, b;
    a.prn = 5;
    a.dop = 2300.0;
    a.code_phase = 100.0;
    a.data_ms = 20;
    b.prn = 21;
    b.dop = -1200.0;
    b.code_phase = 900.5;
    b.data_ms = 20;
    const size_t n = size_t(kFs * 0.8);
    auto x = siggen::make({a, b}, kFs, kIf, n, siggen::sigma_for(45.0, kFs));
    std::vector<uint8_t> codes(n);
    quant2_t q;
    quant2_init(&q, 0.33, 20000.0, 256);
    quant2_apply(&q, x.data(), n, codes.data());
    std::vector<float> w(2 * n);
    for (size_t k = 0; k < n; k++) {
        unsigned cd = codes[k];
        float i = (cd & FE_CODE_I_MAG) ? 3.0f : 1.0f, qq = (cd & FE_CODE_Q_MAG) ? 3.0f : 1.0f;
        w[2 * k] = (cd & FE_CODE_I_SIGN) ? -i : i;
        w[2 * k + 1] = (cd & FE_CODE_Q_SIGN) ? -qq : qq;
    }
    auto d0 = run(codes, w, 0);
    auto d1 = run(codes, w, 2700);  // 400 us
    // Channels are independent: compare each channel's own sequence.
    auto by_ch = [](const corr_dump_t &x, const corr_dump_t &y) { return x.ch != y.ch ? x.ch < y.ch : x.seq < y.seq; };
    std::stable_sort(d0.begin(), d0.end(), by_ch);
    std::stable_sort(d1.begin(), d1.end(), by_ch);
    ASSERT_GT(d0.size(), 1000u);
    ASSERT_EQ(d0.size(), d1.size());
    int late = 0;
    for (size_t k = 0; k < d0.size(); k++) {
        ASSERT_EQ(d0[k].ch, d1[k].ch) << k;
        ASSERT_EQ(d0[k].seq, d1[k].seq) << k;
        ASSERT_EQ(d0[k].t_samp, d1[k].t_samp) << k;
        ASSERT_EQ(d0[k].carr_word, d1[k].carr_word) << k;
        ASSERT_EQ(d0[k].code_word, d1[k].code_word) << k;
        ASSERT_EQ(d0[k].ip, d1[k].ip) << k;
        ASSERT_EQ(d0[k].qp, d1[k].qp) << k;
        late += (d0[k].flags | d1[k].flags) != 0;
    }
    EXPECT_EQ(late, 0);
}
