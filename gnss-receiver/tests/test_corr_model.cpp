#include "sig_gen.h"

extern "C" {
#include "corr_float.h"
#include "corr_model.h"
#include "fe_format.h"
#include "quant.h"
}

#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <memory>
#include <vector>

namespace {

constexpr double kFs = 6.75e6, kIf = 1.2e6;
constexpr double kOne = double(uint64_t(1) << CORR_CODE_FRAC_BITS);

corr_cmd_t start(int ch, int prn, double code_phase, double dop)
{
    corr_cmd_t c{};
    c.type = CORR_CMD_START;
    c.ch = uint8_t(ch);
    c.sig = GNSS_SIG_GPS_L1CA;
    c.prn = uint8_t(prn);
    c.code_phase = uint64_t(std::llround(code_phase * kOne));
    c.tap_offset = uint64_t(0.25 * kOne);
    c.carr_word = int32_t(std::llround((kIf + dop) / kFs * 4294967296.0));
    c.code_word = uint64_t(std::llround(1.023e6 * (1.0 + dop / GNSS_FREQ_L1_HZ) / kFs * kOne));
    return c;
}

// A synthetic satellite through the 2-bit quantizer: codes for the model, weights for the float correlator.
void two_bit(const std::vector<float> &x, std::vector<uint8_t> &codes, std::vector<float> &w)
{
    size_t n = x.size() / 2;
    codes.resize(n);
    w.resize(2 * n);
    quant2_t q;
    quant2_init(&q, 0.33, 20000.0, 256);
    quant2_apply(&q, x.data(), n, codes.data());
    for (size_t k = 0; k < n; k++) {
        unsigned c = codes[k];
        float i = (c & FE_CODE_I_MAG) ? 3.0f : 1.0f, qv = (c & FE_CODE_Q_MAG) ? 3.0f : 1.0f;
        w[2 * k] = (c & FE_CODE_I_SIGN) ? -i : i;
        w[2 * k + 1] = (c & FE_CODE_Q_SIGN) ? -qv : qv;
    }
}

}  // namespace

TEST(CorrModel, DefaultTableIsTheHeaders)
{
    corr_model_cfg_t a, b;
    corr_model_default_cfg(&a);
    ASSERT_EQ(corr_model_lut(&b, 3, 2.2), 0);  // the GP2021-style table, regenerated
    EXPECT_EQ(a.lut_bits, 3);
    for (int k = 0; k < 8; k++) {
        EXPECT_EQ(a.cos_lut[k], b.cos_lut[k]) << k;
        EXPECT_EQ(a.sin_lut[k], b.sin_lut[k]) << k;
    }
    EXPECT_EQ(a.acc_bits, CORR_ACC_BITS);
}

TEST(CorrModel, HandComputedDump)
{
    // Two samples of code 0 (I +1, Q +1), carrier phase held in sector 0 (cos 2, sin 1): each
    // sample mixes to (2 + 1, 2 - 1) = (3, 1). The code starts at chip 1022.5 and steps 0.25
    // chip, so both samples see chip 1022 and the epoch falls at sample 2.
    auto m = std::make_unique<corr_model_t>();
    corr_model_cfg_t cfg;
    corr_model_default_cfg(&cfg);
    corr_model_init(m.get(), &cfg);
    corr_cmd_t c{};
    c.type = CORR_CMD_START;
    c.ch = 5;
    c.sig = GNSS_SIG_GPS_L1CA;
    c.prn = 1;
    c.code_phase = uint64_t(1022.5 * kOne);
    c.tap_offset = uint64_t(0.25 * kOne);
    c.carr_word = 0;
    c.code_word = uint64_t(0.25 * kOne);
    ASSERT_EQ(corr_model_command(m.get(), &c), 0);
    uint8_t codes[4] = {0, 0, 0, 0};
    corr_dump_t d[2];
    int nd = corr_model_process(m.get(), 100, codes, 4, d, 2);
    // START at t_start 0 is in the past for a block at 100: the channel starts with the block.
    ASSERT_EQ(nd, 1);
    uint8_t chips[GPS_CA_LEN];
    gps_ca_code(1, chips);
    int p = chips[1022] ? -1 : 1;
    // Early at +0.25 chip: samples at 1022.75 and 1023.0 -> chips 1022 and 0 (wrapped).
    int e0 = chips[1022] ? -1 : 1, e1 = chips[0] ? -1 : 1;
    // Late at -0.25 chip: 1022.25 and 1022.5 -> chip 1022 twice.
    EXPECT_EQ(d[0].t_samp, 102u);
    EXPECT_EQ(d[0].seq, 0u);
    EXPECT_EQ(d[0].ip, float(2 * 3 * p));
    EXPECT_EQ(d[0].qp, float(2 * 1 * p));
    EXPECT_EQ(d[0].ie, float(3 * (e0 + e1)));
    EXPECT_EQ(d[0].qe, float(1 * (e0 + e1)));
    EXPECT_EQ(d[0].il, float(2 * 3 * p));
    EXPECT_EQ(d[0].code_phase, 0u);
    EXPECT_EQ(d[0].carr_phase, 0u);
}

TEST(CorrModel, SameNcoAndTimingAsTheFloatCorrelator)
{
    siggen::Sat s;
    s.prn = 17;
    s.dop = -2345.0;
    s.code_phase = 777.7;
    s.phase = 0.4;
    s.data_ms = 20;
    const size_t n = size_t(kFs * 0.05);
    auto x = siggen::make({s}, kFs, kIf, n, siggen::sigma_for(45.0, kFs));
    std::vector<uint8_t> codes;
    std::vector<float> w;
    two_bit(x, codes, w);

    auto m = std::make_unique<corr_model_t>();
    corr_model_cfg_t cfg;
    corr_model_default_cfg(&cfg);
    corr_model_init(m.get(), &cfg);
    auto f = std::make_unique<corr_float_t>();
    corr_float_init(f.get(), kFs);
    corr_cmd_t st = start(2, 17, 777.7, -2345.0);
    corr_model_command(m.get(), &st);
    corr_float_command(f.get(), &st);

    const size_t spms = 6750;
    double ratio_sum = 0.0;
    int nratio = 0;
    for (size_t t0 = 0; t0 + spms <= n; t0 += spms) {
        corr_dump_t dm[4], df[4];
        int nm = corr_model_process(m.get(), t0, codes.data() + t0, spms, dm, 4);
        int nf = corr_float_process(f.get(), t0, w.data() + 2 * t0, spms, df, 4);
        ASSERT_EQ(nm, nf);
        for (int k = 0; k < nm; k++) {
            EXPECT_EQ(dm[k].seq, df[k].seq);
            EXPECT_EQ(dm[k].t_samp, df[k].t_samp);
            EXPECT_EQ(dm[k].code_phase, df[k].code_phase);
            EXPECT_EQ(dm[k].carr_phase, df[k].carr_phase);
            EXPECT_EQ(dm[k].carr_cycles, df[k].carr_cycles);
            EXPECT_EQ(dm[k].carr_word, df[k].carr_word);
            if (dm[k].seq > 0) {
                // Integer prompt vs float prompt: same phase, magnitude scaled by the table's gain.
                std::complex<double> pm(dm[k].ip, dm[k].qp), pf(df[k].ip, df[k].qp);
                EXPECT_NEAR(std::remainder(std::arg(pm) - std::arg(pf), 2 * M_PI), 0.0, 0.15);
                ratio_sum += std::abs(pm) / std::abs(pf);
                nratio++;
            }
        }
        // Change the NCO now and then: both latch it at the same epoch.
        if (t0 == 10 * spms) {
            corr_cmd_t c{};
            c.type = CORR_CMD_NCO;
            c.ch = 2;
            c.carr_word = st.carr_word + 1000;
            c.code_word = st.code_word + 12345;
            corr_model_command(m.get(), &c);
            corr_float_command(f.get(), &c);
        }
    }
    // The 8-sector table's fundamental is ~2.3 times the unit sine (levels 1 and 2).
    EXPECT_NEAR(ratio_sum / nratio, 2.3, 0.2);
    EXPECT_EQ(m->acc_overflows, 0u);
}

TEST(CorrModel, FlagsAccumulatorOverflow)
{
    auto m = std::make_unique<corr_model_t>();
    corr_model_cfg_t cfg;
    corr_model_default_cfg(&cfg);
    cfg.acc_bits = 8;  // +-128: a strong signal overflows within a period
    corr_model_init(m.get(), &cfg);
    siggen::Sat s;
    s.prn = 3;
    const size_t n = size_t(kFs * 0.003);
    auto x = siggen::make({s}, kFs, kIf, n, 0.0);
    std::vector<uint8_t> codes;
    std::vector<float> w;
    two_bit(x, codes, w);
    corr_cmd_t st = start(0, 3, 0.0, 0.0);
    corr_model_command(m.get(), &st);
    corr_dump_t d[8];
    corr_model_process(m.get(), 0, codes.data(), n, d, 8);
    EXPECT_GT(m->acc_overflows, 0u);
    EXPECT_GT(m->acc_peak, 127);
}
