// The correlator bank on Galileo E1 and BeiDou B1C (milestone 6): the BOC(1,1) correlation
// shape on five taps, the data prompt beside the pilot, and the secondary code in the pilot's
// dump signs; the bit-exact model and the float bank in step.

#include "sig_gen.h"

extern "C" {
#include "corr_float.h"
#include "corr_model.h"
#include "fe_format.h"
#include "quant.h"
}

#include <gtest/gtest.h>

#include <complex>
#include <memory>
#include <vector>

namespace {

constexpr double kFs = 6.75e6, kIf = -2.658052e6;
constexpr double kOne = double(uint64_t(1) << CORR_CODE_FRAC_BITS);
using cd = std::complex<double>;

corr_cmd_t start(gnss_sig_t sig, int prn, double code_phase, double dop)
{
    corr_cmd_t c{};
    c.type = CORR_CMD_START;
    c.ch = 3;
    c.sig = uint8_t(sig);
    c.prn = uint8_t(prn);
    c.code_phase = uint64_t(std::llround(code_phase * kOne));
    c.tap_offset = uint64_t(0.1 * kOne);
    c.tap_offset2 = uint64_t(0.5 * kOne);
    c.carr_word = int32_t(std::llround((kIf + dop) / kFs * 4294967296.0));
    c.code_word = uint64_t(std::llround(1.023e6 * (1.0 + dop / GNSS_FREQ_L1_HZ) / kFs * kOne));
    return c;
}

std::vector<uint8_t> two_bit(const std::vector<float> &x)
{
    size_t n = x.size() / 2;
    std::vector<uint8_t> codes(n);
    quant2_t q;
    quant2_init(&q, 0.33, 20000.0, 256);
    quant2_apply(&q, x.data(), n, codes.data());
    return codes;
}

std::vector<corr_dump_t> run_model(const std::vector<uint8_t> &codes, const corr_cmd_t &st)
{
    auto m = std::make_unique<corr_model_t>();
    corr_model_cfg_t cfg;
    corr_model_default_cfg(&cfg);
    corr_model_init(m.get(), &cfg);
    EXPECT_EQ(corr_model_command(m.get(), &st), 0);
    std::vector<corr_dump_t> out(256);
    int nd = corr_model_process(m.get(), 0, codes.data(), codes.size(), out.data(), int(out.size()));
    out.resize(size_t(nd));
    EXPECT_EQ(m->acc_overflows, 0u);
    return out;
}

cd P(const corr_dump_t &d) { return {d.ip, d.qp}; }
cd E(const corr_dump_t &d) { return {d.ie, d.qe}; }
cd L(const corr_dump_t &d) { return {d.il, d.ql}; }
cd VE(const corr_dump_t &d) { return {d.ive, d.qve}; }
cd VL(const corr_dump_t &d) { return {d.ivl, d.qvl}; }
cd D(const corr_dump_t &d) { return {d.id, d.qd}; }

// Mean over the full periods of a ratio of taps, each projected on the prompt.
double rel(const std::vector<corr_dump_t> &ds, cd (*tap)(const corr_dump_t &))
{
    double s = 0.0;
    int n = 0;
    for (const auto &d : ds) {
        if (d.seq == 0) {
            continue;
        }
        cd p = P(d);
        s += (tap(d) * std::conj(p)).real() / std::norm(p);
        n++;
    }
    return s / n;
}

}  // namespace

TEST(CorrBoc, GalileoE1ShapeDataAndSecondaryCode)
{
    siggen::Sat s;
    s.sig = GNSS_SIG_GAL_E1C;
    s.prn = 11;
    s.dop = 1234.0;
    s.code_phase = 1000.25;
    s.phase = 0.7;
    s.data_ms = 1;  // a new E1-B symbol every 4 ms period, alternating
    const size_t n = size_t(kFs * 0.2);
    auto x = siggen::make({s}, kFs, kIf, n, siggen::sigma_for(55.0, kFs));
    auto ds = run_model(two_bit(x), start(GNSS_SIG_GAL_E1C, 11, 1000.25, 1234.0));
    ASSERT_GE(ds.size(), 50u);  // 4 ms periods

    // BOC(1,1): R(tau) = 1 - 3|tau| near the peak, -0.5 at +-0.5 chip. 2-bit samples at 6.6 per
    // chip round the corners, so the bands are generous.
    EXPECT_NEAR(rel(ds, E), rel(ds, L), 0.03);
    EXPECT_GT(rel(ds, E), 0.55);
    EXPECT_LT(rel(ds, E), 0.8);
    EXPECT_LT(rel(ds, VE), -0.3);
    EXPECT_LT(rel(ds, VL), -0.3);
    EXPECT_GT(rel(ds, VE), -0.65);

    // E1-B at the same power as E1-C and in phase with it (the ICD's E1 is real); its sign
    // alternates with the symbols, so look at magnitudes and the quadrature residue.
    double ratio = 0.0, quad = 0.0;
    int m = 0;
    for (const auto &d : ds) {
        if (d.seq == 0) {
            continue;
        }
        ratio += std::abs(D(d)) / std::abs(P(d));
        quad += std::abs((D(d) * std::conj(P(d))).imag()) / std::norm(P(d));
        m++;
    }
    EXPECT_NEAR(ratio / m, 1.0, 0.1);
    EXPECT_LT(quad / m, 0.1);

    // The pilot's prompt signs carry CS25, the generator's secondary chip ep % 25 with ep the
    // period since the reference: find it at some offset, up to the carrier's sign.
    uint8_t cs[GAL_E1C_SEC_LEN];
    gal_e1c_secondary(cs);
    cd ref = P(ds[1]);
    int best = 0;
    for (int off = 0; off < GAL_E1C_SEC_LEN; off++) {
        int agree = 0;
        for (size_t k = 1; k < ds.size(); k++) {
            int sgn = (P(ds[k]) * std::conj(ref)).real() > 0 ? 1 : -1;
            int want = cs[(k + size_t(off)) % GAL_E1C_SEC_LEN] ? -1 : 1;
            agree += sgn * want;
        }
        best = std::max(best, std::abs(agree));
    }
    EXPECT_EQ(best, int(ds.size()) - 1);
}

TEST(CorrBoc, BeidouB1cDataInQuadratureWithThePilot)
{
    siggen::Sat s;
    s.sig = GNSS_SIG_BDS_B1CP;
    s.prn = 29;
    s.dop = -2642.0;
    s.code_phase = 4321.5;
    s.phase = -1.1;
    s.data_ms = 1;
    const size_t n = size_t(kFs * 0.2);
    auto x = siggen::make({s}, kFs, kIf, n, siggen::sigma_for(55.0, kFs));
    auto ds = run_model(two_bit(x), start(GNSS_SIG_BDS_B1CP, 29, 4321.5, -2642.0));
    ASSERT_GE(ds.size(), 19u);  // 10 ms periods
    EXPECT_NEAR(rel(ds, E), rel(ds, L), 0.03);
    EXPECT_LT(rel(ds, VE), -0.3);
    // SignalSim's B1C: data on -Q at 1/2 against the pilot's sqrt(29/44) on I: |D| / |P| = 0.62,
    // all of it in quadrature.
    double ratio = 0.0, inphase = 0.0;
    int m = 0;
    for (const auto &d : ds) {
        if (d.seq == 0) {
            continue;
        }
        ratio += std::abs(D(d)) / std::abs(P(d));
        inphase += std::abs((D(d) * std::conj(P(d))).real()) / std::norm(P(d));
        m++;
    }
    EXPECT_NEAR(ratio / m, 0.5 / std::sqrt(29.0 / 44.0), 0.07);
    EXPECT_LT(inphase / m, 0.1);
}

TEST(CorrBoc, ModelAndFloatBankInStepOnE1AndB1c)
{
    for (gnss_sig_t sig : {GNSS_SIG_GAL_E1C, GNSS_SIG_BDS_B1CP}) {
        siggen::Sat s;
        s.sig = sig;
        s.prn = 7;
        s.dop = 800.0;
        s.code_phase = 123.4;
        const size_t n = size_t(kFs * 0.05);
        auto x = siggen::make({s}, kFs, kIf, n, siggen::sigma_for(50.0, kFs));
        auto codes = two_bit(x);
        std::vector<float> w(2 * n);
        for (size_t k = 0; k < n; k++) {
            unsigned c = codes[k];
            float i = (c & FE_CODE_I_MAG) ? 3.0f : 1.0f, q = (c & FE_CODE_Q_MAG) ? 3.0f : 1.0f;
            w[2 * k] = (c & FE_CODE_I_SIGN) ? -i : i;
            w[2 * k + 1] = (c & FE_CODE_Q_SIGN) ? -q : q;
        }
        corr_cmd_t st = start(sig, 7, 123.4, 800.0);
        auto dm = run_model(codes, st);
        auto f = std::make_unique<corr_float_t>();
        corr_float_init(f.get(), kFs);
        ASSERT_EQ(corr_float_command(f.get(), &st), 0);
        std::vector<corr_dump_t> df(64);
        df.resize(size_t(corr_float_process(f.get(), 0, w.data(), n, df.data(), 64)));
        ASSERT_EQ(dm.size(), df.size()) << sig;
        for (size_t k = 0; k < dm.size(); k++) {
            EXPECT_EQ(dm[k].seq, df[k].seq);
            EXPECT_EQ(dm[k].t_samp, df[k].t_samp);
            EXPECT_EQ(dm[k].code_phase, df[k].code_phase);
            EXPECT_EQ(dm[k].carr_phase, df[k].carr_phase);
            if (dm[k].seq > 0) {
                for (auto tap : {P, D, VE}) {
                    double dphi = std::arg(tap(dm[k]) * std::conj(tap(df[k])));
                    EXPECT_NEAR(dphi, 0.0, 0.15) << sig << " dump " << k;
                }
            }
        }
    }
}
