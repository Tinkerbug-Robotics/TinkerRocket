#include "sig_gen.h"

extern "C" {
#include "corr_float.h"
}

#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <memory>

namespace {

constexpr double kFs = 6.75e6, kIf = 1.2e6;
constexpr double kFrac = double(uint64_t(1) << CORR_CODE_FRAC_BITS);

int32_t carr_word(double hz)
{
    return int32_t(std::llround(hz / kFs * 4294967296.0));
}

uint64_t code_word(double chips_per_s)
{
    return uint64_t(std::llround(chips_per_s / kFs * kFrac));
}

corr_cmd_t start_cmd(int prn, double code_phase, double dop)
{
    corr_cmd_t c{};
    c.type = CORR_CMD_START;
    c.ch = 3;
    c.sig = GNSS_SIG_GPS_L1CA;
    c.prn = uint8_t(prn);
    c.t_start = 0;
    c.code_phase = uint64_t(std::llround(code_phase * kFrac));
    c.tap_offset = uint64_t(0.5 * kFrac);
    c.carr_word = carr_word(kIf + dop);
    c.code_word = code_word(1.023e6 * (1.0 + dop / GNSS_FREQ_L1_HZ));
    return c;
}

}  // namespace

TEST(CorrFloat, AlignedReplicaGivesTheFullPromptAndEqualEarlyLate)
{
    siggen::Sat s;
    s.prn = 7;
    s.dop = 1500.0;
    s.code_phase = 100.25;
    s.phase = 0.7;
    const size_t n = size_t(kFs * 0.006);
    auto x = siggen::make({s}, kFs, kIf, n, 0.0);
    auto c = std::make_unique<corr_float_t>();
    corr_float_init(c.get(), kFs);
    corr_cmd_t st = start_cmd(7, 100.25, 1500.0);
    ASSERT_EQ(corr_float_command(c.get(), &st), 0);
    corr_dump_t d[16];
    int nd = corr_float_process(c.get(), 0, x.data(), n, d, 16);
    ASSERT_GE(nd, 5);
    uint8_t chips[GPS_CA_LEN];
    gps_ca_code(7, chips);
    int acc1 = 0;
    for (int k = 0; k < GPS_CA_LEN; k++) {
        acc1 += (chips[k] ^ chips[(k + 1) % GPS_CA_LEN]) ? -1 : 1;
    }
    const double r1 = acc1 / double(GPS_CA_LEN);
    // Dump 0 is the partial period; the others cover whole periods of ~6750 samples.
    for (int k = 1; k < nd; k++) {
        EXPECT_EQ(d[k].ch, 3);
        EXPECT_EQ(d[k].seq, uint32_t(k));
        EXPECT_NEAR(double(d[k].t_samp - d[k - 1].t_samp), kFs * 1e-3 / (1.0 + 1500.0 / GNSS_FREQ_L1_HZ), 1.0);
        std::complex<double> p(d[k].ip, d[k].qp), e(d[k].ie, d[k].qe), l(d[k].il, d[k].ql);
        double full = kFs * 1e-3;
        EXPECT_NEAR(std::abs(p) / full, 1.0, 0.01);
        // Half a chip off the peak: (1 + R(1)) / 2, R(1) being the code's own correlation one chip off
        // (a Gold code's is -1, -65 or +63 / 1023, not 0).
        EXPECT_NEAR(std::abs(e) / full, 0.5 * (1.0 + r1), 0.01);
        EXPECT_NEAR(std::abs(l) / full, 0.5 * (1.0 + r1), 0.01);
        // The NCO starts at phase 0, the signal at 0.7 rad: the prompt sits at 0.7 rad.
        EXPECT_NEAR(std::arg(p), 0.7, 0.01);
    }
}

TEST(CorrFloat, EarlyWinsWhenTheReplicaLags)
{
    siggen::Sat s;
    s.prn = 12;
    s.code_phase = 50.0;
    const size_t n = size_t(kFs * 0.003);
    auto x = siggen::make({s}, kFs, kIf, n, 0.0);
    auto c = std::make_unique<corr_float_t>();
    corr_float_init(c.get(), kFs);
    corr_cmd_t st = start_cmd(12, 49.8, 0.0);  // replica 0.2 chips behind the signal
    corr_float_command(c.get(), &st);
    corr_dump_t d[8];
    int nd = corr_float_process(c.get(), 0, x.data(), n, d, 8);
    ASSERT_GE(nd, 2);
    double e = std::hypot(d[1].ie, d[1].qe), l = std::hypot(d[1].il, d[1].ql);
    // Triangle: E = 1 - |0.2 - 0.5| = 0.7, L = 1 - (0.2 + 0.5) = 0.3.
    EXPECT_NEAR(e / (kFs * 1e-3), 0.7, 0.03);
    EXPECT_NEAR(l / (kFs * 1e-3), 0.3, 0.03);
}

TEST(CorrFloat, NcoCommandsTakeEffectAtTheNextEpoch)
{
    siggen::Sat s;
    s.prn = 3;
    s.code_phase = 1022.5;  // first epoch half a chip in: dump 0 is a sliver, dump 1 a whole period
    const size_t n = size_t(kFs * 0.005);
    auto x = siggen::make({s}, kFs, kIf, n, 0.0);
    auto c = std::make_unique<corr_float_t>();
    corr_float_init(c.get(), kFs);
    corr_cmd_t st = start_cmd(3, 1022.5, 0.0);
    corr_float_command(c.get(), &st);
    corr_dump_t d[8];
    // First 1.5 ms: dump 0 (partial) and dump 1 come out.
    size_t half = size_t(kFs * 0.0015);
    int nd = corr_float_process(c.get(), 0, x.data(), half, d, 8);
    ASSERT_EQ(nd, 2);
    corr_cmd_t nco{};
    nco.type = CORR_CMD_NCO;
    nco.ch = 3;
    nco.carr_word = carr_word(kIf + 100.0);
    nco.code_word = st.code_word;
    corr_float_command(c.get(), &nco);
    int nd2 = corr_float_process(c.get(), half, x.data() + 2 * half, n - half, d + nd, 8 - nd);
    ASSERT_GE(nd2, 3);
    // The period in progress when the command arrived finishes on the old word.
    EXPECT_EQ(d[2].carr_word, st.carr_word);
    EXPECT_EQ(d[3].carr_word, nco.carr_word);
    EXPECT_EQ(d[4].carr_word, nco.carr_word);
}

TEST(CorrFloat, CarrierCyclesCountBothWays)
{
    const size_t n = size_t(kFs * 0.004);
    std::vector<float> x(2 * n, 0.0f);
    for (double f : {kIf + 2000.0, -kIf}) {
        auto c = std::make_unique<corr_float_t>();
        corr_float_init(c.get(), kFs);
        corr_cmd_t st = start_cmd(1, 0.0, 0.0);
        st.carr_word = carr_word(f);
        corr_float_command(c.get(), &st);
        corr_dump_t d[8];
        int nd = corr_float_process(c.get(), 0, x.data(), n, d, 8);
        ASSERT_GE(nd, 2);
        // Phase in cycles at the last dump = cycles + fraction = word * samples / 2^32.
        const corr_dump_t &last = d[nd - 1];
        double cycles = double(int32_t(last.carr_cycles)) + last.carr_phase / 4294967296.0;
        double want = double(st.carr_word) * double(last.t_samp) / 4294967296.0;
        EXPECT_NEAR(cycles, want, 1e-6) << f;
    }
}
