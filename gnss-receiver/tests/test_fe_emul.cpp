#include "fe_emul.h"
#include "fe_format.h"
#include "gnss/types.h"
#include "tone.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

std::vector<float> weights(const std::vector<uint8_t> &c)
{
    std::vector<float> v(2 * c.size());
    for (size_t k = 0; k < c.size(); k++) {
        unsigned x = c[k];
        float i = (x & FE_CODE_I_MAG) ? FE_WEIGHT_LARGE : FE_WEIGHT_SMALL;
        float q = (x & FE_CODE_Q_MAG) ? FE_WEIGHT_LARGE : FE_WEIGHT_SMALL;
        v[2 * k] = (x & FE_CODE_I_SIGN) ? -i : i;
        v[2 * k + 1] = (x & FE_CODE_Q_SIGN) ? -q : q;
    }
    return v;
}

}  // namespace

TEST(FeEmul, DirectModePutsL1AtTheIf)
{
    // A tone 500 kHz above L1 in a wide-format file (L1 7.134 MHz above centre).
    const double fs_in = 18.48e6, fc_in = 1568.286e6, f_rel = 0.5e6;
    const size_t n = 400000;
    auto x = tone::make(GNSS_FREQ_L1_HZ - fc_in + f_rel, fs_in, n, 20.0);
    fe_cfg_t c;
    fe_cfg_default(&c);
    c.fs_in = fs_in;
    c.fc_in = fc_in;
    c.noise_sigma = 20.0;
    c.agc_tau_s = 0.001;  // settles well inside the 22 ms run
    fe_t fe;
    ASSERT_EQ(fe_init(&fe, &c), 0);
    std::vector<uint8_t> codes(fe_max_out(&fe, n));
    size_t m = fe_process(&fe, x.data(), n, nullptr, codes.data(), codes.size());
    codes.resize(m);
    EXPECT_NEAR(double(m), n * FE_FS_CORR_HZ / fs_in, 40.0);
    auto y = weights(codes);
    // The tone now sits at IF + 500 kHz in the 6.75 MS/s stream.
    auto f = tone::fit(y.data(), 1000, m, fe_out_if(&fe) + f_rel, FE_FS_CORR_HZ);
    auto off = tone::fit(y.data(), 1000, m, -fe_out_if(&fe) - f_rel, FE_FS_CORR_HZ);
    EXPECT_GT(std::abs(f.amp), 0.3);
    EXPECT_LT(std::abs(off.amp), 0.02);  // no image: the stream is complex
    EXPECT_NEAR(quant2_density(&fe.q), 0.33, 0.01);
    fe_free(&fe);
}

TEST(FeEmul, CarrierFixShiftsOnlyTheCarrier)
{
    const double fs = 2.6e6;
    const size_t n = 26000;
    auto x = tone::make(1000.0, fs, n);
    fe_cfg_t c;
    fe_cfg_default(&c);
    c.mode = FE_MODE_NATIVE;
    c.fs_in = fs;
    c.if_hz = 0.0;
    c.carrier_fix_hz = 22.0;
    fe_t fe;
    ASSERT_EQ(fe_init(&fe, &c), 0);
    std::vector<float> y(2 * n);
    ASSERT_EQ(fe_process(&fe, x.data(), n, y.data(), nullptr, n), n);
    auto f = tone::fit(y.data(), 0, n, 1022.0, fs);
    EXPECT_NEAR(std::abs(f.amp), 1.0, 1e-5);
    EXPECT_NEAR(fe_out_position(&fe, 100), 100.0, 0.0);
    fe_free(&fe);
}

TEST(FeEmul, SigmaForCn0)
{
    // C = 1 LSB^2, target 45 dB-Hz, nothing in the file: N0 = 10^-4.5 per Hz,
    // so sigma^2 per component = N0 * fs / 2.
    double s = fe_sigma_for_cn0(45.0, 1.0, 0.0, 6.75e6);
    EXPECT_NEAR(s * s, std::pow(10.0, -4.5) * 6.75e6 / 2.0, 1e-9);
    // Already noisier than the target.
    EXPECT_LT(fe_sigma_for_cn0(45.0, 1.0, 1e-3, 6.75e6), 0.0);
}

TEST(FeEmul, Adc27SubsamplesToTheCorrelatorRate)
{
    const double fs_in = 8.184e6;
    const size_t n = 200000;
    auto x = tone::make(0.3e6, fs_in, n, 20.0);
    fe_cfg_t c;
    fe_cfg_default(&c);
    c.mode = FE_MODE_ADC27;
    c.fs_in = fs_in;
    c.noise_sigma = 20.0;
    fe_t fe;
    ASSERT_EQ(fe_init(&fe, &c), 0);
    std::vector<uint8_t> codes(fe_max_out(&fe, n));
    size_t m = fe_process(&fe, x.data(), n, nullptr, codes.data(), codes.size());
    codes.resize(m);
    EXPECT_NEAR(double(m), n * FE_FS_CORR_HZ / fs_in, 40.0);
    auto y = weights(codes);
    auto f = tone::fit(y.data(), 1000, m, fe_out_if(&fe) + 0.3e6, FE_FS_CORR_HZ);
    EXPECT_GT(std::abs(f.amp), 0.3);
    fe_free(&fe);
}
