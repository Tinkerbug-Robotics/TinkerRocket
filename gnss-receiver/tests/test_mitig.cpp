// The interference study's pieces (milestone 7): the interferer's power as set, and the two
// candidate FPGA stages taking a tone out of a 2-bit stream while leaving the noise.

extern "C" {
#include "jam.h"
#include "mitig.h"
#include "quant.h"
#include "rng.h"
}

#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <vector>

namespace {

double mean_power(const std::vector<float> &iq)
{
    double s = 0.0;
    for (float v : iq) {
        s += (double)v * v;
    }
    return s / (double)(iq.size() / 2);
}

// Power at f (Hz) by a direct DFT over the samples, per sample: a tone of amplitude A reads A^2.
double tone_power(const std::vector<float> &iq, size_t from, double f, double fs)
{
    std::complex<double> acc = 0.0;
    const size_t n = iq.size() / 2 - from;
    for (size_t k = 0; k < n; k++) {
        const double ph = -2.0 * M_PI * f * (double)(from + k) / fs;
        acc += std::complex<double>(iq[2 * (from + k)], iq[2 * (from + k) + 1]) * std::polar(1.0, ph);
    }
    return std::norm(acc) / ((double)n * (double)n);
}

// White noise plus a tone, through the 2-bit quantizer: the stage's input.
std::vector<uint8_t> two_bit(double fs, double f, double jnr_db, size_t n, std::vector<float> *pre = nullptr)
{
    std::vector<float> iq(2 * n, 0.0f);
    rng_t r;
    rng_seed(&r, 7);
    rng_add_noise(&r, iq.data(), n, 1.0);  // noise power 2 over fs: n0 = 2 / fs
    jam_cfg_t c{};
    c.type = JAM_CW;
    c.f_hz = f;
    c.jnr_db = jnr_db;
    jam_t j;
    EXPECT_EQ(jam_init(&j, &c, fs, 2.0 / fs, fs, 1), 0);
    jam_add(&j, iq.data(), n);
    jam_free(&j);
    if (pre) {
        *pre = iq;
    }
    quant2_t q;
    quant2_init(&q, 0.33, 0.01 * fs, 256);
    std::vector<uint8_t> codes(n);
    quant2_apply(&q, iq.data(), n, codes.data());
    return codes;
}

std::vector<float> weights(const std::vector<uint8_t> &codes)
{
    std::vector<float> w(2 * codes.size());
    for (size_t k = 0; k < codes.size(); k++) {
        const unsigned c = codes[k];
        const float i = (c & 0x4u) ? 3.0f : 1.0f, q = (c & 0x1u) ? 3.0f : 1.0f;
        w[2 * k] = (c & 0x8u) ? -i : i;
        w[2 * k + 1] = (c & 0x2u) ? -q : q;
    }
    return w;
}

}  // namespace

TEST(Jam, PowerIsSetAgainstTheNoiseInTheBand)
{
    const double fs = 6.75e6, n0 = 1e-6, bw = 4.2e6;  // noise in the band: 4.2
    for (int type = 0; type < 2; type++) {
        jam_cfg_t c{};
        c.type = type == 0 ? JAM_CW : JAM_NB;
        c.f_hz = 300e3;
        c.jnr_db = 10.0;
        c.bw_hz = 100e3;
        jam_t j;
        ASSERT_EQ(jam_init(&j, &c, fs, n0, bw, 3), 0);
        std::vector<float> iq(2 * (1u << 20), 0.0f);
        jam_add(&j, iq.data(), iq.size() / 2);
        jam_free(&j);
        EXPECT_NEAR(mean_power(iq) / (10.0 * n0 * bw), 1.0, type == 0 ? 1e-3 : 0.05) << type;
    }
}

TEST(Jam, ParsesItsThreeForms)
{
    jam_cfg_t c;
    ASSERT_EQ(jam_parse("cw:300e3:20", &c), 0);
    EXPECT_EQ(c.type, JAM_CW);
    EXPECT_DOUBLE_EQ(c.f_hz, 300e3);
    ASSERT_EQ(jam_parse("nb:-1e5:5:1e5", &c), 0);
    EXPECT_EQ(c.type, JAM_NB);
    EXPECT_DOUBLE_EQ(c.bw_hz, 1e5);
    ASSERT_EQ(jam_parse("chirp:0:20:2e6:1e-5", &c), 0);
    EXPECT_EQ(c.type, JAM_CHIRP);
    EXPECT_NE(jam_parse("cw:300e3", &c), 0);
    EXPECT_NE(jam_parse("nb:0:5", &c), 0);
}

TEST(Mitig, NotchFindsTheToneAndTakesItOut)
{
    const double fs = 6.75e6, f = -412.5e3;
    const size_t n = 1u << 18;
    const auto in = two_bit(fs, f, 10.0, n);
    mit_cfg_t c;
    ASSERT_EQ(mit_parse("anf", &c), 0);
    mit_t m;
    ASSERT_EQ(mit_init(&m, &c, fs), 0);
    std::vector<uint8_t> out(n);
    mit_apply(&m, in.data(), out.data(), n);
    const double fz = std::atan2(m.notch[0].zi, m.notch[0].zr) / (2.0 * M_PI) * fs;
    EXPECT_NEAR(fz, f, 500.0);
    const auto wi = weights(in), wo = weights(out);
    const size_t settle = n / 4;
    const double before = tone_power(wi, settle, f, fs) / mean_power(wi);
    const double after = tone_power(wo, settle, f, fs) / mean_power(wo);
    EXPECT_LT(10.0 * std::log10(after / before), -20.0);  // -24 dB: the zero's jitter sets the depth
    mit_free(&m);
}

TEST(Mitig, FixedPointNotchMatchesTheFloatOne)
{
    // The word-length study's candidate: the tone found and taken out within 3 dB of the float
    // notch's depth (in the receiver the two give the same C/N0 to 0.05 dB through 25 dB JNR).
    const double fs = 6.75e6, f = -412.5e3;
    const size_t n = 1u << 18;
    for (const double jnr : {0.0, 10.0, 30.0}) {
        const auto in = two_bit(fs, f, jnr, n);
        double depth[2], fz[2];
        for (int q = 0; q < 2; q++) {
            mit_cfg_t c;
            ASSERT_EQ(mit_parse(q ? "anfq" : "anf", &c), 0);
            mit_t m;
            ASSERT_EQ(mit_init(&m, &c, fs), 0);
            std::vector<uint8_t> out(n);
            mit_apply(&m, in.data(), out.data(), n);
            fz[q] = q ? std::atan2((double)m.notch[0].qzi, (double)m.notch[0].qzr) / (2.0 * M_PI) * fs
                      : std::atan2(m.notch[0].zi, m.notch[0].zr) / (2.0 * M_PI) * fs;
            const auto wi = weights(in), wo = weights(out);
            depth[q] = 10.0 * std::log10((tone_power(wo, n / 4, f, fs) / mean_power(wo)) /
                                         (tone_power(wi, n / 4, f, fs) / mean_power(wi)));
            mit_free(&m);
        }
        EXPECT_NEAR(fz[1], f, 500.0) << jnr;
        EXPECT_LT(depth[1], -20.0) << jnr;
        EXPECT_LT(depth[1], depth[0] + 3.0) << "fixed point " << depth[1] << " dB against float " << depth[0] << " at "
                                            << jnr;
    }
}

TEST(Mitig, ExcisionTakesTheToneOutAndLeavesNoiseAlone)
{
    const double fs = 6.75e6, f = 300e3;
    const size_t n = 1u << 18;
    mit_cfg_t c;
    ASSERT_EQ(mit_parse("fde:1024:8:0.005", &c), 0);  // a short average: the run is 512 frames
    {
        const auto in = two_bit(fs, f, 10.0, n);
        mit_t m;
        ASSERT_EQ(mit_init(&m, &c, fs), 0);
        std::vector<uint8_t> out(n);
        mit_apply(&m, in.data(), out.data(), n);
        const auto wi = weights(in), wo = weights(out);
        const double before = tone_power(wi, n / 4, f, fs) / mean_power(wi);
        const double after = tone_power(wo, n / 4, f, fs) / mean_power(wo);
        EXPECT_LT(10.0 * std::log10(after / before), -20.0);
        EXPECT_GT(m.bins_cut, 0u);
        mit_free(&m);
    }
    {
        // Noise alone: nothing stands 8 times over the median, and nothing is cut.
        const auto in = two_bit(fs, f, -60.0, n);
        mit_t m;
        ASSERT_EQ(mit_init(&m, &c, fs), 0);
        std::vector<uint8_t> out(n);
        mit_apply(&m, in.data(), out.data(), n);
        EXPECT_LT((double)m.bins_cut / (double)m.frames, 0.05);
        mit_free(&m);
    }
}

TEST(Mitig, NoneIsACopy)
{
    mit_cfg_t c;
    ASSERT_EQ(mit_parse("none", &c), 0);
    mit_t m;
    ASSERT_EQ(mit_init(&m, &c, 6.75e6), 0);
    const std::vector<uint8_t> in = {0x0, 0xF, 0x8, 0x5, 0xA, 0x6};
    std::vector<uint8_t> out(in.size());
    mit_apply(&m, in.data(), out.data(), in.size());
    EXPECT_EQ(in, out);
    mit_free(&m);
    EXPECT_NE(mit_parse("anf:9", &c), 0);
    EXPECT_NE(mit_parse("fde:1000", &c), 0);
}
