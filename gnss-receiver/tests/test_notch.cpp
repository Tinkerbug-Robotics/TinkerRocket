// The FPGA notch stage, bit exact (fpga/model/notch.h): the same arithmetic as the word-length
// study that chose its formats, the tone taken out, the requantizer holding a third, and a
// checksum that pins its output for the HDL.

extern "C" {
#include "jam.h"
#include "mitig.h"
#include "notch.h"
#include "quant.h"
#include "rng.h"
}

#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <cstdio>
#include <string>
#include <vector>

namespace {

const double kFs = 6.75e6;

// Noise and a tone through the front end's 2-bit quantizer.
std::vector<uint8_t> codes_with_tone(double f, double jnr_db, size_t n, uint64_t seed = 7)
{
    std::vector<float> iq(2 * n, 0.0f);
    rng_t r;
    rng_seed(&r, seed);
    rng_add_noise(&r, iq.data(), n, 1.0);
    if (jnr_db > -100.0) {
        jam_cfg_t c{};
        c.type = JAM_CW;
        c.f_hz = f;
        c.jnr_db = jnr_db;
        jam_t j;
        EXPECT_EQ(jam_init(&j, &c, kFs, 2.0 / kFs, kFs, 1), 0);
        jam_add(&j, iq.data(), n);
        jam_free(&j);
    }
    quant2_t q;
    quant2_init(&q, 0.33, 0.01 * kFs, 256);
    std::vector<uint8_t> codes(n);
    quant2_apply(&q, iq.data(), n, codes.data());
    return codes;
}

std::complex<double> weight(uint8_t c)
{
    const double i = (c & 0x4u) ? 3.0 : 1.0, q = (c & 0x1u) ? 3.0 : 1.0;
    return {(c & 0x8u) ? -i : i, (c & 0x2u) ? -q : q};
}

// The tone's share of the power at f over the samples from `from` on.
double tone_share(const std::vector<uint8_t> &c, size_t from, double f)
{
    std::complex<double> acc = 0.0;
    double p = 0.0;
    for (size_t k = from; k < c.size(); k++) {
        const auto w = weight(c[k]);
        acc += w * std::polar(1.0, -2.0 * M_PI * f * (double)k / kFs);
        p += std::norm(w);
    }
    const double m = (double)(c.size() - from);
    return std::norm(acc) / (m * m) / (p / m);
}

}  // namespace

TEST(Notch, EqualsTheWordLengthStudy)
{
    // The study (--mitig anfq, two notches, the approved formats) and the model must give the same
    // y, before the requantizer, sample for sample.
    const size_t n = 1u << 16;
    const auto in = codes_with_tone(-412.5e3, 20.0, n);
    mit_cfg_t c;
    ASSERT_EQ(mit_parse("anfq:2", &c), 0);
    mit_t m;
    ASSERT_EQ(mit_init(&m, &c, kFs), 0);
    std::vector<uint8_t> out_study(n);
    mit_apply(&m, in.data(), out_study.data(), n);
    notch_t nt;
    notch_init(&nt);
    long diff = 0;
    for (size_t k = 0; k < n; k++) {
        notch_sample(&nt, in[k]);
        const double yr = (double)nt.yr / (1 << NOTCH_FA), yi = (double)nt.yi / (1 << NOTCH_FA);
        diff += yr != (double)m.y[2 * k] || yi != (double)m.y[2 * k + 1];
    }
    EXPECT_EQ(diff, 0);
    mit_free(&m);
}

TEST(Notch, TakesOutATone)
{
    const size_t n = 1u << 18;
    const double f = 300e3;
    for (const double jnr : {10.0, 30.0}) {
        const auto in = codes_with_tone(f, jnr, n);
        notch_t nt;
        notch_init(&nt);
        std::vector<uint8_t> out(n);
        notch_process(&nt, in.data(), out.data(), n);
        const double depth = 10.0 * std::log10(tone_share(out, n / 4, f) / tone_share(in, n / 4, f));
        EXPECT_LT(depth, -20.0) << jnr;
        double fz, d;
        notch_zero(&nt, 0, &fz, &d);
        EXPECT_NEAR(fz * kFs, f, 500.0) << jnr;
        EXPECT_GT(d, 0.95) << jnr;
        // The counters see the tone's power go.
        uint64_t pin, pout;
        notch_take_power(&nt, &pin, &pout);
        EXPECT_GT((double)pin / (double)pout, jnr > 20.0 ? 10.0 : 2.0) << jnr;
    }
}

TEST(Notch, RequantizerHoldsAThird)
{
    const size_t n = 1u << 18;
    const auto in = codes_with_tone(0.0, -200.0, n);  // noise alone
    notch_t nt;
    notch_init(&nt);
    std::vector<uint8_t> out(n);
    notch_process(&nt, in.data(), out.data(), n);
    long mags = 0;
    for (size_t k = n / 2; k < n; k++) {
        mags += ((out[k] & 0x4u) != 0) + ((out[k] & 0x1u) != 0);
    }
    EXPECT_NEAR((double)mags / (double)n, 0.33, 0.01);
    // With nothing to remove, the stage passes the noise nearly as it came.
    uint64_t pin, pout;
    notch_take_power(&nt, &pin, &pout);
    EXPECT_NEAR(10.0 * std::log10((double)pin / (double)pout), 0.0, 0.3);
}

// Codes from integers alone (no libm, no float), the same on every platform: xorshift noise, with
// a tone's sign taken from an integer phase half the time.
std::vector<uint8_t> integer_codes(size_t n)
{
    std::vector<uint8_t> c(n);
    uint64_t x = 0x9E3779B97F4A7C15ull;
    uint32_t ph = 0;
    const uint32_t step = 190887435u;  // ~300 kHz at 6.75 MS/s
    for (size_t k = 0; k < n; k++) {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        unsigned code = 0;
        const unsigned q = ph >> 30;  // the tone's quadrant: I = cos sign, Q = sin sign
        const int tone = (x >> 8) & 1;
        const int si = tone ? (q == 1 || q == 2) : (int)(x & 1);
        const int sq = tone ? (q >= 2) : (int)((x >> 1) & 1);
        code |= si ? 0x8u : 0u;
        code |= sq ? 0x2u : 0u;
        code |= ((x >> 2) & 3) == 0 ? 0x4u : 0u;  // magnitude bits about a quarter of the time
        code |= ((x >> 4) & 3) == 0 ? 0x1u : 0u;
        c[k] = (uint8_t)code;
        ph += step;
    }
    return c;
}

TEST(Notch, OutputIsPinned)
{
    // FNV-1a over the output of a fixed integer input: the HDL's reference must not move
    // unnoticed. A deliberate change to the model updates this, with the reason in the commit.
    const size_t n = 1u << 16;
    const auto in = integer_codes(n);
    notch_t nt;
    notch_init(&nt);
    uint64_t h = 1469598103934665603ull;
    for (size_t k = 0; k < n; k++) {
        h = (h ^ notch_sample(&nt, in[k])) * 1099511628211ull;
    }
    for (int s = 0; s < NOTCH_N; s++) {
        h = (h ^ (uint64_t)nt.s[s].zr) * 1099511628211ull;
        h = (h ^ (uint64_t)nt.s[s].zi) * 1099511628211ull;
    }
    h = (h ^ (uint64_t)nt.t) * 1099511628211ull;
    EXPECT_EQ(h, 0x3ab515ebad9d4a0bull) << "pinned output changed: 0x" << std::hex << h;
    double fz, d;
    notch_zero(&nt, 0, &fz, &d);
    EXPECT_NEAR(fz * kFs, 300e3, 2e3) << "the pinned input's tone, found";
}

TEST(Notch, WritesItsVectors)
{
    const std::string dir = ::testing::TempDir();
    const size_t n = 6750 * 4;
    const auto in = codes_with_tone(300e3, 10.0, n);
    mit_cfg_t c;
    ASSERT_EQ(mit_parse("notch", &c), 0);
    mit_t m;
    ASSERT_EQ(mit_init(&m, &c, kFs), 0);
    ASSERT_EQ(mit_set_vectors(&m, dir.c_str(), n, 6750), 0);
    std::vector<uint8_t> out(n);
    mit_apply(&m, in.data(), out.data(), n);
    mit_free(&m);
    FILE *f = std::fopen((dir + "/notch_in.u2").c_str(), "rb");
    ASSERT_NE(f, nullptr);
    std::fseek(f, 0, SEEK_END);
    EXPECT_EQ(std::ftell(f), (long)(n / 2));
    std::fclose(f);
    f = std::fopen((dir + "/notch_power.csv").c_str(), "r");
    ASSERT_NE(f, nullptr);
    char line[512];
    int rows = 0;
    while (std::fgets(line, sizeof(line), f)) {
        rows++;
    }
    std::fclose(f);
    EXPECT_EQ(rows, 1 + 4);  // the header and one row a millisecond
    // The output codes the vectors carry are the model's.
    notch_t nt;
    notch_init(&nt);
    std::vector<uint8_t> ref(n);
    notch_process(&nt, in.data(), ref.data(), n);
    EXPECT_EQ(ref, out);
}
