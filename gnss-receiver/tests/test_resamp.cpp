#include "resamp.h"
#include "tone.h"

#include <gtest/gtest.h>

namespace {

// Runs a whole input through the resampler in uneven chunks.
std::vector<float> run(resamp_t *r, const std::vector<float> &x)
{
    size_t n = x.size() / 2;
    std::vector<float> out(2 * (n * 5 + 1000));
    size_t k = 0, nout = 0, i = 0;
    size_t sizes[] = {777, 1, 4096, 13, 2048};
    while (k < n) {
        size_t b = std::min(sizes[i++ % 5], n - k);
        nout += resamp_process(r, x.data() + 2 * k, b, out.data() + 2 * nout, (out.size() / 2) - nout);
        k += b;
    }
    out.resize(2 * nout);
    return out;
}

}  // namespace

TEST(Resamp, DownsamplesATone)
{
    const double fs_in = 8.184e6, fs_out = 6.75e6, f0 = 0.9e6;
    auto x = tone::make(f0, fs_in, 200000);
    resamp_t r;
    ASSERT_EQ(resamp_init(&r, fs_in, fs_out, 2.2e6, 3.8e6, 80.0, 0), 0);
    auto y = run(&r, x);
    size_t n = y.size() / 2;
    ASSERT_GT(n, 150000u);
    // Output k is the input at position k * fs_in / fs_out: the same tone at the same phase.
    auto f = tone::fit(y.data(), 100, n - 100, f0, fs_out);
    EXPECT_NEAR(std::abs(f.amp), 1.0, 2e-3);
    EXPECT_NEAR(std::arg(f.amp), 0.0, 1e-3);
    EXPECT_LT(f.resid_db, -70.0);
    resamp_free(&r);
}

TEST(Resamp, UpsamplesAToneFromTheGpsSdrSimRate)
{
    const double fs_in = 2.6e6, fs_out = 6.75e6, f0 = 0.7e6;
    auto x = tone::make(f0, fs_in, 100000, 1.0, 0.3);
    resamp_t r;
    ASSERT_EQ(resamp_init(&r, fs_in, fs_out, 1.092e6, 1.508e6, 80.0, 0), 0);
    auto y = run(&r, x);
    size_t n = y.size() / 2;
    auto f = tone::fit(y.data(), 200, n - 200, f0, fs_out);
    EXPECT_NEAR(std::abs(f.amp), 1.0, 2e-3);
    EXPECT_NEAR(std::arg(f.amp), 0.3, 1e-3);
    EXPECT_LT(f.resid_db, -70.0);
    resamp_free(&r);
}

TEST(Resamp, RejectsATonePastTheStopBand)
{
    const double fs_in = 18.48e6, fs_out = 6.75e6;
    // The wide files put BeiDou B1I here after L1 is shifted to 0 Hz.
    auto x = tone::make(4.16e6, fs_in, 100000);
    resamp_t r;
    ASSERT_EQ(resamp_init(&r, fs_in, fs_out, 2.2e6, 3.8e6, 80.0, 0), 0);
    auto y = run(&r, x);
    double p = 0.0;
    size_t n = y.size() / 2;
    for (size_t k = 100; k < n; k++) {
        p += y[2 * k] * y[2 * k] + y[2 * k + 1] * y[2 * k + 1];
    }
    EXPECT_LT(10.0 * std::log10(p / double(n - 100)), -75.0);
    resamp_free(&r);
}

TEST(Resamp, StartIndexAlignsOutputWithFilePosition)
{
    // Input sample j carries phase of absolute index j0 + j; an output starting at j0 must match.
    const double fs_in = 8.184e6, fs_out = 6.75e6, f0 = -1.3e6;
    const int64_t j0 = 123456789;
    auto x = tone::make(f0, fs_in, 100000, 1.0, 0.0, double(j0));
    resamp_t r;
    ASSERT_EQ(resamp_init(&r, fs_in, fs_out, 2.2e6, 3.8e6, 80.0, j0), 0);
    auto y = run(&r, x);
    size_t n = y.size() / 2;
    // Output k is at absolute input position j0 + k*step: phase 2*pi*f0*(j0/fs_in + k/fs_out).
    double ph0 = tone::kTwoPi * f0 * double(j0) / fs_in;
    auto f = tone::fit(y.data(), 100, n - 100, f0, fs_out);
    double err = std::remainder(std::arg(f.amp) - ph0, tone::kTwoPi);
    EXPECT_NEAR(err, 0.0, 1e-3);
    EXPECT_NEAR(resamp_position(&r), double(j0) + double(n) * fs_in / fs_out, 1e-6);
    resamp_free(&r);
}

namespace {
double const_warp(void *ctx, double)
{
    return *static_cast<double *>(ctx);
}
}  // namespace

TEST(Resamp, WarpScalesTheSampleClock)
{
    const double fs_in = 8.184e6, fs_out = 6.75e6, f0 = 1.0e6;
    double delta = 2e-4;
    auto x = tone::make(f0, fs_in, 200000);
    resamp_t r;
    ASSERT_EQ(resamp_init(&r, fs_in, fs_out, 2.2e6, 3.8e6, 80.0, 0), 0);
    r.warp = const_warp;
    r.warp_ctx = &delta;
    auto y = run(&r, x);
    size_t n = y.size() / 2;
    // A clock running fast by delta samples the signal later and later: the tone appears at f0*(1+delta).
    auto f = tone::fit(y.data(), 100, n - 100, f0 * (1.0 + delta), fs_out);
    EXPECT_NEAR(std::abs(f.amp), 1.0, 2e-3);
    EXPECT_LT(f.resid_db, -60.0);
    resamp_free(&r);
}

TEST(Resamp, ChunkingDoesNotChangeTheOutput)
{
    const double fs_in = 2.6e6, fs_out = 6.75e6;
    auto x = tone::make(0.4e6, fs_in, 20000, 1.0, 1.0);
    resamp_t a, b;
    ASSERT_EQ(resamp_init(&a, fs_in, fs_out, 1.092e6, 1.508e6, 80.0, 0), 0);
    ASSERT_EQ(resamp_init(&b, fs_in, fs_out, 1.092e6, 1.508e6, 80.0, 0), 0);
    std::vector<float> ya(2 * 60000);
    size_t na = resamp_process(&a, x.data(), 20000, ya.data(), 60000);
    auto yb = run(&b, x);
    ASSERT_EQ(na, yb.size() / 2);
    for (size_t k = 0; k < 2 * na; k++) {
        ASSERT_EQ(ya[k], yb[k]) << k;
    }
    resamp_free(&a);
    resamp_free(&b);
}
