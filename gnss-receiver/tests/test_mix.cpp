#include "mix.h"
#include "tone.h"

#include <gtest/gtest.h>

TEST(Mix, ShiftsATonePhaseContinuouslyAcrossUnevenBlocks)
{
    const double fs = 18.48e6, f0 = 250e3, df = -7.134e6;  // the wide files' L1 offset
    const size_t n = 50000;
    auto x = tone::make(f0, fs, n);
    mix_t m;
    mix_init(&m, df, fs, 0);
    size_t sizes[] = {1, 1023, 1024, 1025, 7, 4096, 3};
    size_t k = 0, i = 0;
    while (k < n) {
        size_t b = std::min(sizes[i++ % 7], n - k);
        mix_apply(&m, x.data() + 2 * k, b);
        k += b;
    }
    auto f = tone::fit(x.data(), 0, n, f0 + m.f_hz, fs);
    EXPECT_NEAR(std::abs(f.amp), 1.0, 1e-6);
    EXPECT_NEAR(std::arg(f.amp), 0.0, 1e-6);
    EXPECT_LT(f.resid_db, -120.0);
    EXPECT_NEAR(m.f_hz, df, 1e-6);
}

TEST(Mix, StartingMidStreamMatchesTheFullRun)
{
    const double fs = 2.6e6, df = 22.0;
    const size_t n = 30000, s = 12345;
    auto a = tone::make(100e3, fs, n);
    auto b = std::vector<float>(a.begin() + 2 * s, a.end());
    mix_t ma, mb;
    mix_init(&ma, df, fs, 0);
    mix_init(&mb, df, fs, s);
    mix_apply(&ma, a.data(), n);
    mix_apply(&mb, b.data(), n - s);
    for (size_t k = 0; k < n - s; k += 997) {
        EXPECT_NEAR(a[2 * (k + s)], b[2 * k], 1e-5);
        EXPECT_NEAR(a[2 * (k + s) + 1], b[2 * k + 1], 1e-5);
    }
}
