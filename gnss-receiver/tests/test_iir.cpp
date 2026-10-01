#include "iir.h"
#include "tone.h"

#include <gtest/gtest.h>

TEST(Iir, ButterworthMagnitudeMatchesTheBilinearPrototype)
{
    const double fs = 6.75e6, fc = 2.1e6;
    for (int order : {3, 5}) {
        iir_t f;
        ASSERT_EQ(iir_butter_lowpass(&f, order, fc, fs), 0);
        EXPECT_NEAR(iir_mag(&f, 0.0, fs), 1.0, 1e-12);
        EXPECT_NEAR(iir_mag(&f, fc, fs), std::sqrt(0.5), 1e-9);
        for (double hz : {0.5e6, 1.0e6, 1.8e6, 2.5e6, 3.0e6}) {
            double r = std::tan(0.5 * tone::kTwoPi * hz / fs) / std::tan(0.5 * tone::kTwoPi * fc / fs);
            double want = 1.0 / std::sqrt(1.0 + std::pow(r, 2 * order));
            EXPECT_NEAR(iir_mag(&f, hz, fs), want, 1e-9) << order << " " << hz;
        }
    }
}

TEST(Iir, FilteringAToneScalesItByTheMagnitude)
{
    const double fs = 6.75e6, fc = 2.1e6, f0 = 1.9e6;
    iir_t f;
    ASSERT_EQ(iir_butter_lowpass(&f, 5, fc, fs), 0);
    auto x = tone::make(f0, fs, 20000);
    iir_apply(&f, x.data(), 7000);
    iir_apply(&f, x.data() + 2 * 7000, 13000);
    // Complex input at +f0: both components see the same real filter, so |out| = |H(f0)|.
    auto fit = tone::fit(x.data(), 2000, 20000, f0, fs);
    EXPECT_NEAR(std::abs(fit.amp), iir_mag(&f, f0, fs), 1e-5);
    EXPECT_LT(fit.resid_db, -100.0);
}
