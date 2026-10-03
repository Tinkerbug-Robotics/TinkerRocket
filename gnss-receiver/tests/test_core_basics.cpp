extern "C" {
#include "gnss/fft.h"
#include "gnss/gmath.h"
#include "gnss/sig.h"
}

#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <vector>

TEST(Codes, FirstTenChipsMatchTheIcd)
{
    // IS-GPS-200 Table 3-Ia, first 10 chips in octal.
    const int want[32] = {01440, 01620, 01710, 01744, 01133, 01455, 01131, 01454, 01626, 01504, 01642,
                          01750, 01764, 01772, 01775, 01776, 01156, 01467, 01633, 01715, 01746, 01763,
                          01063, 01706, 01743, 01761, 01770, 01774, 01127, 01453, 01625, 01712};
    for (int prn = 1; prn <= 32; prn++) {
        uint8_t c[GPS_CA_LEN];
        ASSERT_EQ(gps_ca_code(prn, c), 0);
        int v = 0;
        for (int k = 0; k < 10; k++) {
            v = (v << 1) | c[k];
        }
        EXPECT_EQ(v, want[prn - 1]) << "PRN " << prn;
    }
    uint8_t c[GPS_CA_LEN];
    EXPECT_EQ(gps_ca_code(0, c), -1);
    EXPECT_EQ(gps_ca_code(33, c), -1);
}

TEST(Codes, GoldCodeCorrelationProperties)
{
    uint8_t a[GPS_CA_LEN], b[GPS_CA_LEN];
    gps_ca_code(1, a);
    gps_ca_code(2, b);
    // Balanced: 512 ones, 511 zeros. Cross-correlation of Gold codes takes three values.
    int ones = 0;
    for (int k = 0; k < GPS_CA_LEN; k++) {
        ones += a[k];
    }
    EXPECT_EQ(ones, 512);
    for (int lag = 0; lag < GPS_CA_LEN; lag += 37) {
        int s = 0;
        for (int k = 0; k < GPS_CA_LEN; k++) {
            s += (a[k] ^ b[(k + lag) % GPS_CA_LEN]) ? -1 : 1;
        }
        EXPECT_TRUE(s == -1 || s == -65 || s == 63) << lag << " " << s;
    }
}

TEST(Gmath, Atan2MatchesLibmEverywhere)
{
    double worst = 0.0;
    for (int i = 0; i < 20000; i++) {
        double a = -M_PI + 2.0 * M_PI * (i + 0.5) / 20000.0;
        for (double r : {1e-3, 1.0, 1e4}) {
            float y = float(r * std::sin(a)), x = float(r * std::cos(a));
            double e = std::fabs(std::remainder(double(gnss_atan2f(y, x)) - std::atan2(double(y), double(x)), 2 * M_PI));
            worst = std::max(worst, e);
        }
    }
    EXPECT_LT(worst, 3e-7);
    EXPECT_EQ(gnss_atan2f(0.0f, 0.0f), 0.0f);
    // The half-range form ignores a common sign flip.
    EXPECT_NEAR(gnss_atan_halff(0.3f, -1.0f), gnss_atan_halff(-0.3f, 1.0f), 1e-7);
    EXPECT_NEAR(gnss_atan_halff(-0.3f, 1.0f), std::atan(-0.3), 3e-7);
}

TEST(Fft, MatchesADirectDft)
{
    const int n = 256;
    std::vector<float> tw(n);
    fft_plan_t p;
    ASSERT_EQ(fft_plan_init(&p, n, tw.data()), 0);
    std::vector<float> x(2 * n);
    std::vector<std::complex<double>> ref(n);
    for (int k = 0; k < n; k++) {
        x[2 * k] = float(std::sin(0.37 * k) + 0.1 * k / n);
        x[2 * k + 1] = float(std::cos(1.3 * k * k / n));
    }
    for (int f = 0; f < n; f++) {
        std::complex<double> s = 0.0;
        for (int k = 0; k < n; k++) {
            s += std::complex<double>(x[2 * k], x[2 * k + 1]) * std::polar(1.0, -2.0 * M_PI * f * k / n);
        }
        ref[f] = s;
    }
    auto y = x;
    fft_run(&p, y.data(), 0);
    for (int f = 0; f < n; f++) {
        EXPECT_NEAR(y[2 * f], ref[f].real(), 2e-4);
        EXPECT_NEAR(y[2 * f + 1], ref[f].imag(), 2e-4);
    }
    fft_run(&p, y.data(), 1);
    for (int k = 0; k < 2 * n; k++) {
        EXPECT_NEAR(y[k] / n, x[k], 1e-5);
    }
    EXPECT_EQ(fft_plan_init(&p, 100, tw.data()), -1);
}
