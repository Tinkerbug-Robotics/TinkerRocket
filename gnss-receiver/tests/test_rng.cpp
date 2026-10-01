#include "rng.h"

#include <gtest/gtest.h>

#include <cmath>

TEST(Rng, GaussianMoments)
{
    rng_t r;
    rng_seed(&r, 42);
    const int n = 1000000;
    double s1 = 0, s2 = 0, s4 = 0, cross = 0;
    for (int k = 0; k < n / 2; k++) {
        double a, b;
        rng_gauss2(&r, &a, &b);
        s1 += a + b;
        s2 += a * a + b * b;
        s4 += a * a * a * a + b * b * b * b;
        cross += a * b;
    }
    double mean = s1 / n, var = s2 / n, kurt = (s4 / n) / (var * var);
    EXPECT_NEAR(mean, 0.0, 0.005);
    EXPECT_NEAR(var, 1.0, 0.005);
    EXPECT_NEAR(kurt, 3.0, 0.03);
    EXPECT_NEAR(cross / (n / 2), 0.0, 0.005);
}

TEST(Rng, SameSeedSameSequence)
{
    rng_t a, b;
    rng_seed(&a, 7);
    rng_seed(&b, 7);
    for (int k = 0; k < 1000; k++) {
        ASSERT_EQ(rng_next(&a), rng_next(&b));
    }
    rng_seed(&b, 8);
    EXPECT_NE(rng_next(&a), rng_next(&b));
}

TEST(Rng, AddNoiseSetsTheVariancePerComponent)
{
    rng_t r;
    rng_seed(&r, 3);
    std::vector<float> v(2 * 200000, 0.0f);
    rng_add_noise(&r, v.data(), 200000, 7.0);
    double si = 0, sq = 0;
    for (size_t k = 0; k < 200000; k++) {
        si += v[2 * k] * v[2 * k];
        sq += v[2 * k + 1] * v[2 * k + 1];
    }
    EXPECT_NEAR(std::sqrt(si / 200000), 7.0, 0.05);
    EXPECT_NEAR(std::sqrt(sq / 200000), 7.0, 0.05);
}
