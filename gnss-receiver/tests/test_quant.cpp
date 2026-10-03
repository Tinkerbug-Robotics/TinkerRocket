#include "fe_format.h"
#include "quant.h"
#include "rng.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

TEST(Quant, AgcHoldsTheMagnitudeDensity)
{
    rng_t r;
    rng_seed(&r, 11);
    const size_t n = 400000;
    std::vector<float> v(2 * n, 0.0f);
    rng_add_noise(&r, v.data(), n, 10.0);
    quant2_t q;
    quant2_init(&q, 0.33, 2000.0, 256);
    std::vector<uint8_t> c(n);
    quant2_apply(&q, v.data(), n / 2, c.data());
    // Measure the density over the second half only, after convergence.
    quant2_t q2 = q;
    q2.total_mag = q2.total_bits = 0;
    q2.fill = 0;
    q2.mag_count = 0;
    quant2_apply(&q2, v.data() + n, n / 2, c.data() + n / 2);
    EXPECT_NEAR(quant2_density(&q2), 0.33, 0.004);
    // For Gaussian noise the 0.33 point is 0.974 sigma.
    EXPECT_NEAR(q2.thr / 10.0, 0.974, 0.02);
}

TEST(Quant, CodesFollowTheFrontEndFormat)
{
    quant2_t q;
    quant2_init(&q, 0.33, 1000.0, 4);
    // Fix the threshold at 1.0 by hand after init.
    q.thr = 1.0;
    float v[8] = {2.0f, 0.5f, -0.5f, -2.0f, 0.0f, -0.0f, 1.5f, -1.5f};
    uint8_t c[4];
    quant2_apply(&q, v, 4, c);
    EXPECT_EQ(c[0], FE_CODE_I_MAG);                                    // I +large, Q +small
    EXPECT_EQ(c[1], FE_CODE_I_SIGN | FE_CODE_Q_SIGN | FE_CODE_Q_MAG);  // I -small, Q -large
    EXPECT_EQ(c[2], 0u);                                                // zeros read as +small
    EXPECT_EQ(c[3], FE_CODE_I_MAG | FE_CODE_Q_SIGN | FE_CODE_Q_MAG);
}
