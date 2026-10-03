#include "sig_gen.h"

extern "C" {
#include "gnss/acq.h"
}

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

double chip_diff(double a, double b)
{
    return std::remainder(a - b, double(GPS_CA_LEN));
}

}  // namespace

TEST(Acq, FindsASatelliteAt42DbHzAndRejectsAnAbsentOne)
{
    const double fs = 6.75e6, if_hz = 1.2e6;
    siggen::Sat a;
    a.prn = 5;
    a.dop = 2362.0;
    a.code_phase = 612.37;
    a.phase = 1.0;
    a.data_ms = 20;
    siggen::Sat b;
    b.prn = 19;
    b.dop = -1598.0;
    b.code_phase = 3.9;
    const size_t n = size_t(fs * 0.010);
    auto x = siggen::make({a, b}, fs, if_hz, n, siggen::sigma_for(42.0, fs));

    acq_cfg_t cfg{};
    cfg.fs = fs;
    cfg.if_hz = if_hz;
    cfg.n_fft = 2048;
    cfg.n_ms = 10;
    cfg.dop_max = 5000.0;
    std::vector<float> work(acq_work_floats(cfg.n_fft, cfg.n_ms));
    acq_t q;
    ASSERT_EQ(acq_prepare(&q, &cfg, x.data(), n, work.data()), 0);

    acq_result_t r;
    acq_search(&q, 5, &r);
    EXPECT_GT(r.metric, 6.0f);
    EXPECT_NEAR(r.dop_hz, 2362.0, 250.0);       // coarse: one grid step
    EXPECT_NEAR(chip_diff(r.code_phase, 612.37), 0.0, 0.5);
    acq_refine(&q, &r);
    EXPECT_NEAR(r.dop_hz, 2362.0, 40.0);
    EXPECT_NEAR(chip_diff(r.code_phase, 612.37), 0.0, 0.05);

    acq_search(&q, 19, &r);
    EXPECT_GT(r.metric, 6.0f);
    acq_refine(&q, &r);
    EXPECT_NEAR(r.dop_hz, -1598.0, 40.0);
    EXPECT_NEAR(chip_diff(r.code_phase, 3.9), 0.0, 0.05);

    acq_search(&q, 30, &r);  // not there
    EXPECT_LT(r.metric, 3.0f);
}

TEST(Acq, WorksFromTheGpsSdrSimRate)
{
    const double fs = 2.6e6;
    siggen::Sat a;
    a.prn = 22;
    a.dop = 250.0;
    a.code_phase = 1000.6;
    const size_t n = size_t(fs * 0.010);
    auto x = siggen::make({a}, fs, 0.0, n, siggen::sigma_for(45.0, fs));
    acq_cfg_t cfg{};
    cfg.fs = fs;
    cfg.n_fft = 2048;
    cfg.n_ms = 10;
    cfg.dop_max = 5000.0;
    std::vector<float> work(acq_work_floats(cfg.n_fft, cfg.n_ms));
    acq_t q;
    ASSERT_EQ(acq_prepare(&q, &cfg, x.data(), n, work.data()), 0);
    acq_result_t r;
    acq_search(&q, 22, &r);
    EXPECT_GT(r.metric, 6.0f);
    acq_refine(&q, &r);
    EXPECT_NEAR(r.dop_hz, 250.0, 40.0);
    EXPECT_NEAR(chip_diff(r.code_phase, 1000.6), 0.0, 0.05);
}
