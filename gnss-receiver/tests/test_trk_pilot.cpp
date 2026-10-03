// A pilot channel (Galileo E1-C, BOC(1,1), 4 ms dumps) on synthetic dumps with the BOC(1,1)
// correlation function on all five taps and the contract's three-period command delay: it locks,
// and one parked on a side peak jumps to the main peak (milestone 6).

extern "C" {
#include "gnss/sig.h"
#include "gnss/trk.h"
#include "rng.h"
}

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr double kFs = 6.75e6, kT = 4092.0 / 1.023e6;

// BOC(1,1) autocorrelation, unlimited bandwidth.
double rboc(double x)
{
    x = std::fabs(x);
    return x <= 0.5 ? 1.0 - 3.0 * x : (x <= 1.0 ? x - 1.0 : 0.0);
}

struct PilotSim {
    trk_ch_t ch{};
    rng_t rng{};
    double dop = 1234.0, tau, phase = 0.3, f_q[3], r_q[3];
    const double carr_k = 4294967296.0 / kFs, code_k = 1099511627776.0 / kFs;
    const int32_t if_word = int32_t(std::llround(-2.658052e6 / kFs * 4294967296.0));
    const uint64_t code_word0 = uint64_t(std::llround(1.023e6 / kFs * 1099511627776.0));

    explicit PilotSim(double tau0, uint64_t seed = 3) : tau(tau0)
    {
        rng_seed(&rng, seed);
        trk_start(&ch, 13, float(dop + 3.0), 0.1f, if_word, code_word0, float(carr_k), float(code_k));
        trk_set_signal(&ch, GNSS_SIG_GAL_E1C);
        for (int k = 0; k < 3; k++) {
            f_q[k] = dop + 3.0;
            r_q[k] = 1.023e6 * (1.0 + (dop + 3.0) / GNSS_FREQ_L1_HZ);
        }
    }

    void step(uint32_t k, double cn0)
    {
        // The period's carrier phase error at mid-period, and the code error.
        const double f_nco = f_q[0], r_nco = r_q[0], r_true = 1.023e6 * (1.0 + dop / GNSS_FREQ_L1_HZ);
        phase += 2.0 * M_PI * (dop - f_nco) * kT;
        tau += (r_true - r_nco) * kT;
        const double sig = std::sqrt(1.0 / (2.0 * std::pow(10.0, cn0 / 10.0) * kT));
        const double c = std::cos(phase), s = std::sin(phase);
        double n[10];
        for (int j = 0; j < 10; j += 2) {
            rng_gauss2(&rng, &n[j], &n[j + 1]);
        }
        corr_dump_t d{};
        d.seq = k;
        auto tap = [&](double off, float &i, float &q, int j) {
            double r = rboc(tau + off);
            i = float(r * c + sig * n[j]);
            q = float(r * s + sig * n[j + 1]);
        };
        tap(0.0, d.ip, d.qp, 0);
        tap(-0.1, d.ie, d.qe, 2);   // early: the replica 0.1 chip ahead sees R(tau - 0.1)
        tap(0.1, d.il, d.ql, 4);
        tap(-0.5, d.ive, d.qve, 6);
        tap(0.5, d.ivl, d.qvl, 8);
        int b;
        uint32_t bp;
        trk_update(&ch, &trk_profile_quiet, &d, float(kT), &b, &bp);
        int32_t cw;
        uint64_t kw;
        trk_words(&ch, &cw, &kw);
        f_q[0] = f_q[1];
        f_q[1] = f_q[2];
        f_q[2] = double(cw - if_word) / carr_k;
        r_q[0] = r_q[1];
        r_q[1] = r_q[2];
        r_q[2] = 1.023e6 + double(int64_t(kw - code_word0)) / code_k;
    }
};

}  // namespace

TEST(TrkPilot, LocksOnTheMainPeak)
{
    PilotSim s(0.05);
    for (uint32_t k = 0; k < 750; k++) {  // 3 s
        s.step(k, 40.0);
    }
    EXPECT_EQ(s.ch.state, TRK_LOCKED);
    EXPECT_TRUE(s.ch.locked_once);
    EXPECT_LT(std::fabs(s.tau), 0.03);
    EXPECT_EQ(s.ch.n_jumps, 0);
    EXPECT_NEAR(s.ch.cn0, 40.0f, 1.5f);
}

TEST(TrkPilot, JumpsOffASidePeak)
{
    for (double tau0 : {0.5, -0.5}) {
        PilotSim s(tau0);
        for (uint32_t k = 0; k < 1000; k++) {  // 4 s
            s.step(k, 40.0);
        }
        EXPECT_GE(s.ch.n_jumps, 1) << tau0;
        EXPECT_LT(std::fabs(s.tau), 0.03) << tau0;
        EXPECT_EQ(s.ch.state, TRK_LOCKED) << tau0;
    }
}
