// A pilot channel (Galileo E1-C, BOC(1,1), 4 ms dumps; or the B1C pilot, 10 ms) on synthetic dumps
// with the BOC(1,1) correlation function on all five taps and the contract's three-period command
// delay: it locks, one parked on a side peak jumps to the main peak (milestone 6), and its C/N0
// from consecutive dumps holds a weak pilot and drops a lost one.

extern "C" {
#include "gnss/sig.h"
#include "gnss/trk.h"
#include "rng.h"
}

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr double kFs = 6.75e6;

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
    double T, amp = 1.0;  // the dump's length, s; the signal's amplitude (0: gone)
    const double carr_k = 4294967296.0 / kFs, code_k = 1099511627776.0 / kFs;
    const int32_t if_word = int32_t(std::llround(-2.658052e6 / kFs * 4294967296.0));
    const uint64_t code_word0 = uint64_t(std::llround(1.023e6 / kFs * 1099511627776.0));

    explicit PilotSim(double tau0, uint64_t seed = 3, int sig = GNSS_SIG_GAL_E1C)
        : tau(tau0), T((sig == GNSS_SIG_GAL_E1C ? 4092.0 : 10230.0) / 1.023e6)
    {
        rng_seed(&rng, seed);
        trk_start(&ch, 13, float(dop + 3.0), 0.1f, if_word, code_word0, float(carr_k), float(code_k));
        trk_set_signal(&ch, sig);
        for (int k = 0; k < 3; k++) {
            f_q[k] = dop + 3.0;
            r_q[k] = 1.023e6 * (1.0 + (dop + 3.0) / GNSS_FREQ_L1_HZ);
        }
    }

    void step(uint32_t k, double cn0)
    {
        // The period's carrier phase error at mid-period, and the code error.
        const double f_nco = f_q[0], r_nco = r_q[0], r_true = 1.023e6 * (1.0 + dop / GNSS_FREQ_L1_HZ);
        phase += 2.0 * M_PI * (dop - f_nco) * T;
        tau += (r_true - r_nco) * T;
        const double sig = std::sqrt(1.0 / (2.0 * std::pow(10.0, cn0 / 10.0) * T));
        const double c = amp * std::cos(phase), s = amp * std::sin(phase);
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
        trk_update(&ch, &trk_profile_quiet, &d, float(T), &b, &bp);
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

// 27 dB-Hz, under the 30 dB-Hz line moments of 4 or 10 ms dumps needed: the estimate reads it and the
// channel holds it. The signal gone, the channel is dropped within TRK_LOSS_S and an estimate.
TEST(TrkPilot, HoldsAWeakPilotAndDropsALostOne)
{
    for (int sig : {GNSS_SIG_GAL_E1C, GNSS_SIG_BDS_B1CP}) {
        PilotSim s(0.05, 7, sig);
        const uint32_t n = uint32_t(std::lround(1.0 / s.T));  // dumps a second
        uint32_t k = 0;
        for (; k < 3 * n; k++) {
            s.step(k, 40.0);
        }
        ASSERT_EQ(s.ch.state, TRK_LOCKED) << sig;
        int locked = 0;
        double cn0 = 0.0;
        for (; k < 23 * n; k++) {
            s.step(k, 27.0);
            locked += s.ch.state == TRK_LOCKED;
            cn0 += s.ch.cn0;
            ASSERT_NE(s.ch.state, TRK_OFF) << sig << " at " << k * s.T << " s";
        }
        EXPECT_GT(locked, int(0.97 * 20 * n)) << sig;
        EXPECT_NEAR(cn0 / (20 * n), 27.0, 1.0) << sig;
        s.amp = 0.0;
        const uint32_t k_gone = k;
        for (; k < k_gone + 3 * n && s.ch.state != TRK_OFF; k++) {
            s.step(k, 27.0);
        }
        EXPECT_EQ(s.ch.state, TRK_OFF) << sig;
        EXPECT_LT((k - k_gone) * s.T, TRK_LOSS_S + 0.45) << sig;
    }
}

// Started on an empty sky (an aided start with nothing there), a pilot never counts as locked.
TEST(TrkPilot, AnEmptySkyNeverLocks)
{
    for (int sig : {GNSS_SIG_GAL_E1C, GNSS_SIG_BDS_B1CP}) {
        for (uint64_t seed = 1; seed <= 20; seed++) {
            PilotSim s(0.05, seed, sig);
            s.amp = 0.0;
            for (uint32_t k = 0; k < uint32_t(std::lround(5.0 / s.T)); k++) {
                s.step(k, 40.0);
            }
            EXPECT_FALSE(s.ch.locked_once) << sig << " seed " << seed;
            EXPECT_EQ(s.ch.state, TRK_OFF) << sig << " seed " << seed;
        }
    }
}
