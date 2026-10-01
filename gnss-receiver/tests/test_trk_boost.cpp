// Tracking under dynamics and weak signals (milestone 5), on synthetic dumps: one channel against a
// Doppler profile, with the contract's command delay (period s steers period s + 3), random data
// bits and Gaussian noise, as host/trksim.c does it.

extern "C" {
#include "gnss/trk.h"
#include "rng.h"
}

#include <gtest/gtest.h>

#include <cmath>
#include <functional>

namespace {

constexpr double kFs = 6.75e6, kT = 1e-3;

struct Sim {
    std::function<double(double)> dop;  // true Doppler (Hz) at time t (s)
    double cn0 = 45.0;
    std::function<bool(double)> signal = [](double) { return true; };
    std::function<double(double)> amp = [](double) { return 1.0; };    // signal amplitude scale
    std::function<double(double)> noise = [](double) { return 1.0; };  // noise sigma scale
    std::function<const trk_profile_t *(double)> prof = [](double) { return &trk_profile_quiet; };
    std::function<double(double)> aid;  // IMU aiding: the line of sight's Doppler rate (Hz/s) the loop is given

    trk_ch_t ch{};
    trk_profile_t cur = trk_profile_quiet;
    double theta = 0.0, phi = 0.0, t = 0.0;
    double f_q[3]{};
    int bit = 1;
    rng_t rng{};
    const double carr_k = 4294967296.0 / kFs, code_k = 1099511627776.0 / kFs;
    const int32_t if_word = int32_t(std::llround(-2.658052e6 / kFs * 4294967296.0));
    const uint64_t code_word0 = uint64_t(std::llround(1.023e6 / kFs * 1099511627776.0));

    explicit Sim(std::function<double(double)> d, uint64_t seed = 1) : dop(std::move(d))
    {
        rng_seed(&rng, seed);
        trk_start(&ch, 1, float(dop(0.0) + 20.0), 0.25f, if_word, code_word0, float(carr_k), float(code_k));
        for (double &f : f_q) {
            f = dop(0.0) + 20.0;
        }
    }

    // One period; returns the carrier phase error (cycles) at its end.
    double step()
    {
        const double f_nco = f_q[0];
        double cr = 0.0, ci = 0.0;
        for (int m = 0; m < 10; m++) {
            const double tm = t + (m + 0.5) * kT / 10;
            theta += dop(tm) * kT / 10;
            phi += f_nco * kT / 10;
            cr += std::cos(2 * M_PI * (theta - phi)) / 10;
            ci += std::sin(2 * M_PI * (theta - phi)) / 10;
        }
        const uint32_t k = uint32_t(std::llround(t / kT));
        if (k % 20 == 0 && rng_uniform(&rng) < 0.5) {
            bit = -bit;
        }
        const double a = signal(t) ? bit * amp(t) : 0.0;
        const double sig = noise(t) * std::sqrt(1.0 / (2.0 * std::pow(10.0, cn0 / 10.0) * kT));
        double n[6];
        rng_gauss2(&rng, &n[0], &n[1]);
        rng_gauss2(&rng, &n[2], &n[3]);
        rng_gauss2(&rng, &n[4], &n[5]);
        corr_dump_t d{};
        d.seq = k;
        d.ip = float(a * cr + sig * n[0]);
        d.qp = float(a * ci + sig * n[1]);
        d.ie = float(0.75 * a * cr + sig * n[2]);  // code centred: early and late see the same
        d.qe = float(0.75 * a * ci + sig * n[3]);
        d.il = float(0.75 * a * cr + sig * n[4]);
        d.ql = float(0.75 * a * ci + sig * n[5]);
        trk_profile_step(&cur, prof(t), float(kT));
        int b;
        uint32_t bp;
        trk_update(&ch, &cur, &d, float(kT), &b, &bp);
        if (aid) {
            ch.ff_rate = float(aid(t + kT));
            ch.ff_lead = float(3.0 * kT);  // as rx_tick sets it: the words land three periods on
        }
        int32_t cw;
        uint64_t kw;
        trk_words(&ch, &cw, &kw);
        f_q[0] = f_q[1];
        f_q[1] = f_q[2];
        f_q[2] = double(cw - if_word) / carr_k;
        t += kT;
        return theta - phi;
    }
};

// A hotshot-like line of sight: 1,600 Hz/s from t0 for 2 s, falling to -190 Hz/s over 0.15 s.
double burnout_dop(double t)
{
    const double t0 = 3.0, t1 = 5.0, t2 = 5.15, r1 = 1600.0, r2 = -190.0;
    if (t < t0) {
        return 0.0;
    }
    if (t < t1) {
        return 0.5 * r1 * (t - t0) * (t - t0) / (t1 - t0);  // the rate ramps from 0 to r1
    }
    const double f1 = 0.5 * r1 * (t1 - t0);
    if (t < t2) {
        const double u = t - t1, k = (r2 - r1) / (t2 - t1);
        return f1 + r1 * u + 0.5 * k * u * u;
    }
    return f1 + 0.5 * (r1 + r2) * (t2 - t1) + r2 * (t - t2);
}

}  // namespace

TEST(TrkBoost, LockIndicatorReadsOneAtLowCn0)
{
    // Per-dump cos 2phi would read SNR / (SNR + 1) = 0.6 at 32 dB-Hz and never lock.
    Sim s([](double) { return 1234.0; });
    s.cn0 = 32.0;
    for (int k = 0; k < 6000; k++) {
        s.step();
    }
    EXPECT_EQ(s.ch.state, TRK_LOCKED);
    EXPECT_GT(s.ch.pll_lock, 0.75f);
    EXPECT_NEAR(s.ch.cn0, 32.0f, 1.5f);
}

// A 14 dB step in the noise floor under a strong signal, as the front end's AGC passes it on: 10 ms
// of the new noise at the old scale, then everything scaled down to the old noise (45 -> 31 dB-Hz).
// The C/N0 estimate whose 0.2 s window straddles the step gets the noise power wrong until the next
// one; the channel must ride that out locked.
TEST(TrkBoost, StaysLockedThroughANoiseStep)
{
    for (uint64_t seed = 1; seed <= 16; seed++) {
        const double t_step = 5.0 + 0.0125 * double(seed);  // across the estimate's window
        Sim s([](double) { return 1234.0; }, seed);
        s.cn0 = 45.0;
        s.amp = [t_step](double t) { return t < t_step + 0.01 ? 1.0 : 0.2; };
        s.noise = [t_step](double t) { return t >= t_step && t < t_step + 0.01 ? 5.0 : 1.0; };
        int unlocked = 0;
        for (int k = 0; k < 8000; k++) {
            s.step();
            unlocked += s.t > 4.0 && s.ch.state != TRK_LOCKED;
        }
        EXPECT_EQ(unlocked, 0) << "seed " << seed;
        EXPECT_NEAR(s.ch.cn0, 31.0f, 1.5f) << "seed " << seed;
    }
}

TEST(TrkBoost, DropsALostSignalAndKeepsBitSyncThroughAShortFade)
{
    Sim s([](double) { return -800.0; });
    s.signal = [](double t) { return t < 5.0 || (t >= 5.3 && t < 9.0); };  // a 0.3 s fade, then gone
    double t_off = -1.0;
    int sync_lost = 0;
    for (int k = 0; k < 12000; k++) {
        s.step();
        if (s.t > 4.0 && s.t < 9.0 && !s.ch.bit_sync) {
            sync_lost++;
        }
        if (s.ch.state == TRK_OFF && t_off < 0.0) {
            t_off = s.t;
        }
    }
    EXPECT_EQ(sync_lost, 0);
    ASSERT_GT(t_off, 9.0);   // not dropped for the fade
    EXPECT_LT(t_off, 10.6);  // dropped within 1.6 s of the signal going
}

TEST(TrkBoost, ProfileWidensAtOnceAndNarrowsSlowly)
{
    trk_profile_t cur = trk_profile_quiet;
    trk_profile_step(&cur, &trk_profile_boost, 1e-3f);
    EXPECT_EQ(cur.locked.pll_bw, trk_profile_boost.locked.pll_bw);
    trk_profile_step(&cur, &trk_profile_quiet, 1e-3f);
    EXPECT_GT(cur.locked.pll_bw, 49.0f);
    for (int k = 0; k < 500; k++) {
        trk_profile_step(&cur, &trk_profile_quiet, 1e-3f);
    }
    // After one time constant: 10 + 40 / e.
    EXPECT_NEAR(cur.locked.pll_bw, 10.0f + 40.0f / 2.71828f, 0.5f);
    for (int k = 0; k < 5000; k++) {
        trk_profile_step(&cur, &trk_profile_quiet, 1e-3f);
    }
    EXPECT_EQ(cur.locked.pll_bw, trk_profile_quiet.locked.pll_bw);
    EXPECT_EQ(cur.locked.fll_bw, 0.0f);
}

// The design in one test: the boost profile rides a burnout's step in Doppler rate with the carrier
// phase intact, where the quiet loops slip.
TEST(TrkBoost, BoostProfileRidesTheBurnoutTheQuietLoopsSlipOn)
{
    auto run = [](bool boost) {
        Sim s(burnout_dop);
        s.cn0 = 42.0;
        if (boost) {
            s.prof = [](double t) { return t >= 2.0 && t < 7.0 ? &trk_profile_boost : &trk_profile_quiet; };
        }
        double e_ref = 0.0;
        int slips = 0, unlocked = 0;
        for (int k = 0; k < 10000; k++) {
            double e = s.step();
            if (s.t < 2.5) {
                e_ref = e;  // the half-cycle lock point the loop settled on
                continue;
            }
            slips = std::max(slips, int(std::lround(std::fabs(e - e_ref) * 2.0)));
            unlocked += s.ch.state != TRK_LOCKED;
        }
        EXPECT_NE(s.ch.state, TRK_OFF);
        return std::make_pair(slips, unlocked);
    };
    auto [qs, qu] = run(false);
    auto [bs, bu] = run(true);
    EXPECT_GT(qs, 0) << "the quiet loops were expected to slip";
    EXPECT_GT(qu, 100);
    EXPECT_EQ(bs, 0);
    EXPECT_EQ(bu, 0);
}

// IMU aiding (milestone 7). Exact aiding leaves the quiet loops nothing to track through a burnout.
// An IMU 5 ms late and 3 % off leaves ~50-100 Hz/s at the step: a 10 Hz loop holds +-45 deg against
// only ~20 Hz/s (an acceleration step's phase error is about its size over wn^2) and slips, where
// the 20 Hz loops the aided boost profile runs (~80 Hz/s) do not.
TEST(TrkBoost, ImuAidingCarriesTheBurnout)
{
    static const trk_profile_t aided20 = {{10.0f, 20.0f, 2.0f}, {0.0f, 20.0f, 0.5f}, 2};
    auto run = [](const trk_profile_t *prof, double lag, double scale, double cn0) {
        Sim s(burnout_dop);
        s.cn0 = cn0;
        s.prof = [prof](double t) { return t >= 2.0 && t < 7.0 ? prof : &trk_profile_quiet; };
        s.aid = [lag, scale](double t) {
            const double dt = 1e-4;
            return scale * (burnout_dop(t - lag + dt) - burnout_dop(t - lag - dt)) / (2.0 * dt);
        };
        double e_ref = 0.0;
        int slips = 0, unlocked = 0;
        for (int k = 0; k < 10000; k++) {
            double e = s.step();
            if (s.t < 2.5) {
                e_ref = e;
                continue;
            }
            slips = std::max(slips, int(std::lround(std::fabs(e - e_ref) * 2.0)));
            unlocked += s.ch.state != TRK_LOCKED;
        }
        return std::make_pair(slips, unlocked);
    };
    for (double cn0 : {42.0, 33.0}) {
        auto [es, eu] = run(&trk_profile_quiet, 0.0, 1.0, cn0);
        EXPECT_EQ(es, 0) << cn0;
        EXPECT_EQ(eu, 0) << cn0;
        auto [qs, qu] = run(&trk_profile_quiet, 0.005, 1.03, cn0);
        EXPECT_GT(qs, 0) << cn0 << ": the quiet loops were expected to slip on the IMU's errors";
        auto [bs, bu] = run(&aided20, 0.005, 1.03, cn0);
        EXPECT_EQ(bs, 0) << cn0;
        EXPECT_EQ(bu, 0) << cn0;
    }
}
