// Host tests for the carrier wind-up (TR_RawGnssEKF/RawGnssWindup.h) and the
// covariance's exact attitude transition (RawGnssEKFParams::att_exact): the
// windup study, 2026-10-18. The model's sign against the spin rig's
// measurement, Wu's rate form against roll alone and its bound across the
// boresight, the two-band and the one-band roll rates, the gyro's history,
// the monitor's leaky mean; and a fast roll that the first-order transition
// turns into a tilt variance at its cap, where the exact one only turns it.
// (The shadow's correction end to end: test_raw_gnss_shadow.cpp.)
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "DopplerChange.h"
#include "RawGnssWindup.h"
#include "RawMeas.h"
#include "ReceiverProfile.h"

// attExactBlocks is the filter's own (private): read here directly, its float blocks against its double ones
#define private public
#include "TR_RawGnssEKF.h"
#undef private

namespace w = gnss_raw::windup;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kL1 = 299792458.0 / 1575.42e6;
constexpr double kB1I = 299792458.0 / 1561.098e6;
constexpr double kL5 = 299792458.0 / 1176.45e6;

}  // namespace

TEST(Windup, TheRigsClockwiseTurnFromAboveIsPlusOneCycleRangeLike)
{
    // The spin rig (10/16), the Goblin's helix nose up: +0.991 / +0.994 range-like cycles per clockwise turn seen
    // from above. A clockwise turn from above is a negative turn about the boresight (up): p = -2 pi rad/s for a
    // second winds +k cycles.
    const double k = 0.99;
    const double cycles = w::rollRate(kL1, k, -2.0 * kPi) * 1.0 / kL1;
    EXPECT_NEAR(cycles, +k, 1e-12);
    // and the size the filter sees: 0.19 m/s per rev/s on L1/E1, 1.7 mm/s per rev/s more on B1I
    EXPECT_NEAR(w::rollRate(kL1, 1.0, 2.0 * kPi), -0.190294, 1e-6);
    EXPECT_NEAR(w::rollRate(kB1I, 1.0, 2.0 * kPi) - w::rollRate(kL1, 1.0, 2.0 * kPi), -0.001745, 2e-6);
}

TEST(Windup, RollAloneIsTheSameOnEverySatelliteInFrontOrBehind)
{
    std::mt19937 rng(7);
    std::normal_distribution<double> n(0.0, 1.0);
    const double p = 10.0 * 2.0 * kPi;
    const double wb[3] = {p, 0.0, 0.0};
    int checked = 0;
    for (int i = 0; i < 2000; ++i)
    {
        double e[3] = {n(rng), n(rng), n(rng)};
        const double m = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        for (double& x : e)
            x /= m;
        if (1.0 + e[0] < 0.1)
            continue;                     // the null behind the antenna: held, not Wu's there
        EXPECT_NEAR(w::wuRate(kL1, 0.99, wb, e), w::rollRate(kL1, 0.99, p), 1e-12);
        checked++;
    }
    EXPECT_GT(checked, 1800);
}

TEST(Windup, TheRateAcrossTheBoresightIsEachSatellitesOwnAndBounded)
{
    // at most (lambda / 2pi) |w_perp| tan(theta / 2): 30 mm/s per rad/s at 90 deg off the boresight
    std::mt19937 rng(8);
    std::normal_distribution<double> n(0.0, 1.0);
    for (int i = 0; i < 2000; ++i)
    {
        double e[3] = {n(rng), n(rng), n(rng)};
        const double m = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        for (double& x : e)
            x /= m;
        if (1.0 + e[0] < 0.1)
            continue;
        const double wp[3] = {0.0, n(rng), n(rng)};
        const double th = std::acos(e[0]);
        const double bound = kL1 / (2.0 * kPi) * std::sqrt(wp[1] * wp[1] + wp[2] * wp[2]) * std::tan(th / 2.0);
        EXPECT_LE(std::fabs(w::wuRate(kL1, 1.0, wp, e)), bound * (1.0 + 1e-9) + 1e-15);
    }
    const double side[3] = {0.0, 1.0, 0.0};
    const double wy[3] = {0.0, 1.0, 0.0};
    EXPECT_NEAR(std::fabs(w::wuRate(kL1, 1.0, wy, side)), kL1 / (2.0 * kPi), 1e-12);   // 30.3 mm/s per rad/s
}

TEST(Windup, TwoBandsOfOneSatelliteGiveTheRollRateFreeOfClockAndGeometry)
{
    // rr_a - rr_b = -k (lambda_a - lambda_b) p / 2pi: L1-L5 64.5 mm of lever
    const double p = 3.0 * 2.0 * kPi, k = 0.99;
    const double geo = 1234.5, clk = -80.25;     // the satellite's motion and the clock: common to both bands
    const double a = geo + clk + w::rollRate(kL1, k, p);
    const double b = geo + clk + w::rollRate(kL5, k, p);
    EXPECT_NEAR(w::gfRoll(a, kL1, b, kL5, k), p, 1e-9);
    // its 1-sigma from 30 and 60 mm/s Dopplers: about 1 rev/s a satellite-epoch
    const double s = w::gfRollSigma(0.03, kL1, 0.06, kL5, k);
    EXPECT_NEAR(s / (2.0 * kPi), 0.0671 / (0.99 * (kL5 - kL1)), 0.01);
    EXPECT_GT(s / (2.0 * kPi), 0.9);
    EXPECT_LT(s / (2.0 * kPi), 1.2);
}

TEST(Windup, OneBandAndTwoWavelengthsInTheSkyGiveTheRollRateItsDopplersDifferBy)
{
    std::mt19937 rng(9);
    std::uniform_real_distribution<double> u01(-1.0, 1.0);
    std::normal_distribution<double> n(0.0, 1.0);
    const double k = 0.99, p = 15.0 * 2.0 * kPi;
    const double dv[3] = {0.4, -0.3, 0.2}, clk = 81.0;
    gnss_raw::windup::InterbandFit fit, same;
    for (int i = 0; i < 16; ++i)
    {
        double u[3] = {u01(rng), u01(rng), -std::fabs(u01(rng)) - 0.1};
        const double m = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
        for (double& x : u)
            x /= m;
        const double lam = i % 3 == 0 ? kB1I : kL1;
        const double y = -(u[0] * dv[0] + u[1] * dv[1] + u[2] * dv[2]) + clk + w::rollRate(lam, k, p);
        fit.add(u, y, 0.05, -k * (lam - kL1) / (2.0 * kPi));
        same.add(u, y, 0.05, 0.0);
    }
    double pe = 0.0, se = 0.0;
    ASSERT_TRUE(fit.solve(pe, se));
    EXPECT_NEAR(pe, p, 1e-6 * p);
    // a dozen Dopplers at 5 cm/s, a third on B1I: about 15 rev/s 1-sigma an epoch -- only a gross disagreement
    EXPECT_GT(se / (2.0 * kPi), 8.0);
    EXPECT_LT(se / (2.0 * kPi), 30.0);
    // one wavelength: no roll in it to see
    EXPECT_FALSE(same.solve(pe, se));
}

TEST(Windup, TheGyrosHistoryGivesTheStepThatHoldsAnInstantAndTheAngleTurned)
{
    w::RateRing<4> ring;
    const float a[3] = {1.0f, 0.0f, 0.0f}, b[3] = {2.0f, 0.5f, 0.0f}, c[3] = {3.0f, 0.0f, 0.0f};
    float out[3];
    ring.push(10.00, a);
    EXPECT_FALSE(ring.rateAt(10.0, out));       // one step: its span's start is not known
    ring.push(10.01, b);
    ring.push(10.02, c);
    EXPECT_TRUE(ring.rateAt(10.015, out));
    EXPECT_EQ(out[0], 3.0f);
    EXPECT_TRUE(ring.rateAt(10.01, out));       // a step's end is its own
    EXPECT_EQ(out[0], 2.0f);
    EXPECT_EQ(out[1], 0.5f);
    EXPECT_TRUE(ring.rateAt(10.05, out));       // past the newest: the newest
    EXPECT_EQ(out[0], 3.0f);
    EXPECT_FALSE(ring.rateAt(10.0, out));       // the oldest step's span is not held
    double dphi = 0.0;
    ASSERT_TRUE(ring.rollOver(10.005, 10.02, dphi));
    EXPECT_NEAR(dphi, 2.0 * 0.005 + 3.0 * 0.01, 1e-9);
    // full: the oldest goes
    ring.push(10.03, a);
    ring.push(10.04, a);
    EXPECT_EQ(ring.size(), 4);
    EXPECT_FALSE(ring.rateAt(10.005, out));
    EXPECT_FALSE(ring.rollOver(10.005, 10.02, dphi));
}

TEST(Windup, TheMonitorsLeakyMeanAndTheGyrosScaleAsTheWindupSeesIt)
{
    w::Monitor m(1.0);
    for (int i = 0; i < 50; ++i)
        m.add(0.1 * i, 0.98 * 60.0 + (i % 2 ? 5.0 : -5.0), 5.0, 60.0);
    EXPECT_NEAR(m.diff(), -0.02 * 60.0, 0.6);
    EXPECT_GT(m.diffSigma(), 0.0);
    EXPECT_NEAR(m.scale(), 0.98, 0.01);
    // the leak: an old disagreement fades
    w::Monitor f(1.0);
    for (int i = 0; i < 10; ++i)
        f.add(0.1 * i, 50.0, 1.0, 0.0);
    for (int i = 10; i < 60; ++i)
        f.add(0.1 * i, 0.0, 1.0, 0.0);
    EXPECT_LT(std::fabs(f.diff()), 1.0);
}

namespace
{

// The attitude block's 1-sigma about body y and z after 2 s of pure roll at rev_s, 100 steps a second, level on
// a pad with no update: the transition's work alone (plus the gyro's own noise)
void rollFor(double rev_s, bool exact, double& sig_yz_deg, double& sig_x_deg)
{
    gnss_raw::RawGnssEKFParams prm;
    prm.att_exact = exact;
    prm.p0_wbias = 1e-6;                 // a gyro bias across the nose would wind less under the roll: kept out
    gnss_raw::RawGnssEKF<float> f(prm);
    const double lla[3] = {0.6, -1.9, 100.0};
    const float v[3] = {0.0f, 0.0f, 0.0f};
    const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    f.init(lla, v, q, 0.0);
    f.setQuaternion(q);                  // the attitude known: what grows is the transition's and the gyro's noise
    const float acc[3] = {0.0f, 0.0f, -9.807f};
    const float gyro[3] = {static_cast<float>(rev_s * 2.0 * kPi), 0.0f, 0.0f};
    for (int i = 0; i < 200; ++i)
        f.propagate(acc, gyro, 0.01f);
    // the attitude state is a half-angle: twice its sigma is the angle's
    sig_yz_deg = 2.0 * std::sqrt(0.5 * (f.P(7, 7) + f.P(8, 8))) * 180.0 / kPi;
    sig_x_deg = 2.0 * std::sqrt(f.P(6, 6)) * 180.0 / kPi;
}

}  // namespace

TEST(RawGnssEKFExact, AFastRollTurnsTheTiltVarianceInsteadOfStretchingIt)
{
    // First order, I - [w x] dt, stretches the variance across the rate by 1 + (|w| dt)^2 a step: at 5 rev/s and
    // 100 steps a second that is x19,000 a second, so from a known attitude the 1-sigma across the nose reaches its
    // cap (362 deg) within the 2 s; exact, the roll turns that variance about the nose and leaves its size: the
    // gyro's own noise over 2 s, as with no roll
    double s0, x0, s1, x1, sr, xr;
    rollFor(0.0, false, s0, x0);
    rollFor(5.0, false, s1, x1);
    rollFor(5.0, true, sr, xr);
    EXPECT_LT(s0, 1.0);
    EXPECT_GT(s1, 300.0);
    EXPECT_NEAR(sr, s0, 0.05 * s0);
    EXPECT_NEAR(xr, x0, 0.05 * x0);
    // and at 1 rev/s, x1.5 a second first order: the noise the 2 s gathered, stretched
    double s2, x2, s3, x3;
    rollFor(1.0, false, s2, x2);
    rollFor(1.0, true, s3, x3);
    EXPECT_GT(s2, 1.15 * s3);
    EXPECT_NEAR(s3, s0, 0.05 * s0);
}

TEST(RawGnssEKFExact, AtNoRateTheExactTransitionIsTheFirstOrdersToItsOwnRounding)
{
    // no rate: both are I and -I dt / 2 on the gyro bias
    double s0, x0, s1, x1;
    rollFor(0.0, false, s0, x0);
    rollFor(0.0, true, s1, x1);
    EXPECT_NEAR(s1, s0, 1e-4 * s0);
    EXPECT_NEAR(x1, x0, 1e-4 * x0);
}

TEST(RawGnssEKFExact, TheFloatBlocksAreTheDoubleOnesWithinAMillionth)
{
    // The flight filter is RawGnssEKF<float>: its blocks come from sinf and cosf, and 1 - cos th cancels at a
    // small turn -- an absolute error of about 6e-8, against the I beside it. Against the double blocks for the
    // same rate (the float one, widened), a turn th = |w| dt a step from 1e-6 to pi about axes in every direction:
    // both within 1e-6. (Ga's (1 - cos th) / |w| term is off by up to its own size at a small turn, 2.5e-7 at most
    // in a 10 ms step: a term first order leaves out altogether.)
    gnss_raw::RawGnssEKFParams prm;
    const gnss_raw::RawGnssEKF<float> ff(prm);
    const gnss_raw::RawGnssEKF<double> fd(prm);
    std::mt19937 rng(7);
    std::normal_distribution<double> n01(0.0, 1.0);
    const float dt = 0.01f;
    double worst_ra = 0.0, worst_ga = 0.0;
    int n = 0;
    for (double th : {1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 0.1, 0.3, 1.0, 2.0, 3.0, kPi})
        for (int k = 0; k < 20; ++k)
        {
            double a[3] = {n01(rng), n01(rng), n01(rng)};
            const double an = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            float wf[3];
            double wd[3];
            for (int i = 0; i < 3; ++i)
            {
                wf[i] = static_cast<float>(a[i] / an * th / dt);
                wd[i] = wf[i];
            }
            float Rf[3][3], Gf[3][3];
            double Rd[3][3], Gd[3][3];
            ff.attExactBlocks(wf, dt, Rf, Gf);
            fd.attExactBlocks(wd, static_cast<double>(dt), Rd, Gd);
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                {
                    worst_ra = std::max(worst_ra, std::fabs(static_cast<double>(Rf[i][j]) - Rd[i][j]));
                    worst_ga = std::max(worst_ga, std::fabs(static_cast<double>(Gf[i][j]) - Gd[i][j]));
                }
            n++;
        }
    EXPECT_EQ(n, 220);
    EXPECT_LT(worst_ra, 1e-6);
    EXPECT_LT(worst_ga, 1e-6);
}

TEST(RawGnssEKFExact, TheBlocksAreTheTransitionsByItsDefinition)
{
    // F's attitude rows are -[w x] on the attitude and -I/2 on the gyro's bias, so over a step Phi's blocks are
    // Ra = exp(-[w x] dt) and Ga = -1/2 int_0^dt exp(-[w x] s) ds. attExactBlocks (double) against those from their
    // definitions: the exponential as its series (40 terms), the integral by Simpson's rule (400 intervals) on the
    // same series, a turn th = |w| dt a step from 1e-3 to pi about 20 axes each: Ra within 1e-12 and a rotation
    // (Ra Ra' = I), Ga within 1e-12. (The float blocks against these double ones: the test above.)
    gnss_raw::RawGnssEKFParams prm;
    const gnss_raw::RawGnssEKF<double> fd(prm);
    // exp(-[w x] s), its series
    auto expm = [](const double w[3], double s, double E[3][3]) {
        const double A[3][3] = {{0.0, w[2] * s, -w[1] * s}, {-w[2] * s, 0.0, w[0] * s}, {w[1] * s, -w[0] * s, 0.0}};
        double term[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                E[i][j] = term[i][j];
        for (int k = 1; k <= 40; ++k)
        {
            double nt[3][3];
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    nt[i][j] = (term[i][0] * A[0][j] + term[i][1] * A[1][j] + term[i][2] * A[2][j]) / k;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                {
                    term[i][j] = nt[i][j];
                    E[i][j] += nt[i][j];
                }
        }
    };
    std::mt19937 rng(11);
    std::normal_distribution<double> n01(0.0, 1.0);
    const double dt = 0.01;
    const int ns = 400;
    double worst_ra = 0.0, worst_ga = 0.0, worst_orth = 0.0;
    int n = 0;
    for (double th : {1e-3, 1e-2, 0.1, 0.5, 1.0, 2.0, 3.0, kPi})
        for (int k = 0; k < 20; ++k)
        {
            double w[3] = {n01(rng), n01(rng), n01(rng)};
            const double wn = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
            for (double& c : w)
                c *= th / dt / wn;
            double Ra[3][3], Ga[3][3];
            fd.attExactBlocks(w, dt, Ra, Ga);
            double E[3][3], I[3][3] = {};
            expm(w, dt, E);
            for (int m = 0; m <= ns; ++m)
            {
                double Em[3][3];
                expm(w, dt * m / ns, Em);
                const double c = (m == 0 || m == ns) ? 1.0 : (m % 2 == 1 ? 4.0 : 2.0);
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j)
                        I[i][j] += c * Em[i][j] * dt / (3.0 * ns);
            }
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                {
                    worst_ra = std::max(worst_ra, std::fabs(Ra[i][j] - E[i][j]));
                    worst_ga = std::max(worst_ga, std::fabs(Ga[i][j] + 0.5 * I[i][j]));
                    const double rr = Ra[i][0] * Ra[j][0] + Ra[i][1] * Ra[j][1] + Ra[i][2] * Ra[j][2];
                    worst_orth = std::max(worst_orth, std::fabs(rr - (i == j ? 1.0 : 0.0)));
                }
            n++;
        }
    EXPECT_EQ(n, 160);
    EXPECT_LT(worst_ra, 1e-12);
    EXPECT_LT(worst_orth, 1e-12);
    EXPECT_LT(worst_ga, 1e-12);
}
