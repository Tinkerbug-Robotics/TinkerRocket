// Host tests for the GNSS pad survey (#1600 phase 5, PadSurvey.h): each
// satellite's pad correction, against a pad whose pseudoranges carry a
// per-satellite bias (the ionosphere it stands for), a receiver clock that
// wanders and steps, and noise -- what it recovers, when a satellite counts,
// a relock, a satellite rising late, a refused residual, the pad moving, and
// correct() in flight.
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "PadSurvey.h"
#include "TR_RawGnssEKF.h"
#include "gnss/types.h"

using gnss_raw::PadSurvey;
using gnss_raw::RawMeas;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
// the pad: the bench fix of tests_cpp/fixtures/raw_meas_golden.h
const double kPad[3] = {1251903.5764443001, -4769240.891127206, 4033008.234123381};
constexpr double kTow0 = 345600.0;     // Thursday 00:00 of the GPS week

struct Sv
{
    uint8_t sys, prn;
    double az, el;   // deg
    double bias;     // m: the ionosphere and the rest the pseudorange carries
};

// The satellites: GPS, Galileo, BeiDou around the sky (geocentric
// elevations, as correct() reckons them).
std::vector<Sv> sky()
{
    return {
        {GNSS_SYS_GPS, 5, 40, 75, 4.1},  {GNSS_SYS_GPS, 10, 130, 35, 6.3},
        {GNSS_SYS_GPS, 18, 220, 20, 8.8}, {GNSS_SYS_GPS, 20, 310, 50, 5.0},
        {GNSS_SYS_GPS, 23, 10, 12, 11.2}, {GNSS_SYS_GPS, 27, 270, 28, 7.4},
        {GNSS_SYS_GAL, 4, 80, 60, 4.9},   {GNSS_SYS_GAL, 9, 190, 40, 6.0},
        {GNSS_SYS_GAL, 31, 350, 25, 7.9}, {GNSS_SYS_BDS, 22, 150, 65, 4.6},
        {GNSS_SYS_BDS, 37, 240, 33, 6.8},
    };
}

void basis(const double r[3], double e[3], double n[3], double u[3])
{
    const double rn = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
    for (int a = 0; a < 3; ++a)
        u[a] = r[a] / rn;
    const double h = std::sqrt(r[0] * r[0] + r[1] * r[1]);
    e[0] = -r[1] / h;
    e[1] = r[0] / h;
    e[2] = 0.0;
    n[0] = u[1] * e[2] - u[2] * e[1];
    n[1] = u[2] * e[0] - u[0] * e[2];
    n[2] = u[0] * e[1] - u[1] * e[0];
}

RawMeas satAt(const Sv& s, const double pad[3])
{
    double e[3], n[3], u[3];
    basis(pad, e, n, u);
    const double az = s.az * kDeg, el = s.el * kDeg, d = 2.2e7;
    RawMeas m{};
    m.sys = s.sys;
    m.prn = s.prn;
    m.has_pr = m.has_rr = true;
    for (int a = 0; a < 3; ++a)
        m.sat_pos[a] = pad[a] + d * (std::cos(el) * std::sin(az) * e[a] + std::cos(el) * std::cos(az) * n[a]
                                     + std::sin(el) * u[a]);
    m.sigma_pr = 1.0f;
    m.sigma_pr_corr = 2.0f;
    m.tau_pr = 15.0f;
    return m;
}

// One epoch: each satellite's pseudorange from the pad, with the receiver
// clock, its system's offset, its bias and noise.
std::vector<RawMeas> epoch(const std::vector<Sv>& svs, const double pad[3], double clock_m, double noise_m,
                           std::mt19937& rng)
{
    std::normal_distribution<double> g(0.0, 1.0);
    const double isb[3] = {0.0, 12.5, -31.0};
    std::vector<RawMeas> out;
    for (const Sv& s : svs)
    {
        RawMeas m = satAt(s, pad);
        m.pr = gnss_raw::sagnacRange(m, pad) + clock_m + isb[s.sys] + s.bias + noise_m * g(rng);
        out.push_back(m);
    }
    return out;
}

// The corrections against the biases, each system's constant taken out.
double worstCorrectionError(const PadSurvey& ps, const std::vector<Sv>& svs)
{
    double worst = 0.0;
    for (int sys = 0; sys < 3; ++sys)
    {
        double dsum = 0.0;
        int k = 0;
        for (const Sv& s : svs)
        {
            double c;
            if (s.sys == sys && ps.correction(s.sys, s.prn, c))
            {
                dsum += c - s.bias;
                k++;
            }
        }
        if (k == 0)
            continue;
        const double off = dsum / k;
        for (const Sv& s : svs)
        {
            double c;
            if (s.sys == sys && ps.correction(s.sys, s.prn, c))
                worst = std::max(worst, std::fabs(c - s.bias - off));
        }
    }
    return worst;
}

std::vector<double> ages(size_t n, double a)
{
    return std::vector<double>(n, a);
}

}  // namespace

TEST(PadSurvey, RecoversEachSatellitesBiasThroughAWanderingClock)
{
    const std::vector<Sv> svs = sky();
    PadSurvey ps;
    std::mt19937 rng(7);
    // the receiver clock drifts 30 m/s and steps once by a millisecond and a bit
    for (int k = 0; k < 300; ++k)
    {
        const double clock = 1.0e5 + 30.0 * k + (k >= 140 ? 299792.458 + 17.0 : 0.0);
        auto m = epoch(svs, kPad, clock, 1.0, rng);
        ps.observe(m.data(), static_cast<int>(m.size()), ages(m.size(), 600.0).data(), kTow0 + k, kPad);
    }
    EXPECT_EQ(ps.summary().tracked, svs.size());
    EXPECT_EQ(ps.summary().settling, 0);
    EXPECT_EQ(ps.summary().corrected, svs.size());
    // 1 m of white noise over a 120 s average: a tenth of a metre or so
    EXPECT_LT(worstCorrectionError(ps, svs), 0.35);
    // their spread about each system's mean is the biases' own
    double ss = 0.0;
    for (int sys = 0; sys < 3; ++sys)
    {
        double sum = 0.0, sum2 = 0.0;
        int k = 0;
        for (const Sv& s : svs)
            if (s.sys == sys)
            {
                sum += s.bias;
                sum2 += s.bias * s.bias;
                k++;
            }
        ss += sum2 - sum * sum / k;
    }
    EXPECT_NEAR(ps.summary().rms_m, std::sqrt(ss / svs.size()), 0.15);
    EXPECT_EQ(ps.summary().last_tow_s, kTow0 + 299);
}

TEST(PadSurvey, ASatelliteCountsOnceSettledAndInUseAfterMinSpan)
{
    const std::vector<Sv> svs = sky();
    PadSurvey ps;
    std::mt19937 rng(1);
    const PadSurvey::Config& c = ps.config();
    // locked 10 s when the survey starts
    for (int k = 0; k < 120; ++k)
    {
        auto m = epoch(svs, kPad, 0.0, 0.5, rng);
        const double age = 10.0 + k;
        ps.observe(m.data(), static_cast<int>(m.size()), ages(m.size(), age).data(), kTow0 + k, kPad);
        if (age < c.settle_s)
        {
            EXPECT_EQ(ps.summary().settling, svs.size()) << k;
            EXPECT_EQ(ps.summary().corrected, 0) << k;
        }
        else if (age - c.settle_s < c.min_span_s)
        {
            EXPECT_EQ(ps.summary().settling, 0) << k;
            EXPECT_EQ(ps.summary().corrected, 0) << k;
        }
        else
        {
            EXPECT_EQ(ps.summary().corrected, svs.size()) << k;
        }
    }
}

TEST(PadSurvey, ARelockStartsASatelliteOver)
{
    const std::vector<Sv> svs = sky();
    PadSurvey ps;
    std::mt19937 rng(2);
    int k = 0;
    for (; k < 100; ++k)
    {
        auto m = epoch(svs, kPad, 0.0, 0.5, rng);
        ps.observe(m.data(), static_cast<int>(m.size()), ages(m.size(), 600.0).data(), kTow0 + k, kPad);
    }
    double c;
    ASSERT_TRUE(ps.correction(GNSS_SYS_GPS, 18, c));
    // G18 missing for 3 epochs: back, it has relocked
    std::vector<Sv> less;
    for (const Sv& s : svs)
        if (!(s.sys == GNSS_SYS_GPS && s.prn == 18))
            less.push_back(s);
    for (; k < 103; ++k)
    {
        auto m = epoch(less, kPad, 0.0, 0.5, rng);
        ps.observe(m.data(), static_cast<int>(m.size()), ages(m.size(), 600.0).data(), kTow0 + k, kPad);
    }
    EXPECT_TRUE(ps.correction(GNSS_SYS_GPS, 18, c));   // frozen while it is away
    auto m = epoch(svs, kPad, 0.0, 0.5, rng);
    ps.observe(m.data(), static_cast<int>(m.size()), ages(m.size(), 600.0).data(), kTow0 + k, kPad);
    EXPECT_FALSE(ps.correction(GNSS_SYS_GPS, 18, c));
    EXPECT_EQ(ps.summary().corrected, svs.size() - 1);
}

TEST(PadSurvey, ASatelliteRisingLateTakesTheOthersReference)
{
    std::vector<Sv> svs = sky();
    const Sv late = svs.back();          // C37
    svs.pop_back();
    PadSurvey ps;
    std::mt19937 rng(3);
    std::vector<Sv> all = svs;
    all.push_back(late);
    for (int k = 0; k < 400; ++k)
    {
        const bool up = k >= 150;
        auto m = epoch(up ? all : svs, kPad, 5.0e4 - 11.0 * k, 0.8, rng);
        std::vector<double> age(m.size(), 600.0);
        if (up)
            age.back() = 60.0 + (k - 150);   // settled, as it rises
        ps.observe(m.data(), static_cast<int>(m.size()), age.data(), kTow0 + k, kPad);
    }
    double c22, c37;
    ASSERT_TRUE(ps.correction(GNSS_SYS_BDS, 22, c22));
    ASSERT_TRUE(ps.correction(GNSS_SYS_BDS, 37, c37));
    // on C22's reference: their difference is their biases'
    EXPECT_NEAR(c37 - c22, late.bias - 4.6, 0.3);
    EXPECT_LT(worstCorrectionError(ps, all), 0.4);
}

TEST(PadSurvey, ARefusedResidualLeavesTheCorrection)
{
    const std::vector<Sv> svs = sky();
    PadSurvey ps;
    std::mt19937 rng(4);
    int k = 0;
    for (; k < 100; ++k)
    {
        auto m = epoch(svs, kPad, 0.0, 0.0, rng);
        ps.observe(m.data(), static_cast<int>(m.size()), ages(m.size(), 600.0).data(), kTow0 + k, kPad);
    }
    double before, after;
    ASSERT_TRUE(ps.correction(GNSS_SYS_GPS, 23, before));
    auto m = epoch(svs, kPad, 0.0, 0.0, rng);
    for (RawMeas& x : m)
        if (x.sys == GNSS_SYS_GPS && x.prn == 23)
            x.pr += 120.0;
    ps.observe(m.data(), static_cast<int>(m.size()), ages(m.size(), 600.0).data(), kTow0 + k, kPad);
    ASSERT_TRUE(ps.correction(GNSS_SYS_GPS, 23, after));
    EXPECT_NEAR(after, before, 1e-9);
}

TEST(PadSurvey, ThePadMovingStartsItOver)
{
    const std::vector<Sv> svs = sky();
    PadSurvey ps;
    std::mt19937 rng(5);
    int k = 0;
    for (; k < 100; ++k)
    {
        auto m = epoch(svs, kPad, 0.0, 0.5, rng);
        ps.observe(m.data(), static_cast<int>(m.size()), ages(m.size(), 600.0).data(), kTow0 + k, kPad);
    }
    EXPECT_EQ(ps.summary().corrected, svs.size());
    // the averaged fix creeping by 5 m is not a move
    const double creep[3] = {kPad[0] + 3.0, kPad[1] - 3.0, kPad[2] + 2.5};
    auto m = epoch(svs, creep, 0.0, 0.5, rng);
    ps.observe(m.data(), static_cast<int>(m.size()), ages(m.size(), 600.0).data(), kTow0 + k++, creep);
    EXPECT_EQ(ps.summary().corrected, svs.size());
    const double moved[3] = {creep[0] + 30.0, creep[1], creep[2]};
    m = epoch(svs, moved, 0.0, 0.5, rng);
    ps.observe(m.data(), static_cast<int>(m.size()), ages(m.size(), 600.0).data(), kTow0 + k, moved);
    EXPECT_EQ(ps.summary().corrected, 0);
}

TEST(PadSurvey, CorrectTakesTheCorrectionOutOrWidensTheSigma)
{
    std::vector<Sv> svs = sky();
    PadSurvey ps;
    std::mt19937 rng(6);
    for (int k = 0; k < 100; ++k)
    {
        auto m = epoch(svs, kPad, 0.0, 0.0, rng);
        ps.observe(m.data(), static_cast<int>(m.size()), ages(m.size(), 600.0).data(), kTow0 + k, kPad);
    }
    // in flight: two satellites risen since, one at the zenith, one at 5 degrees
    svs.push_back({GNSS_SYS_GPS, 30, 0, 90, 3.0});
    svs.push_back({GNSS_SYS_GAL, 12, 90, 5, 14.0});
    auto m = epoch(svs, kPad, 0.0, 0.0, rng);
    const std::vector<RawMeas> before = m;
    EXPECT_EQ(ps.correct(m.data(), static_cast<int>(m.size()), kPad), static_cast<int>(svs.size()) - 2);
    for (size_t i = 0; i + 2 < m.size(); ++i)
    {
        double c;
        ASSERT_TRUE(ps.correction(m[i].sys, m[i].prn, c));
        EXPECT_DOUBLE_EQ(m[i].pr, before[i].pr - c);
        EXPECT_EQ(m[i].sigma_pr_corr, before[i].sigma_pr_corr);
    }
    // the zenith one keeps 5 m (the obliquity factor is 1), the low one ~3x that
    const RawMeas& z = m[m.size() - 2];
    const RawMeas& lo = m.back();
    EXPECT_EQ(z.pr, before[m.size() - 2].pr);
    EXPECT_NEAR(z.sigma_pr_corr, std::sqrt(4.0 + 25.0), 0.01);
    const double e = 0.53 - 5.0 / 180.0;
    const double f = 1.0 + 16.0 * e * e * e;
    EXPECT_NEAR(lo.sigma_pr_corr, std::sqrt(4.0 + 25.0 * f * f), 0.05);
    EXPECT_GT(f, 2.9);
    // the corrected pseudoranges, less their pad ranges, now agree per system
    // to the constant the filter's clock takes up
    double spread = 0.0;
    double ref[3] = {NAN, NAN, NAN};
    for (size_t i = 0; i + 2 < m.size(); ++i)
    {
        const double r = m[i].pr - gnss_raw::sagnacRange(m[i], kPad);
        if (std::isnan(ref[m[i].sys]))
            ref[m[i].sys] = r;
        spread = std::max(spread, std::fabs(r - ref[m[i].sys]));
    }
    EXPECT_LT(spread, 1e-6);
}

TEST(PadSurvey, WithNoCorrectionInUseCorrectLeavesEveryMeasurementAlone)
{
    const std::vector<Sv> svs = sky();
    PadSurvey ps;
    std::mt19937 rng(8);
    // settled, but not yet min_span_s behind them
    for (int k = 0; k < 10; ++k)
    {
        auto m = epoch(svs, kPad, 0.0, 0.5, rng);
        ps.observe(m.data(), static_cast<int>(m.size()), ages(m.size(), 600.0).data(), kTow0 + k, kPad);
    }
    ASSERT_EQ(ps.summary().corrected, 0);
    auto m = epoch(svs, kPad, 0.0, 0.5, rng);
    const std::vector<RawMeas> before = m;
    EXPECT_EQ(ps.correct(m.data(), static_cast<int>(m.size()), kPad), 0);
    for (size_t i = 0; i < m.size(); ++i)
    {
        EXPECT_EQ(m[i].pr, before[i].pr);
        EXPECT_EQ(m[i].sigma_pr_corr, before[i].sigma_pr_corr);
    }
}

namespace
{

// The pad as the receiver's average gives it: offset from the true one by
// east, north, up (m).
std::vector<double> padOff(double e_m, double n_m, double u_m)
{
    double e[3], n[3], u[3];
    basis(kPad, e, n, u);
    std::vector<double> p(3);
    for (int a = 0; a < 3; ++a)
        p[a] = kPad[a] + e_m * e[a] + n_m * n[a] + u_m * u[a];
    return p;
}

}  // namespace

TEST(PadSurvey, ARawSolutionNearTheReceiversPadVouchesForIt)
{
    // the receiver's average 10 m off the truth: the ionosphere and the
    // receiver's own error are of that order, and the pad is used
    const std::vector<Sv> svs = sky();
    const std::vector<double> pad = padOff(6.0, -3.0, 7.0);
    PadSurvey ps;
    std::mt19937 rng(9);
    for (int k = 0; k < 120; ++k)
    {
        auto m = epoch(svs, kPad, 0.0, 0.5, rng);   // satellites seen from the true pad
        ps.observe(m.data(), static_cast<int>(m.size()), ages(m.size(), 600.0).data(), kTow0 + k, pad.data());
    }
    EXPECT_TRUE(ps.summary().pad_ok);
    EXPECT_LT(ps.summary().pad_offset_m, 20.0f);
    auto m = epoch(svs, kPad, 0.0, 0.5, rng);
    EXPECT_EQ(ps.correct(m.data(), static_cast<int>(m.size()), kPad), static_cast<int>(svs.size()));
}

TEST(PadSurvey, AReceiverFixFarFromTheRawMeasurementsCorrectsNothing)
{
    // 2026-10-13 on the bench: the LC86G's fix 316 m below and 70 m east of
    // where its own pseudoranges put it
    const std::vector<Sv> svs = sky();
    const std::vector<double> pad = padOff(67.6, 18.0, -316.4);
    PadSurvey ps;
    std::mt19937 rng(10);
    for (int k = 0; k < 120; ++k)
    {
        auto m = epoch(svs, kPad, 0.0, 0.5, rng);
        ps.observe(m.data(), static_cast<int>(m.size()), ages(m.size(), 600.0).data(), kTow0 + k, pad.data());
    }
    EXPECT_EQ(ps.summary().corrected, svs.size());   // surveyed all the same
    EXPECT_FALSE(ps.summary().pad_ok);
    // where the raw measurements put the pad: the offset, less the biases' share
    EXPECT_NEAR(ps.summary().pad_offset_m, std::sqrt(67.6 * 67.6 + 18.0 * 18.0 + 316.4 * 316.4), 15.0);
    auto m = epoch(svs, kPad, 0.0, 0.5, rng);
    const std::vector<RawMeas> before = m;
    EXPECT_EQ(ps.correct(m.data(), static_cast<int>(m.size()), kPad), 0);
    for (size_t i = 0; i < m.size(); ++i)
    {
        EXPECT_EQ(m[i].pr, before[i].pr);
        EXPECT_EQ(m[i].sigma_pr_corr, before[i].sigma_pr_corr);
    }
}

TEST(PadSurvey, TooFewSatellitesToVouchForThePadCorrectNothing)
{
    // two systems, five satellites: three position unknowns and two clocks
    // leave no redundancy to judge the pad by
    std::vector<Sv> svs;
    for (const Sv& s : sky())
        if (s.sys == GNSS_SYS_GPS && svs.size() < 3)
            svs.push_back(s);
    svs.push_back({GNSS_SYS_GAL, 4, 80, 60, 4.9});
    svs.push_back({GNSS_SYS_GAL, 9, 190, 40, 6.0});
    PadSurvey ps;
    std::mt19937 rng(11);
    for (int k = 0; k < 100; ++k)
    {
        auto m = epoch(svs, kPad, 0.0, 0.5, rng);
        ps.observe(m.data(), static_cast<int>(m.size()), ages(m.size(), 600.0).data(), kTow0 + k, kPad);
    }
    EXPECT_EQ(ps.summary().corrected, svs.size());
    EXPECT_FALSE(ps.summary().pad_ok);
    auto m = epoch(svs, kPad, 0.0, 0.5, rng);
    EXPECT_EQ(ps.correct(m.data(), static_cast<int>(m.size()), kPad), 0);
}
