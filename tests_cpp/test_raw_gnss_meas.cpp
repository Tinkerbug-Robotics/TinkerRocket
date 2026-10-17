// Host tests for the raw-GNSS measurement builder (#1600): the LC86G's MSM7
// epochs through the flight code's own parser (Lc86Parser), Lc86RawEpoch.h and
// TR_GNSSRaw's RawMeasBuilder must give the corrected measurements the bench
// prototype's filter took (gnss repo sim/tinkerrocket_gnss gnss_raw.corrected,
// sim/tc_ekf_cocom.py meas_at). fixtures/raw_meas_golden.h holds two
// consecutive epochs of RTCM the Beetle's LC86G sent on the COCOM rig, every
// ephemeris it sent, and the prototype's measurements for the second.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "EphemerisStore.h"
#include "Lc86Parser.h"
#include "Lc86RawEpoch.h"
#include "RawMeasBuilder.h"
#include "Rtcm3Msm.h"
#include "RtcmEphemeris.h"
#include "fixtures/raw_meas_golden.h"
#include "gnss/atmo.h"
#include "gnss/types.h"

using gnss_raw::EphemerisStore;
using gnss_raw::RawEpoch;
using gnss_raw::RawMeas;
using gnss_raw::RawMeasBuilder;

namespace
{

constexpr int kNearWeek = 2440;

// A payload framed as the module sends it: preamble, length, payload, CRC-24Q.
std::vector<uint8_t> frame(const GoldenPayload& p)
{
    std::vector<uint8_t> f(static_cast<size_t>(p.len) + rtcm3::kFrameOverhead);
    f[0] = rtcm3::kPreamble;
    f[1] = static_cast<uint8_t>((p.len >> 8) & 0x03);
    f[2] = static_cast<uint8_t>(p.len & 0xFF);
    for (int i = 0; i < p.len; ++i)
        f[3 + i] = p.bytes[i];
    const uint32_t crc = rtcm3::crc24q(f.data(), static_cast<size_t>(p.len) + 3);
    f[p.len + 3] = static_cast<uint8_t>(crc >> 16);
    f[p.len + 4] = static_cast<uint8_t>(crc >> 8);
    f[p.len + 5] = static_cast<uint8_t>(crc);
    return f;
}

// Both epochs, through the parser as the flight computer reads them.
std::vector<RawEpoch> parseEpochs()
{
    lc86::Lc86Parser parser;
    std::vector<RawEpoch> out;
    for (int i = 0; i < kGoldenRmMsmN; ++i)
        for (uint8_t b : frame(kGoldenRmMsm[i]))
            if (parser.feed(b) == lc86::Event::MSM_EPOCH)
            {
                rtcm3::MsmEpoch me;
                EXPECT_TRUE(parser.takeMsmEpoch(me));
                EXPECT_TRUE(me.complete);
                RawEpoch ep;
                EXPECT_TRUE(lc86::rawEpochFromMsm(me, ep));
                out.push_back(ep);
            }
    return out;
}

EphemerisStore storeAll()
{
    EphemerisStore s;
    for (int i = 0; i < kGoldenRmEphN; ++i)
    {
        gps_eph_t e;
        EXPECT_TRUE(gnss_raw::decodeEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len),
                                              kNearWeek, e));
        s.add(e);
    }
    return s;
}

gps_iono_t goldenIono()
{
    gps_iono_t io{};
    io.valid = 1;
    for (int k = 0; k < 4; ++k)
    {
        io.alpha[k] = kGoldenRmIon[k];
        io.beta[k] = kGoldenRmIon[4 + k];
    }
    return io;
}

const GoldenMeas* goldenFor(int sys, int prn)
{
    for (int i = 0; i < kGoldenRmMeasN; ++i)
        if (kGoldenRmMeas[i].sys == sys && kGoldenRmMeas[i].prn == prn)
            return &kGoldenRmMeas[i];
    return nullptr;
}

}  // namespace

TEST(RawGnssMeas, TheParserGivesTheEpochsThePrototypeRead)
{
    const std::vector<RawEpoch> eps = parseEpochs();
    ASSERT_EQ(eps.size(), 2u);
    EXPECT_DOUBLE_EQ(eps[0].tow_s, kGoldenRmTowA);
    EXPECT_DOUBLE_EQ(eps[1].tow_s, kGoldenRmTowB);
    int matched = 0;
    for (int i = 0; i < kGoldenRmObsN; ++i)
    {
        const GoldenObs& g = kGoldenRmObs[i];
        for (int k = 0; k < eps[1].n; ++k)
        {
            const gnss_raw::RawObs& o = eps[1].obs[k];
            if (o.sys != g.sys || o.prn != g.prn)
                continue;
            SCOPED_TRACE(testing::Message() << "sys " << g.sys << " prn " << g.prn);
            ASSERT_TRUE(o.has_pr && o.has_rate);
            EXPECT_NEAR(o.pr_m, g.pr, 1e-6);
            EXPECT_NEAR(o.rate_mps, g.rate, 5e-4);   // the driver keeps the rate in float
            EXPECT_FLOAT_EQ(o.cn0_dbhz, static_cast<float>(g.cn0));
            matched++;
        }
    }
    EXPECT_EQ(matched, kGoldenRmObsN);
}

TEST(RawGnssMeas, BuildsTheMeasurementsThePrototypeFiltered)
{
    const std::vector<RawEpoch> eps = parseEpochs();
    ASSERT_EQ(eps.size(), 2u);
    const EphemerisStore store = storeAll();
    RawMeasBuilder b;                        // the LC86G defaults
    b.setIono(goldenIono());
    b.noteEpoch(eps[0]);
    RawMeas m[RawEpoch::kMaxObs];
    const int n = b.build(eps[1], store, kGoldenRmRx, kGoldenRmGPeak, m, RawEpoch::kMaxObs);
    ASSERT_EQ(n, kGoldenRmMeasN);
    for (int i = 0; i < n; ++i)
    {
        SCOPED_TRACE(testing::Message() << "sys " << int(m[i].sys) << " prn " << int(m[i].prn));
        const GoldenMeas* g = goldenFor(m[i].sys, m[i].prn);
        ASSERT_NE(g, nullptr);
        for (int k = 0; k < 3; ++k)
        {
            EXPECT_NEAR(m[i].sat_pos[k], g->sat_pos[k], 1e-3);
            EXPECT_NEAR(m[i].sat_vel[k], g->sat_vel[k], 1e-5);
            EXPECT_NEAR(m[i].sat_acc[k], g->sat_acc[k], 1e-5);
        }
        // the C core's dry troposphere constant is 0.0022768, the prototype's
        // 0.0022767: up to 2 mm near the mask
        EXPECT_NEAR(m[i].pr, g->pr, 3e-3);
        ASSERT_TRUE(m[i].has_rr);
        EXPECT_NEAR(m[i].rr, g->rr, 5e-4);
        EXPECT_NEAR(m[i].sigma_pr, g->sigma_pr, 1e-6 * g->sigma_pr);
        EXPECT_NEAR(m[i].sigma_pr_corr, g->sigma_pr_corr, 1e-6 * g->sigma_pr_corr);
        EXPECT_NEAR(m[i].sigma_rr, g->sigma_rr, 1e-6 * g->sigma_rr);
        EXPECT_FLOAT_EQ(m[i].tau_pr, 15.0f);
        EXPECT_FLOAT_EQ(m[i].tau_rr, 0.1f);
    }
}

TEST(RawGnssMeas, WithoutAPositionNothingIsMaskedOrCorrectedForTheAtmosphere)
{
    const std::vector<RawEpoch> eps = parseEpochs();
    const EphemerisStore store = storeAll();
    RawMeasBuilder b;
    b.setIono(goldenIono());
    RawMeas with[RawEpoch::kMaxObs], without[RawEpoch::kMaxObs];
    const int n0 = b.build(eps[1], store, nullptr, 0.0, without, RawEpoch::kMaxObs);
    EXPECT_EQ(n0, kGoldenRmUnmaskedN);
    // from the other side of the Earth every satellite is below the mask
    const double antipode[3] = {-kGoldenRmRx[0], -kGoldenRmRx[1], -kGoldenRmRx[2]};
    EXPECT_EQ(b.build(eps[1], store, antipode, 0.0, with, RawEpoch::kMaxObs), 0);
    // at the receiver the atmosphere comes off: ionosphere and troposphere both delay the code
    const int n1 = b.build(eps[1], store, kGoldenRmRx, 0.0, with, RawEpoch::kMaxObs);
    ASSERT_EQ(n1, n0);
    for (int i = 0; i < n1; ++i)
    {
        EXPECT_LT(with[i].pr, without[i].pr - 2.0);
        EXPECT_GT(with[i].pr, without[i].pr - 100.0);
    }
}

TEST(RawGnssMeas, ASatelliteMissingLongerThanTheGapHasRelocked)
{
    RawMeasBuilder b;                         // gap 2.5 s
    RawEpoch ep;
    ep.n = 1;
    ep.obs[0] = gnss_raw::RawObs{GNSS_SYS_GPS, 7, true, true, 40.0f, 2.2e7, 100.0, 0};
    for (int s = 0; s <= 5; ++s)
    {
        ep.tow_s = 1000.0 + s;
        b.noteEpoch(ep);
    }
    EXPECT_DOUBLE_EQ(b.relockAge(GNSS_SYS_GPS, 7, 1005.0), 5.0);
    ep.tow_s = 1007.0;                        // 2 s missing: still the same lock
    b.noteEpoch(ep);
    EXPECT_DOUBLE_EQ(b.relockAge(GNSS_SYS_GPS, 7, 1007.0), 7.0);
    ep.tow_s = 1011.0;                        // 4 s missing: relocked
    b.noteEpoch(ep);
    EXPECT_DOUBLE_EQ(b.relockAge(GNSS_SYS_GPS, 7, 1011.0), 0.0);
    EXPECT_DOUBLE_EQ(b.relockAge(GNSS_SYS_GPS, 8, 1011.0), 0.0);   // never seen
    // across the end of the GPS week
    ep.tow_s = 604799.0;
    b.reset();
    b.noteEpoch(ep);
    ep.tow_s = 0.0;
    b.noteEpoch(ep);
    EXPECT_DOUBLE_EQ(b.relockAge(GNSS_SYS_GPS, 7, 1.0), 2.0);
}

TEST(RawGnssMeas, Lc86EpochKeepsOneL1SignalOfGpsGalileoAndBeiDou)
{
    rtcm3::MsmEpoch me{};
    me.gps_tow_ms = 123456789;
    auto cell = [](uint8_t gnss, uint8_t sv, uint8_t sig, double pr, float rate) {
        rtcm3::MsmCell c{};
        c.pseudorange_m = pr;
        c.phase_range_m = pr;
        c.rate_mps = rate;
        c.cn0_dbhz = 42.0f;
        c.lock_ms = 1000;
        c.gnss_id = gnss;
        c.sv_id = sv;
        c.sig_id = sig;
        return c;
    };
    me.cells[me.n_cells++] = cell(0, 5, 2, 2.1e7, -300.0f);          // GPS L1 C/A
    me.cells[me.n_cells++] = cell(0, 6, 15, 2.1e7, -300.0f);         // GPS, another signal
    me.cells[me.n_cells++] = cell(6, 3, 2, 2.1e7, -300.0f);          // GLONASS
    me.cells[me.n_cells++] = cell(2, 11, 2, 2.4e7, 150.0f);          // Galileo E1
    me.cells[me.n_cells++] = cell(3, 25, 2, 2.3e7, std::nanf(""));   // BeiDou B1I, no rate
    me.cells[me.n_cells++] = cell(5, 2, 2, 3.7e7, 10.0f);            // QZSS
    me.cells[me.n_cells++] = cell(0, 9, 2, std::nan(""), 12.0f);     // GPS, no pseudorange
    RawEpoch ep;
    ASSERT_TRUE(lc86::rawEpochFromMsm(me, ep));
    EXPECT_DOUBLE_EQ(ep.tow_s, 123456.789);
    ASSERT_EQ(ep.n, 4);
    EXPECT_EQ(ep.obs[0].sys, GNSS_SYS_GPS);
    EXPECT_EQ(ep.obs[0].prn, 5);
    EXPECT_TRUE(ep.obs[0].has_pr && ep.obs[0].has_rate);
    EXPECT_EQ(ep.obs[1].sys, GNSS_SYS_GAL);
    EXPECT_EQ(ep.obs[2].sys, GNSS_SYS_BDS);
    EXPECT_TRUE(ep.obs[2].has_pr);
    EXPECT_FALSE(ep.obs[2].has_rate);
    EXPECT_EQ(ep.obs[3].prn, 9);
    EXPECT_FALSE(ep.obs[3].has_pr);
    me.gps_tow_ms = rtcm3::kInvalidTow;      // GLONASS alone: no GPS time
    EXPECT_FALSE(lc86::rawEpochFromMsm(me, ep));
    EXPECT_EQ(ep.n, 0);
}

TEST(RawGnssMeas, AnEpochTakesThePvtWeekAcrossTheWeeksEnd)
{
    // the same week
    EXPECT_DOUBLE_EQ(lc86::epochGpsSeconds(345600000u, 2440, 345599000u), 2440 * 604800.0 + 345600.0);
    // the epoch just after the week ended, the latest $PQTMPVT just before
    EXPECT_DOUBLE_EQ(lc86::epochGpsSeconds(1000u, 2440, 604799000u), 2441 * 604800.0 + 1.0);
    // and the other way round
    EXPECT_DOUBLE_EQ(lc86::epochGpsSeconds(604799000u, 2441, 1000u), 2440 * 604800.0 + 604799.0);
}

TEST(RawGnssMeas, Lc86EpochCarriesThePhaseItsHalfCycleFlagAndLockTime)
{
    // The MSM7 phase range (DF397 + DF398 + DF406) as the observation's
    // carrier phase, its half-cycle flag (DF420) beside it; a phase the module
    // marked invalid is none
    rtcm3::MsmEpoch me{};
    me.gps_tow_ms = 123456789;
    auto cell = [](uint8_t gnss, uint8_t sv, double pr, double phase, bool half, uint32_t lock) {
        rtcm3::MsmCell c{};
        c.pseudorange_m = pr;
        c.phase_range_m = phase;
        c.rate_mps = -300.0f;
        c.cn0_dbhz = 42.0f;
        c.lock_ms = lock;
        c.gnss_id = gnss;
        c.sv_id = sv;
        c.sig_id = 2;
        c.half_cycle = half;
        return c;
    };
    me.cells[me.n_cells++] = cell(0, 5, 2.1e7, 2.1e7 + 3.25, false, 557056);           // GPS, locked
    me.cells[me.n_cells++] = cell(2, 11, 2.4e7, std::nan(""), false, 0);               // Galileo, no phase
    me.cells[me.n_cells++] = cell(3, 25, 2.3e7, 2.3e7 - 1.5, true, 64);                // BeiDou, half-cycle unresolved
    RawEpoch ep;
    ASSERT_TRUE(lc86::rawEpochFromMsm(me, ep));
    ASSERT_EQ(ep.n, 3);
    EXPECT_TRUE(ep.obs[0].has_phase);
    EXPECT_DOUBLE_EQ(ep.obs[0].phase_m, 2.1e7 + 3.25);
    EXPECT_FALSE(ep.obs[0].halfcyc);
    EXPECT_EQ(ep.obs[0].lock_ms, 557056u);
    EXPECT_FALSE(ep.obs[1].has_phase);
    EXPECT_EQ(ep.obs[1].phase_m, 0.0);
    EXPECT_TRUE(ep.obs[2].has_phase);
    EXPECT_DOUBLE_EQ(ep.obs[2].phase_m, 2.3e7 - 1.5);
    EXPECT_TRUE(ep.obs[2].halfcyc);
    EXPECT_EQ(ep.obs[2].lock_ms, 64u);
}

TEST(RawGnssMeas, TheBeetlesCarrierPhaseMovesWithItsDoppler)
{
    // The fixture's two epochs, a second apart: where a satellite's phase is
    // continuous across them (a phase at both, the lock time grown by the
    // second, no half cycle pending), its change over the second is its range
    // rate over the second -- the two Dopplers' mean, to their noise -- less a
    // rate common to every satellite (the receiver clock's, which the MSM7
    // Doppler has taken out and the phase has not). Two do here, G10 and G29:
    // the rest have no phase at one end or the other.
    const std::vector<RawEpoch> eps = parseEpochs();
    ASSERT_EQ(eps.size(), 2u);
    std::vector<double> excess;
    int continuous = 0;
    for (int j = 0; j < eps[1].n; ++j)
    {
        const gnss_raw::RawObs& b = eps[1].obs[j];
        for (int i = 0; i < eps[0].n; ++i)
        {
            const gnss_raw::RawObs& a = eps[0].obs[i];
            if (a.sys != b.sys || a.prn != b.prn || !a.has_phase || !b.has_phase || a.halfcyc || b.halfcyc
                || b.lock_ms < a.lock_ms + 900)
                continue;
            continuous++;
            SCOPED_TRACE(testing::Message() << "sys " << int(b.sys) << " prn " << int(b.prn));
            // within a code-minus-carrier's metres of the pseudorange
            EXPECT_NEAR(b.phase_m, b.pr_m, 30.0);
            excess.push_back((b.phase_m - a.phase_m) - 0.5 * (a.rate_mps + b.rate_mps));
        }
    }
    ASSERT_EQ(continuous, 2);
    // the common rate: the receiver's clock, a few m/s here
    EXPECT_LT(std::fabs(excess[0]), 5.0);
    // and each satellite's to the Dopplers' noise: a decimetre a second
    EXPECT_NEAR(excess[0], excess[1], 0.25);
}

// ---- the carrier range rate (RawMeasBuilder's carrierRate)

namespace
{

// An epoch tow - tow0 seconds after the fixture's first: each of its
// satellites with a pseudorange moved on by its range rate, a phase range a
// fixed distance from it, locked lock0 + the time since, at 42 dB-Hz.
RawEpoch carrierEpoch(const RawEpoch& first, double tow, uint32_t lock0 = 60000)
{
    RawEpoch ep;
    ep.tow_s = tow;
    const double dt = tow - first.tow_s;
    for (int i = 0; i < first.n; ++i)
    {
        const gnss_raw::RawObs& a = first.obs[i];
        if (!a.has_pr || !a.has_rate)
            continue;
        gnss_raw::RawObs o = a;
        o.pr_m = a.pr_m + a.rate_mps * dt;
        o.cn0_dbhz = 42.0f;
        o.has_phase = true;
        o.halfcyc = false;
        o.phase_m = o.pr_m + 1000.0 + 7.0 * o.prn;
        o.lock_ms = lock0 + static_cast<uint32_t>(std::lround(dt * 1000.0));
        ep.obs[ep.n++] = o;
    }
    return ep;
}

const RawMeas* findMeas(const RawMeas* m, int n, int sys, int prn)
{
    for (int i = 0; i < n; ++i)
        if (m[i].sys == sys && m[i].prn == prn)
            return &m[i];
    return nullptr;
}

int countCr(const RawMeas* m, int n)
{
    int c = 0;
    for (int i = 0; i < n; ++i)
        c += m[i].has_cr ? 1 : 0;
    return c;
}

}  // namespace

TEST(RawGnssMeas, TheCarrierRateIsThePhaseRangesChangeCorrectedAsTheCodeIs)
{
    const std::vector<RawEpoch> eps = parseEpochs();
    const EphemerisStore store = storeAll();
    RawMeasBuilder b;                          // the LC86G's: carrier rates built
    RawMeas m0[RawEpoch::kMaxObs], m1[RawEpoch::kMaxObs];
    // no receiver position: the satellite clock alone corrects either
    const int n0 = b.build(carrierEpoch(eps[0], eps[0].tow_s), store, nullptr, 0.0, m0, RawEpoch::kMaxObs);
    ASSERT_GE(n0, 8);
    EXPECT_EQ(countCr(m0, n0), 0);             // the first epoch: nothing to difference
    const int n1 = b.build(carrierEpoch(eps[0], eps[0].tow_s + 1.0), store, nullptr, 0.0, m1, RawEpoch::kMaxObs);
    ASSERT_EQ(n1, n0);
    EXPECT_EQ(countCr(m1, n1), n1);
    for (int i = 0; i < n1; ++i)
    {
        SCOPED_TRACE(testing::Message() << "sys " << int(m1[i].sys) << " prn " << int(m1[i].prn));
        const RawMeas* p = findMeas(m0, n0, m1[i].sys, m1[i].prn);
        ASSERT_NE(p, nullptr);
        EXPECT_FLOAT_EQ(m1[i].cr_dt, 1.0f);
        // the phase sits a fixed distance from the code here: its change is
        // the corrected code's (the satellite clock in both)
        EXPECT_NEAR(m1[i].cr_dphi_m, m1[i].pr - p->pr, 1e-6);
        for (int k = 0; k < 3; ++k)
            EXPECT_EQ(m1[i].sat_pos_prev[k], p->sat_pos[k]);
        // both ends' phase sigma at 42 dB-Hz, over the second, above the floor
        const gnss_raw::GnssSignalProfile& sig = b.config().sig[m1[i].sys];
        const double s = std::sqrt(2.0) * gnss_raw::phaseSigmaM(sig, 42.0f);
        EXPECT_NEAR(m1[i].sigma_cr, std::fmax(s, 0.008), 1e-7);
    }
    // with the receiver: the ionosphere, which advances the phase, comes off
    // with the other sign -- a millimetre's change over a second here
    RawMeasBuilder c;
    c.build(carrierEpoch(eps[0], eps[0].tow_s), store, kGoldenRmRx, 0.0, m0, RawEpoch::kMaxObs);
    const int n2 = c.build(carrierEpoch(eps[0], eps[0].tow_s + 1.0), store, kGoldenRmRx, 0.0, m1, RawEpoch::kMaxObs);
    ASSERT_GT(n2, 4);
    EXPECT_EQ(countCr(m1, n2), n2);
    for (int i = 0; i < n2; ++i)
    {
        const RawMeas* p = findMeas(m0, n2, m1[i].sys, m1[i].prn);
        ASSERT_NE(p, nullptr);
        EXPECT_NEAR(m1[i].cr_dphi_m, m1[i].pr - p->pr, 0.01);
    }
}

TEST(RawGnssMeas, AClimbsTroposphereComesOffTheCarrierRate)
{
    // 300 m up in the second: the troposphere's delay falls by a decimetre at
    // the zenith, more lower down, and the rate takes it out (a climbing
    // rocket's carrier would otherwise read it as a vertical velocity)
    const std::vector<RawEpoch> eps = parseEpochs();
    const EphemerisStore store = storeAll();
    double lat, lon, h, r_up[3];
    ecef_to_geo(kGoldenRmRx, &lat, &lon, &h);
    geo_to_ecef(lat, lon, h + 300.0, r_up);
    RawMeasBuilder::Config no_trop;
    no_trop.troposphere = false;
    RawMeasBuilder with, without(no_trop);
    RawMeas a0[RawEpoch::kMaxObs], a1[RawEpoch::kMaxObs], b0[RawEpoch::kMaxObs], b1[RawEpoch::kMaxObs];
    with.build(carrierEpoch(eps[0], eps[0].tow_s), store, kGoldenRmRx, 0.0, a0, RawEpoch::kMaxObs);
    without.build(carrierEpoch(eps[0], eps[0].tow_s), store, kGoldenRmRx, 0.0, b0, RawEpoch::kMaxObs);
    const int na = with.build(carrierEpoch(eps[0], eps[0].tow_s + 1.0), store, r_up, 0.0, a1, RawEpoch::kMaxObs);
    const int nb = without.build(carrierEpoch(eps[0], eps[0].tow_s + 1.0), store, r_up, 0.0, b1, RawEpoch::kMaxObs);
    ASSERT_EQ(na, nb);
    ASSERT_GT(na, 4);
    for (int i = 0; i < na; ++i)
    {
        SCOPED_TRACE(testing::Message() << "sys " << int(a1[i].sys) << " prn " << int(a1[i].prn));
        ASSERT_TRUE(a1[i].has_cr && b1[i].has_cr);
        const double trop_fall = a1[i].cr_dphi_m - b1[i].cr_dphi_m;
        EXPECT_GT(trop_fall, 0.05);
        EXPECT_LT(trop_fall, 2.0);             // 15 times the zenith's near the 3 deg mask
    }
}

TEST(RawGnssMeas, ACarrierRateNeedsAContinuousLockAtBothEnds)
{
    const std::vector<RawEpoch> eps = parseEpochs();
    const EphemerisStore store = storeAll();
    const double t0 = eps[0].tow_s;
    RawMeas m[RawEpoch::kMaxObs];
    auto rateOf = [&](RawMeasBuilder& b, const RawEpoch& e, int idx) -> bool {
        const int n = b.build(e, store, nullptr, 0.0, m, RawEpoch::kMaxObs);
        const RawMeas* x = findMeas(m, n, e.obs[idx].sys, e.obs[idx].prn);
        return x != nullptr && x->has_cr;
    };
    const RawEpoch e0 = carrierEpoch(eps[0], t0), e1 = carrierEpoch(eps[0], t0 + 1.0);
    {
        RawMeasBuilder b;                      // the reference: a rate
        rateOf(b, e0, 0);
        EXPECT_TRUE(rateOf(b, e1, 0));
    }
    {
        RawMeasBuilder b;                      // the lock time went back: a relock
        rateOf(b, e0, 0);
        RawEpoch e = e1;
        e.obs[0].lock_ms = 500;
        const int n = b.build(e, store, nullptr, 0.0, m, RawEpoch::kMaxObs);
        const RawMeas* x0 = findMeas(m, n, e.obs[0].sys, e.obs[0].prn);
        const RawMeas* x1 = findMeas(m, n, e.obs[1].sys, e.obs[1].prn);
        ASSERT_TRUE(x0 != nullptr && x1 != nullptr);
        EXPECT_FALSE(x0->has_cr);
        EXPECT_TRUE(x1->has_cr);               // the others are kept
    }
    {
        RawMeasBuilder b;                      // locked under min_lock_ms at the earlier end
        RawEpoch e = carrierEpoch(eps[0], t0, 400);
        rateOf(b, e, 0);
        EXPECT_FALSE(rateOf(b, carrierEpoch(eps[0], t0 + 1.0, 400), 0));
        EXPECT_TRUE(rateOf(b, carrierEpoch(eps[0], t0 + 2.0, 400), 0));   // 1,400 ms then
    }
    {
        RawMeasBuilder b;                      // the later end's lock younger than the interval
        rateOf(b, e0, 0);
        RawEpoch e = e1;
        for (int i = 0; i < e.n; ++i)
            e.obs[i].lock_ms = 60000;          // grown less than the second since: relocked between
        e.obs[0].lock_ms = 900;
        EXPECT_FALSE(rateOf(b, e, 0));
    }
    for (int end = 0; end < 2; ++end)
    {
        RawMeasBuilder b;                      // a half cycle pending at either end
        RawEpoch a = e0, c = e1;
        (end == 0 ? a : c).obs[0].halfcyc = true;
        rateOf(b, a, 0);
        EXPECT_FALSE(rateOf(b, c, 0)) << "end " << end;
    }
    for (int end = 0; end < 2; ++end)
    {
        RawMeasBuilder b;                      // under the system's carrier floor at either end
        RawEpoch a = e0, c = e1;
        const float floor = b.config().sig[a.obs[0].sys].cn0_min_cr;
        (end == 0 ? a : c).obs[0].cn0_dbhz = floor - 0.5f;
        rateOf(b, a, 0);
        EXPECT_FALSE(rateOf(b, c, 0)) << "end " << end;
        a.obs[0].cn0_dbhz = c.obs[0].cn0_dbhz = floor;   // at the floor: taken
        RawMeasBuilder d;
        rateOf(d, a, 0);
        EXPECT_TRUE(rateOf(d, c, 0)) << "end " << end;
    }
    {
        RawMeasBuilder b;                      // no phase at the earlier end
        RawEpoch a = e0;
        a.obs[0].has_phase = false;
        rateOf(b, a, 0);
        EXPECT_FALSE(rateOf(b, e1, 0));
    }
    {
        RawMeasBuilder b;                      // a gap: two seconds is not the epoch before
        rateOf(b, e0, 0);
        EXPECT_FALSE(rateOf(b, carrierEpoch(eps[0], t0 + 2.0), 0));
        EXPECT_TRUE(rateOf(b, carrierEpoch(eps[0], t0 + 3.0), 0));   // and from there it goes on
    }
    {
        RawMeasBuilder b;                      // missing for an epoch: it starts over
        rateOf(b, e0, 0);
        RawEpoch e = e1;
        const gnss_raw::RawObs missing = e.obs[0];
        e.obs[0] = e.obs[--e.n];
        b.build(e, store, nullptr, 0.0, m, RawEpoch::kMaxObs);
        RawEpoch back = carrierEpoch(eps[0], t0 + 2.0);
        const int n = b.build(back, store, nullptr, 0.0, m, RawEpoch::kMaxObs);
        const RawMeas* x = findMeas(m, n, missing.sys, missing.prn);
        ASSERT_NE(x, nullptr);
        EXPECT_FALSE(x->has_cr);
        EXPECT_GT(countCr(m, n), 0);           // the others went on
    }
    {
        RawMeasBuilder b;                      // an epoch only noted, not built: nothing to difference
        rateOf(b, e0, 0);
        b.noteEpoch(e1);
        EXPECT_FALSE(rateOf(b, carrierEpoch(eps[0], t0 + 2.0), 0));
    }
    {
        RawMeasBuilder b;                      // a position at one end, none at the other
        b.build(e0, store, kGoldenRmRx, 0.0, m, RawEpoch::kMaxObs);
        EXPECT_FALSE(rateOf(b, e1, 0));
        // and an ionosphere set between two epochs
        RawMeasBuilder c;
        c.build(e0, store, kGoldenRmRx, 0.0, m, RawEpoch::kMaxObs);
        c.setIono(goldenIono());
        const int n = c.build(e1, store, kGoldenRmRx, 0.0, m, RawEpoch::kMaxObs);
        EXPECT_EQ(countCr(m, n), 0);
        const int n2 = c.build(carrierEpoch(eps[0], t0 + 2.0), store, kGoldenRmRx, 0.0, m, RawEpoch::kMaxObs);
        EXPECT_EQ(countCr(m, n2), n2);
    }
    {
        RawMeasBuilder::Config off;            // the profile's mode off: none at all
        off.carrier = false;
        RawMeasBuilder b(off);
        b.build(e0, store, nullptr, 0.0, m, RawEpoch::kMaxObs);
        const int n = b.build(e1, store, nullptr, 0.0, m, RawEpoch::kMaxObs);
        EXPECT_EQ(countCr(m, n), 0);
    }
    {
        RawMeasBuilder b;                      // reset forgets every end
        rateOf(b, e0, 0);
        b.reset();
        EXPECT_FALSE(rateOf(b, e1, 0));
    }
}

TEST(RawGnssMeas, AnEpochBuiltInPartsGivesTheSameCarrierRates)
{
    const std::vector<RawEpoch> eps = parseEpochs();
    const EphemerisStore store = storeAll();
    const RawEpoch e0 = carrierEpoch(eps[0], eps[0].tow_s), e1 = carrierEpoch(eps[0], eps[0].tow_s + 1.0);
    RawMeas whole[RawEpoch::kMaxObs], parts[RawEpoch::kMaxObs];
    RawMeasBuilder a, b;
    a.build(e0, store, kGoldenRmRx, 0.0, whole, RawEpoch::kMaxObs);
    const int n = a.build(e1, store, kGoldenRmRx, 0.0, whole, RawEpoch::kMaxObs);
    for (const RawEpoch* e : {&e0, &e1})
    {
        b.begin(*e, kGoldenRmRx, 0.0);
        int k = 0;
        while (!b.done(*e))
            k = b.next(*e, store, parts, k, RawEpoch::kMaxObs, 2);
        ASSERT_EQ(k, n);
    }
    EXPECT_EQ(countCr(whole, n), n);
    for (int i = 0; i < n; ++i)
    {
        EXPECT_EQ(parts[i].has_cr, whole[i].has_cr);
        EXPECT_EQ(parts[i].cr_dphi_m, whole[i].cr_dphi_m);
        EXPECT_EQ(parts[i].sigma_cr, whole[i].sigma_cr);
    }
    // an epoch begun and left half built keeps the last epoch's ends
    RawMeas again[RawEpoch::kMaxObs];
    RawMeasBuilder c;
    c.build(e0, store, kGoldenRmRx, 0.0, again, RawEpoch::kMaxObs);
    c.begin(e1, kGoldenRmRx, 0.0);
    c.next(e1, store, again, 0, RawEpoch::kMaxObs, 3);   // abandoned
    const int n2 = c.build(e1, store, kGoldenRmRx, 0.0, again, RawEpoch::kMaxObs);
    EXPECT_EQ(countCr(again, n2), n2);
}

TEST(RawGnssMeas, TheBeetlesOwnEpochsGiveTwoCarrierRates)
{
    // The fixture's second epoch built after its first: G10 and G29 had a
    // continuous phase (above), the rest none at one end or the other. Their
    // rates against the Dopplers: a common clock rate and a decimetre's noise.
    const std::vector<RawEpoch> eps = parseEpochs();
    const EphemerisStore store = storeAll();
    RawMeasBuilder b;
    b.setIono(goldenIono());
    RawMeas m0[RawEpoch::kMaxObs], m[RawEpoch::kMaxObs];
    b.build(eps[0], store, kGoldenRmRx, kGoldenRmGPeak, m0, RawEpoch::kMaxObs);
    const int n = b.build(eps[1], store, kGoldenRmRx, kGoldenRmGPeak, m, RawEpoch::kMaxObs);
    ASSERT_EQ(n, kGoldenRmMeasN);
    std::vector<double> excess;
    for (int i = 0; i < n; ++i)
    {
        if (!m[i].has_cr)
            continue;
        EXPECT_EQ(m[i].sys, GNSS_SYS_GPS);
        EXPECT_TRUE(m[i].prn == 10 || m[i].prn == 29) << int(m[i].prn);
        EXPECT_FLOAT_EQ(m[i].cr_dt, 1.0f);
        const RawMeas* p = findMeas(m0, kGoldenRmMeasN, m[i].sys, m[i].prn);
        ASSERT_NE(p, nullptr);
        excess.push_back(m[i].cr_dphi_m / m[i].cr_dt - 0.5 * (m[i].rr + p->rr));
    }
    ASSERT_EQ(excess.size(), 2u);
    EXPECT_LT(std::fabs(excess[0]), 5.0);
    EXPECT_NEAR(excess[0], excess[1], 0.25);
}
