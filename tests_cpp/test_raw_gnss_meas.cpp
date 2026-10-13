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
