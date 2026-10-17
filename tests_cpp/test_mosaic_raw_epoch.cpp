// Host tests for the mosaic-G5's MeasEpoch adapter and its time tag
// (TR_GNSSReceiverMosaic_Serial/MosaicRawEpoch.h): which signal of which
// satellite becomes a cell (GPS L1 C/A, Galileo E1, BeiDou B1I wherever
// MeasEpoch put it, one per satellite, no GEO, main antenna), the units the
// raw-GNSS builder takes (the range rate's sign checked through
// RawMeasBuilder against the LC86G's golden ephemerides), clock jumps left in
// with the lock counted from them, the lock held to the Type2 clip, carrier
// intervals through the builder (none across a jump, whichever epochs between
// are lost), and the time tag: epochGpsS/dtrMs on the PPS clock line (the
// LC86G driver's GnssTimeSync, fed exact edges) across a -1 ms clock jump and
// a week rollover, with planted whole-second, week, sign and jump errors each
// failing the PPS residual check, and a PVT whose MeasEpoch was lost at a
// jump not anchoring the tag. With
// TR_SBF_PRIVATE_CAPTURE set, both 10/16 V9 captures go through the adapter
// too (statistics only).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <vector>

#include "EphemerisStore.h"
#include "GnssTimeSync.h"
#include "MosaicRawEpoch.h"
#include "RawMeasBuilder.h"
#include "RtcmEphemeris.h"
#include "fixtures/raw_meas_golden.h"
#include "gnss/eph.h"
#include "gnss/types.h"
#include "sbf_meas_build.h"
#include "sbf_private_capture.h"

using namespace mosaic;
using gnss_raw::RawEpoch;
using gnss_raw::RawObs;
using sbftest::MeasEpochSpec;
using sbftest::MeasT1;
using sbftest::MeasT2;

namespace {

constexpr double kC = 299792458.0;
constexpr double kLamL1 = kC / 1575.42e6;
constexpr double kLamB1I = kC / 1561.098e6;

struct Adapted
{
    bool ok = false;
    RawEpoch ep;
    MosaicEpochInfo info;
};

Adapted adapt(const std::vector<uint8_t>& blk, MosaicJumpState& js)
{
    Adapted a;
    a.ok = rawEpochFromMeasEpoch(blk.data(), blk.size(), a.ep, js, a.info);
    return a;
}

Adapted adapt(const MeasEpochSpec& s)
{
    MosaicJumpState js;
    return adapt(sbftest::measEpoch(s), js);
}

const RawObs* cell(const RawEpoch& ep, int sys, int prn)
{
    for (int i = 0; i < ep.n; ++i)
        if (ep.obs[i].sys == sys && ep.obs[i].prn == prn) return &ep.obs[i];
    return nullptr;
}

MeasT1 sat(uint8_t svid, uint8_t sig, uint8_t channel)
{
    MeasT1 t;
    t.rx_channel = channel;
    t.svid = svid;
    t.sig(sig);
    t.codeMm(22000000000LL + svid * 1000);
    t.doppler = -15000000 + svid;    // about -1500 Hz: receding
    t.carrier(-2000 - svid);
    t.cn0 = 128;
    t.lock = 600;
    return t;
}

MeasT2 slave(uint8_t sig, uint8_t antenna = 0)
{
    MeasT2 u;
    u.sig(sig, antenna);
    u.codeOffsetMm(1234);
    u.dopplerOffset(-567);
    u.carrier(890);
    u.lock = 200;
    u.cn0 = 120;
    return u;
}

}  // namespace

TEST(MosaicRawEpoch, PicksTheL1SignalWhereverItSits)
{
    MeasEpochSpec s;
    s.t1.push_back(sat(7, 0, 1));                      // G07 L1CA as master
    MeasT1 g8 = sat(8, 4, 2);                          // G08 with L5 master, L1CA as its Type2
    g8.t2 = {slave(3), slave(0)};
    s.t1.push_back(g8);
    MeasT1 e11 = sat(70 + 11, 20, 3);                  // E11: E5a master, E1 slave
    e11.t2 = {slave(17), slave(21)};
    s.t1.push_back(e11);
    MeasT1 c19 = sat(140 + 19, 30, 4);                 // C19: B3I master, B1I and B1C slaves
    c19.t2 = {slave(13), slave(28)};
    s.t1.push_back(c19);
    MeasT1 c45 = sat(182 + 45, 28, 5);                 // C45 (SVID 227) on B1I
    s.t1.push_back(c45);
    s.t1.push_back(sat(140 + 3, 28, 6));               // C03: GEO
    s.t1.push_back(sat(182 + 60, 28, 7));              // C60 (SVID 242): GEO
    MeasT1 c30 = sat(140 + 30, 13, 8);                 // C30 on B1C alone
    s.t1.push_back(c30);
    s.t1.push_back(sat(38 + 2, 8, 9));                 // R02 L1CA
    s.t1.push_back(sat(183, 6, 10));                   // J03 L1CA (QZSS)
    s.t1.push_back(sat(127, 24, 11));                  // S127 L1CA (SBAS)
    MeasT1 aux = sat(12, 4, 12);                       // G12: L1CA only on Aux1
    aux.t2 = {slave(0, 1)};
    s.t1.push_back(aux);
    MeasT1 wrong = sat(13, 17, 13);                    // a Galileo signal number on a GPS SVID
    s.t1.push_back(wrong);

    const Adapted a = adapt(s);
    ASSERT_TRUE(a.ok);
    EXPECT_EQ(a.info.status, SbfStatus::OK);
    ASSERT_EQ(a.ep.n, 5);
    EXPECT_EQ(a.info.n_gps, 2);
    EXPECT_EQ(a.info.n_gal, 1);
    EXPECT_EQ(a.info.n_bds, 2);
    EXPECT_EQ(a.info.geo, 2);
    EXPECT_NE(cell(a.ep, GNSS_SYS_GPS, 7), nullptr);
    EXPECT_NE(cell(a.ep, GNSS_SYS_GPS, 8), nullptr);
    EXPECT_NE(cell(a.ep, GNSS_SYS_GAL, 11), nullptr);
    EXPECT_NE(cell(a.ep, GNSS_SYS_BDS, 19), nullptr);
    EXPECT_NE(cell(a.ep, GNSS_SYS_BDS, 45), nullptr);
    EXPECT_EQ(cell(a.ep, GNSS_SYS_BDS, 30), nullptr);
    EXPECT_EQ(cell(a.ep, GNSS_SYS_GPS, 12), nullptr);
    EXPECT_EQ(cell(a.ep, GNSS_SYS_GPS, 13), nullptr);
    EXPECT_DOUBLE_EQ(a.ep.tow_s, 507606.0);

    // a Type2 cell is the slave's own code, Doppler and carrier
    const RawObs* g8c = cell(a.ep, GNSS_SYS_GPS, 8);
    const double pr1 = (22000000000LL + 8 * 1000) * 1e-3;
    EXPECT_DOUBLE_EQ(g8c->pr_m, pr1 + 1.234);
    const double d1 = (-15000000 + 8) * 1e-4;
    EXPECT_NEAR(g8c->rate_mps, -kLamL1 * (d1 * 1575.42 / 1176.45 - 0.0567), 1e-9);
    EXPECT_NEAR(g8c->phase_m, pr1 + 1.234 + 0.890 * kLamL1, 1e-9);
    EXPECT_EQ(g8c->lock_ms, 200000u);
    EXPECT_FLOAT_EQ(g8c->cn0_dbhz, 40.0f);
    const RawObs* c19c = cell(a.ep, GNSS_SYS_BDS, 19);
    const double d19 = (-15000000 + 159) * 1e-4;
    EXPECT_NEAR(c19c->rate_mps, -kLamB1I * (d19 * 1561.098 / 1268.52 - 0.0567), 1e-9);
}

TEST(MosaicRawEpoch, UnitsAndTheSignOfTheRate)
{
    MeasEpochSpec s;
    MeasT1 away = sat(5, 0, 1);
    away.codeMm(20123456789LL);
    away.doppler = -20000;                              // -2 Hz: the satellite is receding
    away.carrier(-123456);
    away.cn0 = 170;                                     // 52.5 dB-Hz
    away.lock = 4321;
    away.obs = 0x04;                                    // half-cycle
    MeasT1 toward = sat(70 + 3, 17, 2);
    toward.doppler = 31000000;                          // +3100 Hz: approaching
    MeasT1 b1i = sat(140 + 23, 28, 3);
    b1i.doppler = 10000;                                // +1 Hz
    s.t1 = {away, toward, b1i};
    const Adapted a = adapt(s);
    ASSERT_EQ(a.ep.n, 3);

    const RawObs& g = a.ep.obs[0];
    EXPECT_EQ(g.sys, GNSS_SYS_GPS);
    EXPECT_EQ(g.prn, 5);
    EXPECT_TRUE(g.has_pr);
    EXPECT_DOUBLE_EQ(g.pr_m, 20123456.789);
    EXPECT_TRUE(g.has_rate);
    EXPECT_GT(g.rate_mps, 0.0);                         // range growing
    EXPECT_DOUBLE_EQ(g.rate_mps, 2.0 * kLamL1);
    EXPECT_TRUE(g.has_phase);
    EXPECT_DOUBLE_EQ(g.phase_m, 20123456.789 - 123.456 * kLamL1);
    EXPECT_EQ(g.lock_ms, 254000u);                      // 4321 s, held to the Type2 clip
    EXPECT_TRUE(g.halfcyc);
    EXPECT_FLOAT_EQ(g.cn0_dbhz, 52.5f);
    EXPECT_LT(a.ep.obs[1].rate_mps, 0.0);               // range shrinking
    EXPECT_DOUBLE_EQ(a.ep.obs[1].rate_mps, -3100.0 * kLamL1);
    EXPECT_DOUBLE_EQ(a.ep.obs[2].rate_mps, -1.0 * kLamB1I);   // B1I's own wavelength
}

TEST(MosaicRawEpoch, DoNotUseFieldsLeaveTheirPartOut)
{
    MeasEpochSpec s;
    MeasT1 nocode = sat(1, 0, 1);
    nocode.code_msb = 0; nocode.code_lsb = 0;
    MeasT1 nodop = sat(2, 0, 2);
    nodop.doppler = -2147483647 - 1;
    MeasT1 nocar = sat(3, 0, 3);
    nocar.car_msb = -128; nocar.car_lsb = 0;
    MeasT1 nolock = sat(4, 0, 4);
    nolock.lock = 65535;
    MeasT1 nocn0 = sat(5, 0, 5);
    nocn0.cn0 = 255;
    MeasT1 slavednu = sat(6, 4, 6);
    MeasT2 l1 = slave(0);
    l1.lock = 255;
    slavednu.t2 = {l1};
    s.t1 = {nocode, nodop, nocar, nolock, nocn0, slavednu};
    const Adapted a = adapt(s);
    ASSERT_EQ(a.ep.n, 6);
    const RawObs* o = cell(a.ep, GNSS_SYS_GPS, 1);
    EXPECT_FALSE(o->has_pr);
    EXPECT_TRUE(o->has_rate);
    EXPECT_FALSE(o->has_phase);                          // the phase is a range from the code
    o = cell(a.ep, GNSS_SYS_GPS, 2);
    EXPECT_TRUE(o->has_pr);
    EXPECT_FALSE(o->has_rate);
    EXPECT_TRUE(o->has_phase);
    o = cell(a.ep, GNSS_SYS_GPS, 3);
    EXPECT_FALSE(o->has_phase);
    EXPECT_EQ(o->phase_m, 0.0);
    o = cell(a.ep, GNSS_SYS_GPS, 4);
    EXPECT_FALSE(o->has_phase);
    EXPECT_EQ(o->lock_ms, 0u);
    o = cell(a.ep, GNSS_SYS_GPS, 5);
    EXPECT_FALSE(o->has_pr);
    EXPECT_FALSE(o->has_rate);
    EXPECT_FALSE(o->has_phase);
    EXPECT_EQ(o->cn0_dbhz, 0.0f);
    o = cell(a.ep, GNSS_SYS_GPS, 6);
    EXPECT_FALSE(o->has_phase);
    EXPECT_EQ(o->lock_ms, 0u);
    EXPECT_TRUE(o->has_pr);
}

TEST(MosaicRawEpoch, OneCellPerSatellite)
{
    MeasEpochSpec s;
    MeasT1 first = sat(9, 0, 1);
    first.code_msb = 0; first.code_lsb = 0;            // no code on the first channel
    MeasT1 second = sat(9, 0, 2);
    MeasT1 third = sat(9, 0, 3);
    third.codeMm(1);
    s.t1 = {first, second, third};
    const Adapted a = adapt(s);
    ASSERT_EQ(a.ep.n, 1);
    EXPECT_EQ(a.info.dupes, 2);
    EXPECT_EQ(a.info.n_gps, 1);
    EXPECT_TRUE(a.ep.obs[0].has_pr);
    EXPECT_DOUBLE_EQ(a.ep.obs[0].pr_m, (22000000000LL + 9 * 1000) * 1e-3);   // the second, kept over the third
}

TEST(MosaicRawEpoch, RefusedAndEmptyEpochsGiveNothing)
{
    MeasEpochSpec s;
    s.t1 = {sat(1, 0, 1)};
    s.flags = 0xA5;                                     // scrambled
    Adapted a = adapt(s);
    EXPECT_FALSE(a.ok);
    EXPECT_EQ(a.ep.n, 0);
    EXPECT_EQ(a.info.status, SbfStatus::SCRAMBLED);

    MeasEpochSpec e;
    e.sb1 = e.sb2 = 0;                                  // N1 = 0
    a = adapt(e);
    EXPECT_FALSE(a.ok);
    EXPECT_EQ(a.info.status, SbfStatus::OK);

    MeasEpochSpec g;
    g.t1 = {sat(38 + 1, 8, 1), sat(140 + 1, 28, 2)};     // GLONASS and a GEO only
    a = adapt(g);
    EXPECT_FALSE(a.ok);
    EXPECT_EQ(a.ep.n, 0);
    EXPECT_EQ(a.info.geo, 1);

    MeasEpochSpec t;
    t.t1 = {sat(1, 0, 1)};
    t.tow = kTowDnu;
    a = adapt(t);
    EXPECT_FALSE(a.ok);
    EXPECT_EQ(a.info.status, SbfStatus::NO_TIME);
}

TEST(MosaicRawEpoch, ClockJumpsAreLeftInWithTheLockReset)
{
    // A -1 ms jump (252 -> 251), as on the 10/16 cone-off capture at TOW
    // 498404600: every code moves -c * 1 ms with the receiver's clock; the
    // fields stay as given, so tow - pr/c is still the transmit time
    MosaicJumpState js;
    const double rho0 = 21500000.0, rdot = 300.0;      // range and its rate, m and m/s
    const double bias0_ms = 0.4998;                     // nearly 0.5 ms ahead: the receiver is about to step
    std::vector<Adapted> eps;
    const uint8_t cums[4] = {252, 252, 251, 251};
    double t_sys[4];
    for (int k = 0; k < 4; ++k)
    {
        const uint32_t tow = 498404500 + 50 * k;
        const double bias_ms = bias0_ms + 0.0001 * k + (k >= 2 ? -1.0 : 0.0);   // 2 ppm fast, then -1 ms
        t_sys[k] = tow * 1e-3 - bias_ms * 1e-3;
        const double rho = rho0 + rdot * (t_sys[k] - t_sys[0]);
        MeasEpochSpec s;
        s.tow = tow;
        s.cum = cums[k];
        MeasT1 g = sat(17, 0, 1);
        g.prM(rho + kC * bias_ms * 1e-3);
        s.t1 = {g};
        eps.push_back(adapt(sbftest::measEpoch(s), js));
        ASSERT_TRUE(eps.back().ok);
        // the transmit time from the uncorrected fields, against the truth
        EXPECT_NEAR(eps.back().ep.tow_s - eps.back().ep.obs[0].pr_m / kC, t_sys[k] - rho / kC, 1e-9) << k;
    }
    EXPECT_TRUE(eps[0].info.rebased);
    EXPECT_FALSE(eps[0].info.jumped);
    EXPECT_FALSE(eps[1].info.jumped);
    EXPECT_TRUE(eps[2].info.jumped);
    EXPECT_EQ(eps[1].info.lock_cap_ms, 254000u);
    EXPECT_EQ(eps[2].info.lock_cap_ms, 0u);
    EXPECT_EQ(eps[3].info.lock_cap_ms, 50u);
    EXPECT_FALSE(eps[2].info.rebased);
    EXPECT_EQ(eps[2].info.jump_ms, -1);
    EXPECT_EQ(eps[2].info.j_ms, -1);
    EXPECT_EQ(eps[3].info.j_ms, -1);
    EXPECT_FALSE(eps[3].info.jumped);
    EXPECT_EQ(js.jumps, 1u);
    EXPECT_EQ(eps[1].ep.obs[0].lock_ms, 254000u);
    EXPECT_EQ(eps[2].ep.obs[0].lock_ms, 0u);            // no carrier interval spans the step:
    EXPECT_EQ(eps[3].ep.obs[0].lock_ms, 50u);           // the lock counts from it
    // the code steps by c * 1 ms, uncorrected; the epochs are 51 ms apart in GPS time
    const double step = eps[2].ep.obs[0].pr_m - eps[1].ep.obs[0].pr_m - rdot * (t_sys[2] - t_sys[1]);
    EXPECT_NEAR(step, -kC * 1e-3 + kC * 1e-7, 1e-3);
    EXPECT_NEAR(t_sys[2] - t_sys[1], 0.051 - 1e-7, 1e-9);
}

TEST(MosaicRawEpoch, AJumpOffTheFuseGridResetsTheNextCellsLock)
{
    // 20 Hz from the receiver, 10 Hz adapted: the epochs between are only
    // noted, and the clock steps at one of them
    MosaicJumpState js;
    std::vector<MosaicEpochInfo> grid;
    std::vector<uint32_t> locks;
    for (uint32_t tow = 1000; tow <= 1400; tow += 50)
    {
        MeasEpochSpec s;
        s.tow = tow;
        s.cum = tow >= 1150 ? 251 : 252;   // the step is at 1150, off the 100 ms grid
        s.t1 = {sat(2, 0, 1)};
        const auto blk = sbftest::measEpoch(s);
        MosaicEpochInfo noted;
        ASSERT_TRUE(noteMeasEpochClock(blk.data(), blk.size(), js, noted));   // every epoch, as the driver's poll sees it
        EXPECT_EQ(noted.jumped, tow == 1150) << tow;
        if (!onFuseGrid(tow, 100)) continue;
        RawEpoch ep;
        MosaicEpochInfo info;
        ASSERT_TRUE(rawEpochFromMeasEpoch(blk.data(), blk.size(), ep, js, info));   // seen twice: counted once
        grid.push_back(info);
        locks.push_back(ep.obs[0].lock_ms);
    }
    ASSERT_EQ(grid.size(), 5u);   // 1000, 1100, 1200, 1300, 1400
    EXPECT_EQ(js.jumps, 1u);
    EXPECT_EQ(locks[1], 254000u);
    EXPECT_FALSE(grid[2].jumped);                 // its own header did not step ...
    EXPECT_EQ(locks[2], 50u);                     // ... but its lock counts from 1150
    EXPECT_EQ(grid[2].j_ms, -1);
    EXPECT_EQ(locks[3], 150u);
    EXPECT_EQ(grid[4].j_ms, -1);
    // shorter than the interval from 1100, the last grid epoch before the
    // step, at every grid epoch after it (as if the ones between were lost)
    for (size_t k = 2; k < grid.size(); ++k)
        EXPECT_LT(locks[k], grid[k].tow_ms - 1100u) << grid[k].tow_ms;
}

TEST(MosaicRawEpoch, JumpCountWrapsAndStartsOver)
{
    MosaicJumpState js;
    uint32_t lock_ms = 0;
    auto at = [&](uint32_t tow, uint8_t cum, uint16_t wnc = 2440) {
        MeasEpochSpec s;
        s.tow = tow;
        s.wnc = wnc;
        s.cum = cum;
        s.t1 = {sat(1, 0, 1)};
        const Adapted a = adapt(sbftest::measEpoch(s), js);
        lock_ms = a.ep.obs[0].lock_ms;
        return a.info;
    };
    EXPECT_TRUE(at(1000, 255).rebased);
    EXPECT_EQ(lock_ms, 254000u);                        // the first epoch: no jump to count from
    MosaicEpochInfo i = at(1050, 0);                    // 255 -> 0: +1 ms
    EXPECT_TRUE(i.jumped);
    EXPECT_EQ(i.jump_ms, 1);
    EXPECT_EQ(i.j_ms, 1);
    EXPECT_EQ(lock_ms, 0u);
    i = at(1100, 254);                                  // -2 ms: still a jump
    EXPECT_EQ(i.j_ms, -1);
    EXPECT_FALSE(i.rebased);
    i = at(1150, 1);                                    // +3: a receiver reset, start over
    EXPECT_TRUE(i.rebased);
    EXPECT_TRUE(i.jumped);
    EXPECT_EQ(i.j_ms, 0);
    EXPECT_EQ(lock_ms, 0u);
    i = at(1100, 1);                                    // time went back: start over
    EXPECT_TRUE(i.rebased);
    EXPECT_FALSE(i.jumped);
    EXPECT_EQ(lock_ms, 0u);                             // a start-over cuts the lock too
    EXPECT_EQ(js.rebases, 2u);
    i = at(604799950, 1);
    i = at(0, 0, 2441);                                 // across the week's end: a jump, not a start-over
    EXPECT_FALSE(i.rebased);
    EXPECT_EQ(i.j_ms, -1);

    // revision 0 says nothing about jumps: the count holds, and the lock
    // still counts from the last jump seen (at 0 of week 2441)
    MeasEpochSpec s;
    s.rev = 0;
    s.tow = 50;
    s.wnc = 2441;
    s.cum = 200;
    s.t1 = {sat(1, 0, 1)};
    const Adapted a = adapt(sbftest::measEpoch(s), js);
    EXPECT_FALSE(a.info.jumped);
    EXPECT_FALSE(a.info.rebased);
    EXPECT_EQ(a.info.j_ms, -1);
    EXPECT_EQ(a.ep.obs[0].lock_ms, 50u);
}

TEST(MosaicRawEpoch, LockTimeHeldToTheType2Clip)
{
    // G05's L1 C/A as the master at 600 s, then as a Type2 (clipped at 254)
    // under its L5, then the master again, then a relock as a Type2
    MosaicJumpState js;
    std::vector<uint32_t> locks;
    for (int k = 0; k < 5; ++k)
    {
        MeasEpochSpec s;
        s.tow = 1000 + 100 * k;
        MeasT1 g = sat(5, 0, 1);
        g.lock = static_cast<uint16_t>(600 + k);
        if (k == 1 || k == 3)
        {
            MeasT2 l1 = slave(0);
            l1.lock = k == 1 ? 254 : 0;
            g.sig(4);
            g.t2 = {l1};
        }
        MeasT1 young = sat(6, 0, 2);
        young.lock = static_cast<uint16_t>(100 + k);
        s.t1 = {g, young};
        const Adapted a = adapt(sbftest::measEpoch(s), js);
        ASSERT_EQ(a.ep.n, 2);
        locks.push_back(cell(a.ep, GNSS_SYS_GPS, 5)->lock_ms);
        EXPECT_EQ(cell(a.ep, GNSS_SYS_GPS, 6)->lock_ms, (100u + k) * 1000u);   // under the clip: as given
        EXPECT_TRUE(cell(a.ep, GNSS_SYS_GPS, 5)->has_phase) << k;
    }
    EXPECT_EQ(locks[0], 254000u);
    EXPECT_EQ(locks[1], 254000u);                       // the switch to Type2 does not go back
    EXPECT_EQ(locks[2], 254000u);
    EXPECT_EQ(locks[3], 0u);                            // a relock still does
    EXPECT_EQ(locks[4], 254000u);
}

// Carrier range rates through the builder (its carrier on, 10 Hz), with the
// LC86G's golden ephemerides: G-first's L1 C/A moving between Type1 and Type2
// every epoch, 600 s locked, keeps its carrier; a -1 ms clock jump whose epoch
// and the one after it never reach the builder (adapted, then lost on the
// queue or dropped as stale) leaves no interval across it; and the carrier
// is back cr_min_lock_ms after the jump.
TEST(MosaicRawEpoch, CarrierIntervalsThroughTheBuilder)
{
    gnss_raw::EphemerisStore store;
    for (int i = 0; i < kGoldenRmEphN; ++i)
    {
        gps_eph_t e;
        ASSERT_TRUE(gnss_raw::decodeEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len), 2440, e));
        store.add(e);
    }
    const uint32_t tow0 = static_cast<uint32_t>(std::llround(kGoldenRmTowB * 10.0)) * 100U;   // on the 100 ms grid
    std::vector<uint8_t> prns;
    for (uint8_t prn = 1; prn <= 32; ++prn)
        if (store.select(GNSS_SYS_GPS, prn, tow0 * 1e-3) != nullptr) prns.push_back(prn);
    ASSERT_GE(prns.size(), 4u);

    gnss_raw::RawMeasBuilder::Config cfg;
    cfg.epoch_period_s = 0.1;
    gnss_raw::RawMeasBuilder b(cfg);
    MosaicJumpState js;
    constexpr int kJump = 15, kEpochs = 30;
    int cr_before = 0, cr_across = 0, cr_after = 0;
    for (int k = 0; k < kEpochs; ++k)
    {
        MeasEpochSpec s;
        s.wnc = 2433;
        s.tow = tow0 + 100U * static_cast<uint32_t>(k);
        s.cum = k >= kJump ? 251 : 252;
        for (size_t i = 0; i < prns.size(); ++i)
        {
            MeasT1 t = sat(prns[i], 0, static_cast<uint8_t>(i + 1));
            t.prM(2.2e7 + 1.0e5 * static_cast<double>(i) + 30.0 * k);
            t.cn0 = 140;                                // 45 dB-Hz
            if (i == 0 && k % 2 == 1)
            {
                MeasT2 l1 = slave(0);
                l1.lock = 254;
                l1.cn0 = 140;
                t.sig(4);
                t.t2 = {l1};
            }
            s.t1.push_back(t);
        }
        const Adapted a = adapt(sbftest::measEpoch(s), js);
        ASSERT_EQ(a.ep.n, static_cast<int>(prns.size()));
        if (k == kJump || k == kJump + 1)
            continue;                                   // adapted, never built
        gnss_raw::RawMeas m[RawEpoch::kMaxObs];
        const int n = b.build(a.ep, store, nullptr, 0.0, m, RawEpoch::kMaxObs);
        ASSERT_EQ(n, a.ep.n);
        int cr = 0;
        for (int i = 0; i < n; ++i)
            if (m[i].has_cr) cr++;
        if (k > 0 && k < kJump)
        {
            EXPECT_EQ(cr, n) << k;                      // the level switches included
            cr_before += cr;
        }
        else if (k == kJump + 2)
            cr_across = cr;                             // from kJump - 1: across the jump
        else if (k >= kJump + 11)
        {
            EXPECT_EQ(cr, n) << k;                      // a lock of 1 s at the earlier end
            cr_after += cr;
        }
    }
    EXPECT_GT(cr_before, 0);
    EXPECT_EQ(cr_across, 0);
    EXPECT_GT(cr_after, 0);
}

TEST(MosaicRawEpoch, FuseGridRunsAcrossTheWeek)
{
    EXPECT_TRUE(onFuseGrid(604799900, 100));
    EXPECT_FALSE(onFuseGrid(604799950, 100));
    EXPECT_TRUE(onFuseGrid(0, 100));                    // the next week's first epoch
    EXPECT_FALSE(onFuseGrid(50, 100));
    EXPECT_TRUE(onFuseGrid(100, 100));
    EXPECT_TRUE(onFuseGrid(604799000, 1000));
    EXPECT_FALSE(onFuseGrid(604799800, 1000));
    EXPECT_TRUE(onFuseGrid(50, 50));
    EXPECT_TRUE(onFuseGrid(1234, 0));                   // no grid: every epoch
    EXPECT_FALSE(onFuseGrid(kTowDnu, 100));
    EXPECT_EQ(604800000u % 50, 0u);
    EXPECT_EQ(604800000u % 200, 0u);
}

TEST(MosaicRawEpoch, EpochTimeFromTheReceiverClockBias)
{
    // t_sys = t_rx - bias: a receiver 0.4 ms ahead tags GPS 0.4 ms earlier
    EXPECT_NEAR(epochGpsS(2440, 1000, 0.4) - 2440 * 604800.0, 0.9996, 1e-6);
    // the bias carried from the PVT by its drift and the jumps since it
    EXPECT_DOUBLE_EQ(dtrMs(0.4, 0.27, 0.05, 0), 0.4 + 0.27e-3 * 0.05);
    EXPECT_DOUBLE_EQ(dtrMs(0.4, 0.0, 0.0, -1), -0.6);   // a -1 ms jump: the new bias is the old less 1 ms
    // across the week's end the seconds run on
    EXPECT_NEAR(epochGpsS(2441, 0, 0.3) - epochGpsS(2440, 604799950, 0.3), 0.050, 1e-6);
    // across a -1 ms jump, 50 ms of receiver time are 51 ms of GPS time
    EXPECT_NEAR(epochGpsS(2440, 498404600, dtrMs(-0.4998, 0.0, 0.05, -1)) -
                    epochGpsS(2440, 498404550, -0.4998), 0.051, 1e-6);
}

TEST(MosaicRawEpoch, PpsResidualSaturates)
{
    EXPECT_EQ(ppsResidualUs(1000500, 1000000, -0.5), 0);
    EXPECT_EQ(ppsResidualUs(1000000, 1000000, 0.25), 250);
    EXPECT_EQ(ppsResidualUs(int64_t(604800) * 1000000 + 5, 0, 0.0), 2147483647);
    EXPECT_EQ(ppsResidualUs(0, int64_t(604800) * 1000000, 0.0), -2147483647 - 1);
}

// The whole tag on a simulated bench: a mosaic whose clock runs 0.27 ppm
// fast from +0.4996 ms and steps -1 ms at a whole second, its MeasEpoch at
// 20 Hz from 10 s before the end of GPS week 2440 to 10 s into 2441, a PVT
// at every epoch, and the 1PPS edges of every GPS second on an MCU clock
// 25 ppm fast fed to the LC86G driver's GnssTimeSync. The arithmetic alone
// (each epoch's GPS instant through the exact MCU clock) must be within
// 0.5 us of the truth; through the PPS line, every tag within 10 us and every
// whole-second epoch's PPS residual too (the line's own fit and the edges'
// whole microseconds); a tag with a whole-second, week, bias-sign or
// missed-jump slip must fail the 200 us check.
TEST(MosaicRawEpoch, TimeTagOnThePpsLineAcrossAJumpAndAWeek)
{
    const double t_ref = 2440 * 604800.0 + 604700.0;   // GPS s
    auto mcuTrue = [&](double gps) { return 4.0e9 + (gps - t_ref) * 1.0e6 * (1.0 + 25e-6); };
    lc86::GnssTimeSync sync;
    int64_t next_pps = static_cast<int64_t>(t_ref);
    auto ppsUpTo = [&](double gps) {
        for (; next_pps <= static_cast<int64_t>(std::floor(gps)); ++next_pps)
            sync.addSample(static_cast<double>(next_pps), std::llround(mcuTrue(static_cast<double>(next_pps))));
    };

    const double drift_ppm = 0.27;
    const int64_t start_ms = int64_t(2440) * 604800000 + 604790000;   // receiver time of the first epoch
    const int64_t jump_ms = int64_t(2440) * 604800000 + 604795000;    // the receiver steps here (a whole second)
    auto biasMs = [&](int64_t rx_ms) {
        return 0.4996 + drift_ppm * 1e-3 * (rx_ms - start_ms) * 1e-3 + (rx_ms >= jump_ms ? -1.0 : 0.0);
    };

    MosaicJumpState js;
    double pvt_bias = 0.0;
    int64_t pvt_rx_ms = 0;
    int32_t pvt_j = 0;
    bool have_pvt = false;
    double worst_math_us = 0.0, worst_tag_us = 0.0, worst_pps_us = 0.0;
    int whole = 0, jumps_seen = 0, planted = 0, caught = 0;
    for (int64_t rx_ms = start_ms; rx_ms <= start_ms + 20000; rx_ms += 50)
    {
        const uint16_t wnc = static_cast<uint16_t>(rx_ms / 604800000);
        const uint32_t tow = static_cast<uint32_t>(rx_ms % 604800000);
        const double bias = biasMs(rx_ms);
        const double gps = rx_ms * 1e-3 - bias * 1e-3;   // the epoch's true GPS instant
        ppsUpTo(gps);

        MeasEpochSpec s;
        s.wnc = wnc;
        s.tow = tow;
        s.cum = static_cast<uint8_t>(rx_ms >= jump_ms ? 251 : 252);
        s.t1 = {sat(3, 0, 1)};
        const auto blk = sbftest::measEpoch(s);
        RawEpoch ep;
        MosaicEpochInfo info;
        ASSERT_TRUE(rawEpochFromMeasEpoch(blk.data(), blk.size(), ep, js, info));
        if (info.jumped) jumps_seen++;

        if (have_pvt && sync.valid())
        {
            const double dt_s = (rx_ms - pvt_rx_ms) * 1e-3;
            const double dtr = dtrMs(pvt_bias, drift_ppm, dt_s, info.j_ms - pvt_j);
            const double gps_tag = epochGpsS(info.wnc, info.tow_ms, dtr);
            worst_math_us = std::max(worst_math_us, std::fabs(mcuTrue(gps_tag) - mcuTrue(gps)));
            int64_t tag = 0;
            ASSERT_TRUE(sync.mcuAt(gps_tag, tag));
            worst_tag_us = std::max(worst_tag_us, std::fabs(tag - mcuTrue(gps)));
            if (tow % 1000 == 0)
            {
                whole++;
                // the edge of this GPS second, and the receiver's own bias for the epoch
                const int64_t edge = std::llround(mcuTrue(std::round(rx_ms * 1e-3)));
                const int32_t r = ppsResidualUs(tag, edge, bias);
                worst_pps_us = std::max(worst_pps_us, std::fabs(static_cast<double>(r)));

                // planted slips: each must fail the check
                int64_t bad = 0;
                auto fails = [&](double gps_s) {
                    planted++;
                    if (sync.mcuAt(gps_s, bad) && std::abs(ppsResidualUs(bad, edge, bias)) > 200) caught++;
                };
                fails(epochGpsS(info.wnc, info.tow_ms + 1000, dtr));                    // a whole second
                fails(epochGpsS(static_cast<uint16_t>(info.wnc + 1), info.tow_ms, dtr)); // a week
                fails(epochGpsS(static_cast<uint16_t>(info.wnc - 1), info.tow_ms, dtr));
                fails(epochGpsS(info.wnc, info.tow_ms, -dtr));                          // the bias's sign
                if (info.jumped)
                    fails(epochGpsS(info.wnc, info.tow_ms, dtrMs(pvt_bias, drift_ppm, dt_s, 0)));   // the jump missed
            }
        }
        // this epoch's PVTGeodetic: its bias, used by the next epoch's tag
        pvt_bias = bias;
        pvt_rx_ms = rx_ms;
        pvt_j = info.j_ms;
        have_pvt = true;
    }
    std::printf("[tag] worst arithmetic error %.3f us, tag error %.3f us, PPS residual %.3f us over %d whole "
                "seconds; %d of %d planted slips caught\n", worst_math_us, worst_tag_us, worst_pps_us, whole, caught,
                planted);
    EXPECT_EQ(jumps_seen, 1);
    EXPECT_GE(whole, 19);
    EXPECT_LT(worst_math_us, 0.5);
    EXPECT_LT(worst_tag_us, 10.0);
    EXPECT_LT(worst_pps_us, 10.0);
    EXPECT_EQ(caught, planted);
    EXPECT_EQ(planted, 4 * whole + 1);
}

// A PVT whose own MeasEpoch was lost exactly at a clock jump: its RxClkBias
// already holds the jump, the jump count takes it only at the next
// MeasEpoch, and a tag carried from it would hold the jump twice (1 ms).
// pvtAnchorsJumps passes it over, and the PVT before it carries the tag.
TEST(MosaicRawEpoch, APvtWhoseMeasEpochWasLostDoesNotAnchorTheTag)
{
    const double drift_ppm = 0.27;
    const uint32_t tow0 = 498404000, jump_tow = 498404600;
    auto biasMs = [&](uint32_t tow) {
        return 0.4996 + drift_ppm * 1e-3 * (tow - tow0) * 1e-3 + (tow >= jump_tow ? -1.0 : 0.0);
    };
    MosaicJumpState js;
    struct Pvt { double bias_ms; uint32_t tow; int32_t j; };
    Pvt kept{0.0, 0, 0}, naive{0.0, 0, 0};
    bool have_kept = false;
    double worst_kept_ms = 0.0, worst_naive_ms = 0.0;
    int passed_over = 0;
    for (uint32_t tow = tow0; tow <= tow0 + 1500; tow += 50)
    {
        if (tow != jump_tow)                            // this MeasEpoch is lost
        {
            MeasEpochSpec s;
            s.tow = tow;
            s.cum = tow >= jump_tow ? 251 : 252;
            s.t1 = {sat(3, 0, 1)};
            const auto blk = sbftest::measEpoch(s);
            MosaicEpochInfo info;
            ASSERT_TRUE(noteMeasEpochClock(blk.data(), blk.size(), js, info));
            if (have_kept)
            {
                const double dtr = dtrMs(kept.bias_ms, drift_ppm, (tow - kept.tow) * 1e-3, info.j_ms - kept.j);
                const double dtr_naive = dtrMs(naive.bias_ms, drift_ppm, (tow - naive.tow) * 1e-3, info.j_ms - naive.j);
                worst_kept_ms = std::max(worst_kept_ms, std::fabs(dtr - biasMs(tow)));
                worst_naive_ms = std::max(worst_naive_ms, std::fabs(dtr_naive - biasMs(tow)));
            }
        }
        // the PVTGeodetic of this epoch, after its MeasEpoch (if any)
        naive = {biasMs(tow), tow, js.j_ms};
        if (pvtAnchorsJumps(js, 2440, tow))
        {
            kept = {biasMs(tow), tow, js.j_ms};
            have_kept = true;
        }
        else
            passed_over++;
    }
    EXPECT_EQ(passed_over, 1);
    EXPECT_LT(worst_kept_ms, 1e-9);
    EXPECT_NEAR(worst_naive_ms, 1.0, 1e-9);             // what the check keeps out
    EXPECT_EQ(js.jumps, 1u);
}

// The range rate's sign through the builder: with the LC86G's golden
// ephemerides (fixtures/raw_meas_golden.h, the COCOM rig), each satellite's
// SBF Doppler is made from its geometry as the mosaic measures it -- positive
// approaching, with the receiver's clock drift and the satellite's -- and
// the builder must give the range rate the filter predicts: the satellite's
// velocity along the line of sight plus the receiver's clock drift.
TEST(MosaicRawEpoch, RangeRateSignThroughTheBuilder)
{
    gnss_raw::EphemerisStore store;
    for (int i = 0; i < kGoldenRmEphN; ++i)
    {
        gps_eph_t e;
        ASSERT_TRUE(gnss_raw::decodeEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len), 2440, e));
        store.add(e);
    }
    const double t0 = kGoldenRmTowB;                     // GPS s of week
    const double* r = kGoldenRmRx;
    const double dtr_dot = 2.7e-7;                       // the receiver's clock drift: 81 m/s
    const double dtr = 0.3e-3;

    MeasEpochSpec s;
    s.tow = static_cast<uint32_t>(std::llround(t0 * 1000.0));
    s.wnc = 2433;
    struct Want { int sys, prn; };
    std::vector<Want> sats;
    uint8_t ch = 1;
    for (int sys : {GNSS_SYS_GPS, GNSS_SYS_GAL, GNSS_SYS_BDS})
        for (int prn = 1; prn < 64; ++prn)
        {
            const double to_sys = sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0;
            const gps_eph_t* e = store.select(sys, prn, t0 + to_sys);
            if (e == nullptr) continue;
            // the satellite where the builder will place it: at the
            // receiver's tow less the travel time and the receiver's bias
            double p[3], v[3], clk, pa[3], pb[3], ca, cb, d[3], rho = 0.075 * kC;
            for (int it = 0; it < 3; ++it)
            {
                gps_sat_pos(e, t0 - dtr - rho / kC + to_sys, p, v, &clk);
                for (int k = 0; k < 3; ++k) d[k] = p[k] - r[k];
                rho = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            }
            const double t_tx = t0 - dtr - rho / kC + to_sys;
            gps_sat_pos(e, t_tx, p, v, &clk);
            gps_sat_pos(e, t_tx - 0.5, pa, nullptr, &ca);
            gps_sat_pos(e, t_tx + 0.5, pb, nullptr, &cb);
            const double rate_geo = (d[0] * v[0] + d[1] * v[1] + d[2] * v[2]) / rho;
            const double lam = sys == GNSS_SYS_BDS ? kLamB1I : kLamL1;
            // SBF's Doppler: positive approaching, so -(range rate)/lambda
            const double doppler = -(rate_geo + kC * dtr_dot - kC * (cb - ca)) / lam;
            const uint8_t svid = static_cast<uint8_t>(sys == GNSS_SYS_GPS   ? prn
                                                      : sys == GNSS_SYS_GAL ? prn + 70
                                                      : prn <= 40           ? prn + 140
                                                                            : prn + 182);
            MeasT1 t;
            t.rx_channel = ch++;
            t.svid = svid;
            t.sig(sys == GNSS_SYS_GPS ? 0 : sys == GNSS_SYS_GAL ? 17 : 28);
            t.prM(rho + kC * dtr - kC * clk);
            t.dopplerHz(doppler);
            t.cn0 = 140;
            s.t1.push_back(t);
            sats.push_back({sys, prn});
        }
    ASSERT_GE(sats.size(), 8u);

    MosaicJumpState js;
    const Adapted a = adapt(sbftest::measEpoch(s), js);
    ASSERT_TRUE(a.ok);
    ASSERT_EQ(a.ep.n, static_cast<int>(sats.size()));
    gnss_raw::RawMeasBuilder b;
    gnss_raw::RawMeas m[RawEpoch::kMaxObs];
    const int n = b.build(a.ep, store, nullptr, 0.0, m, RawEpoch::kMaxObs);
    ASSERT_EQ(n, a.ep.n);
    int receding = 0, approaching = 0;
    for (int i = 0; i < n; ++i)
    {
        const double d[3] = {m[i].sat_pos[0] - r[0], m[i].sat_pos[1] - r[1], m[i].sat_pos[2] - r[2]};
        const double rho = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        const double along = (d[0] * m[i].sat_vel[0] + d[1] * m[i].sat_vel[1] + d[2] * m[i].sat_vel[2]) / rho;
        ASSERT_TRUE(m[i].has_rr);
        EXPECT_NEAR(m[i].rr, along + kC * dtr_dot, 0.01) << m[i].sys << "/" << int(m[i].prn);
        (along > 0.0 ? receding : approaching)++;
    }
    // both signs in the sky, so a flipped convention could not pass
    EXPECT_GT(receding, 0);
    EXPECT_GT(approaching, 0);
}

// ── The 10/16 captures (private; TR_SBF_PRIVATE_CAPTURE) ─────────────────

TEST(MosaicRawEpochPrivateCapture, L1PicksPerEpoch)
{
    const auto paths = sbftest::privateCapturePaths();
    if (paths.empty()) GTEST_SKIP() << "TR_SBF_PRIVATE_CAPTURE not set";
    for (const std::string& path : paths)
    {
        MosaicJumpState js;
        std::vector<int> picks;
        uint32_t empty = 0, refused = 0, geo = 0, dupes = 0, over = 0, jumps = 0;
        uint32_t no_pr = 0, no_rate = 0;
        // lock_ms going back to a nonzero value from one epoch to the next (a
        // false slip to the carrier), and locks that would let a carrier
        // interval from before a clock jump through, from any epoch before it
        // (as if the ones between were lost) up to 1.5 s on
        uint32_t lock_backs = 0, span_locks = 0, level_switches = 0;
        int64_t pre_jump_ms = -1, last_ms = -1;
        std::map<std::pair<int, int>, bool> was_master;
        double jump_interval_ms = 0.0;
        // when the log's epochs under 18 picks (or empty) were, s from its first epoch
        int64_t first_ms = -1;
        double low_from_s = -1.0, low_to_s = -1.0;
        auto noteLow = [&](const MosaicEpochInfo& info) {
            const double t = (int64_t(info.wnc) * 604800000 + info.tow_ms - first_ms) * 1e-3;
            if (low_from_s < 0.0) low_from_s = t;
            low_to_s = t;
        };
        RawEpoch prev;
        bool have_prev = false;
        uint32_t prev_tow = 0;
        sbftest::CaptureScan scan;
        const bool read = sbftest::scanCapture(path, [&](const uint8_t* b, size_t len) {
            SbfHeader h;
            if (!sbfHeader(b, len, h) || h.number != SBF_MEAS_EPOCH) return;
            RawEpoch ep;
            MosaicEpochInfo info;
            const bool ok = rawEpochFromMeasEpoch(b, len, ep, js, info);
            if (info.status != SbfStatus::OK) { refused++; return; }
            const int64_t t_ms = int64_t(info.wnc) * 604800000 + info.tow_ms;
            if (first_ms < 0) first_ms = t_ms;
            if (info.jumped) pre_jump_ms = last_ms;   // the last epoch on the old clock
            last_ms = t_ms;
            if (!ok) { empty++; noteLow(info); return; }
            picks.push_back(ep.n);
            if (ep.n < 18) noteLow(info);
            geo += info.geo;
            dupes += info.dupes;
            over += info.over;
            for (int i = 0; i < ep.n; ++i)
            {
                if (!ep.obs[i].has_pr) no_pr++;
                if (!ep.obs[i].has_rate) no_rate++;
            }
            for (int i = 0; i < ep.n; ++i)
            {
                const RawObs& o = ep.obs[i];
                const RawObs* p = have_prev ? cell(prev, o.sys, o.prn) : nullptr;
                if (p != nullptr && o.lock_ms != 0 && o.lock_ms < p->lock_ms) lock_backs++;
                if (pre_jump_ms >= 0 && t_ms - pre_jump_ms < 1500 && int64_t(o.lock_ms) >= t_ms - pre_jump_ms)
                    span_locks++;
            }
            // which level each L1 pick sat at (a second pass over the block)
            {
                SbfMeasReader rd;
                SbfMeasSignal s;
                if (rd.begin(b, len) == SbfStatus::OK)
                    while (rd.next(s))
                    {
                        uint8_t prn;
                        const SbfSys sys = sbfSvid(s.svid, prn);
                        if (s.antenna != 0 || !((sys == SbfSys::GPS && s.sig == SBF_SIG_GPS_L1CA) ||
                                                (sys == SbfSys::GAL && s.sig == SBF_SIG_GAL_E1) ||
                                                (sys == SbfSys::BDS && s.sig == SBF_SIG_BDS_B1I)))
                            continue;
                        const auto key = std::make_pair(static_cast<int>(sys), static_cast<int>(prn));
                        const auto at = was_master.find(key);
                        if (at != was_master.end() && at->second != s.master) level_switches++;
                        was_master[key] = s.master;
                    }
            }
            if (info.jumped)
            {
                jumps++;
                // the GPS interval the jump makes of 50 ms of receiver time,
                // from the uncorrected fields: the transmit time's step plus
                // the range's change over it (the mean rate, drift included)
                if (have_prev && info.tow_ms - prev_tow == 50)
                {
                    std::vector<double> iv;
                    for (int i = 0; i < ep.n; ++i)
                    {
                        const RawObs* p = cell(prev, ep.obs[i].sys, ep.obs[i].prn);
                        if (p == nullptr || !p->has_pr || !ep.obs[i].has_pr || !p->has_rate || !ep.obs[i].has_rate) continue;
                        const double tx_step = (ep.tow_s - ep.obs[i].pr_m / kC) - (prev.tow_s - p->pr_m / kC);
                        iv.push_back(tx_step + 0.5 * (p->rate_mps + ep.obs[i].rate_mps) * 0.05 / kC);
                    }
                    if (!iv.empty())
                    {
                        std::nth_element(iv.begin(), iv.begin() + iv.size() / 2, iv.end());
                        jump_interval_ms = iv[iv.size() / 2] * 1e3;
                    }
                }
            }
            prev = ep;
            prev_tow = info.tow_ms;
            have_prev = true;
        }, scan);
        ASSERT_TRUE(read) << path;
        ASSERT_FALSE(picks.empty());
        std::vector<int> sorted = picks;
        std::sort(sorted.begin(), sorted.end());
        const auto pct = [&](double q) { return sorted[static_cast<size_t>(q * (sorted.size() - 1))]; };
        const size_t in_band = static_cast<size_t>(
            std::count_if(picks.begin(), picks.end(), [](int n) { return n >= 18 && n <= 28; }));
        std::printf("[capture] %s: %zu epochs with picks (%u empty, %u refused); picks min %d p1 %d p5 %d median %d "
                    "max %d, %.2f %% in 18-28; GEO left out %u, duplicates %u, over %u; cells without code %u, "
                    "without Doppler %u; clock jumps %u (locks spanning one %u), GPS interval across it %.4f ms; "
                    "start-overs %u; L1 level switches %u, locks going back %u; epochs under 18 picks from %.1f s "
                    "to %.1f s of %.1f s\n",
                    sbftest::captureName(path).c_str(), picks.size(), empty, refused, sorted.front(), pct(0.01),
                    pct(0.05), pct(0.5), sorted.back(), 100.0 * in_band / picks.size(), geo, dupes, over, no_pr,
                    no_rate, jumps, span_locks, jump_interval_ms, js.rebases, level_switches, lock_backs, low_from_s,
                    low_to_s, (js.last_ms - first_ms) * 1e-3);
        EXPECT_EQ(scan.crc_fails, 0u);
        EXPECT_EQ(refused, 0u);
        // 18-28 L1 picks an epoch, but for the end of the nose-cone log
        // (its last 29 s lose the sky: 562 epochs, 332 of them empty)
        EXPECT_LE(sorted.back(), 28);
        EXPECT_GE(static_cast<double>(in_band) / picks.size(), 0.98);
        EXPECT_EQ(over, 0u);
        EXPECT_EQ(span_locks, 0u);
        EXPECT_EQ(lock_backs, 0u);
        EXPECT_EQ(js.rebases, 0u);
        if (jumps > 0 && jump_interval_ms != 0.0) EXPECT_NEAR(jump_interval_ms, 51.0, 0.01);
    }
}
