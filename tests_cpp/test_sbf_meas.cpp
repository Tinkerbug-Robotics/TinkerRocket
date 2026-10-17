// Host tests for the mosaic-G5's raw-measurement readers
// (TR_GNSSReceiverMosaic_Serial/SbfMeas.cpp): MeasEpoch's two levels folded
// back to whole signals, every do-not-use pair, the SigIdxLo 31 extension,
// GLONASS frequency numbers, the layout checks against Length, and
// MeasExtra's sub-block count and revision-gated fields. Blocks are built
// from their fields (Reference Guide, firmware 1.0.1, 4.2.1) with made-up
// values. With TR_SBF_PRIVATE_CAPTURE set, both 10/16 V9 captures are read
// end to end as well (statistics only).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <vector>

#include "SbfMeas.h"
#include "sbf_build.h"
#include "sbf_meas_build.h"
#include "sbf_private_capture.h"

using namespace mosaic;
using sbftest::MeasEpochSpec;
using sbftest::MeasT1;
using sbftest::MeasT2;

namespace {

constexpr double kC = 299792458.0;

std::vector<SbfMeasSignal> readAll(const std::vector<uint8_t>& blk, SbfStatus* st = nullptr, uint16_t* skipped = nullptr)
{
    SbfMeasReader r;
    const SbfStatus s = r.begin(blk.data(), blk.size());
    if (st != nullptr) *st = s;
    std::vector<SbfMeasSignal> out;
    SbfMeasSignal sig;
    while (r.next(sig)) out.push_back(sig);
    if (skipped != nullptr) *skipped = r.skipped();
    return out;
}

// A GPS satellite on L1 C/A, its fields all valid.
MeasT1 gpsL1(uint8_t svid, uint8_t channel = 1)
{
    MeasT1 t;
    t.rx_channel = channel;
    t.sig(0);
    t.svid = svid;
    t.codeMm(21123456789LL);
    t.doppler = -12345678;          // -1234.5678 Hz
    t.carrier(-1234567);
    t.cn0 = 130;                    // 42.5 dB-Hz
    t.lock = 345;
    return t;
}

}  // namespace

TEST(SbfSvid, MapsEveryRangeOfTheGuide)
{
    uint8_t prn = 99;
    EXPECT_EQ(sbfSvid(0, prn), SbfSys::NONE);
    EXPECT_EQ(sbfSvid(1, prn), SbfSys::GPS);    EXPECT_EQ(prn, 1);
    EXPECT_EQ(sbfSvid(37, prn), SbfSys::GPS);   EXPECT_EQ(prn, 37);
    EXPECT_EQ(sbfSvid(38, prn), SbfSys::GLO);   EXPECT_EQ(prn, 1);
    EXPECT_EQ(sbfSvid(61, prn), SbfSys::GLO);   EXPECT_EQ(prn, 24);
    EXPECT_EQ(sbfSvid(62, prn), SbfSys::GLO);   EXPECT_EQ(prn, 0);    // slot not known
    EXPECT_EQ(sbfSvid(63, prn), SbfSys::GLO);   EXPECT_EQ(prn, 25);
    EXPECT_EQ(sbfSvid(68, prn), SbfSys::GLO);   EXPECT_EQ(prn, 30);
    EXPECT_EQ(sbfSvid(69, prn), SbfSys::NONE);
    EXPECT_EQ(sbfSvid(70, prn), SbfSys::NONE);
    EXPECT_EQ(sbfSvid(71, prn), SbfSys::GAL);   EXPECT_EQ(prn, 1);
    EXPECT_EQ(sbfSvid(106, prn), SbfSys::GAL);  EXPECT_EQ(prn, 36);
    EXPECT_EQ(sbfSvid(107, prn), SbfSys::LBAND);
    EXPECT_EQ(sbfSvid(120, prn), SbfSys::SBAS); EXPECT_EQ(prn, 120);
    EXPECT_EQ(sbfSvid(141, prn), SbfSys::BDS);  EXPECT_EQ(prn, 1);
    EXPECT_EQ(sbfSvid(180, prn), SbfSys::BDS);  EXPECT_EQ(prn, 40);
    EXPECT_EQ(sbfSvid(181, prn), SbfSys::QZS);  EXPECT_EQ(prn, 1);
    EXPECT_EQ(sbfSvid(191, prn), SbfSys::NAVIC); EXPECT_EQ(prn, 1);
    EXPECT_EQ(sbfSvid(198, prn), SbfSys::SBAS); EXPECT_EQ(prn, 141);
    EXPECT_EQ(sbfSvid(216, prn), SbfSys::NAVIC); EXPECT_EQ(prn, 8);
    EXPECT_EQ(sbfSvid(223, prn), SbfSys::BDS);  EXPECT_EQ(prn, 41);   // C41
    EXPECT_EQ(sbfSvid(245, prn), SbfSys::BDS);  EXPECT_EQ(prn, 63);   // C63
    EXPECT_EQ(sbfSvid(246, prn), SbfSys::NONE);
}

TEST(SbfSignal, CarriersAndUndefinedNumbers)
{
    EXPECT_DOUBLE_EQ(sbfCarrierHz(0), 1575.42e6);
    EXPECT_DOUBLE_EQ(sbfCarrierHz(17), 1575.42e6);
    EXPECT_DOUBLE_EQ(sbfCarrierHz(28), 1561.098e6);
    EXPECT_DOUBLE_EQ(sbfCarrierHz(30), 1268.52e6);
    EXPECT_DOUBLE_EQ(sbfCarrierHz(20), 1176.45e6);
    EXPECT_DOUBLE_EQ(sbfCarrierHz(21), 1207.14e6);
    EXPECT_DOUBLE_EQ(sbfCarrierHz(13), 1575.42e6);   // B1C
    EXPECT_DOUBLE_EQ(sbfCarrierHz(8, -7), 1602.0e6 - 7 * 562.5e3);
    EXPECT_DOUBLE_EQ(sbfCarrierHz(11, 6), 1246.0e6 + 6 * 437.5e3);
    EXPECT_EQ(sbfCarrierHz(8), 0.0);                 // GLONASS without its frequency number
    EXPECT_EQ(sbfCarrierHz(23), 0.0);                // L-band: beam specific
    for (uint8_t s : {16, 18, 31, 35, 36, 40, 63})
    {
        EXPECT_FALSE(sbfSignalDefined(s)) << int(s);
        EXPECT_EQ(sbfCarrierHz(s), 0.0) << int(s);
    }
    for (uint8_t s : {0, 1, 17, 23, 28, 32, 34, 37, 39}) EXPECT_TRUE(sbfSignalDefined(s)) << int(s);
}

TEST(SbfMeasReader, ReadsAType1InFull)
{
    MeasEpochSpec s;
    s.cum = 251;
    MeasT1 t = gpsL1(7, 3);
    t.codeMm((2LL << 32) + 123456789);
    t.obs = 0x05;   // smoothed, half-cycle
    s.t1 = {t};
    const auto blk = sbftest::measEpoch(s);

    SbfMeasReader r;
    ASSERT_EQ(r.begin(blk.data(), blk.size()), SbfStatus::OK);
    EXPECT_EQ(r.header().tow_ms, 507606000u);
    EXPECT_EQ(r.header().wnc, 2440);
    EXPECT_EQ(r.header().revision, 1);
    EXPECT_EQ(r.header().n1, 1);
    EXPECT_EQ(r.header().common_flags, 0x25);
    EXPECT_TRUE(r.header().has_cum_clk_jumps);
    EXPECT_EQ(r.header().cum_clk_jumps, 251);
    SbfMeasSignal g;
    ASSERT_TRUE(r.next(g));
    EXPECT_TRUE(g.master);
    EXPECT_EQ(g.rx_channel, 3);
    EXPECT_EQ(g.svid, 7);
    EXPECT_EQ(g.sig, 0);
    EXPECT_EQ(g.antenna, 0);
    EXPECT_TRUE(g.pr_valid);
    EXPECT_EQ(g.pr_mm, (2LL << 32) + 123456789);
    EXPECT_DOUBLE_EQ(g.prM(), ((2LL << 32) + 123456789) * 1e-3);
    EXPECT_TRUE(g.dop_valid);
    EXPECT_DOUBLE_EQ(g.doppler_hz, -1234.5678);
    EXPECT_TRUE(g.carrier_valid);
    EXPECT_EQ(g.carrier_mcyc, -1234567);
    const double lam = kC / 1575.42e6;
    EXPECT_DOUBLE_EQ(g.phaseRangeM(lam), g.prM() - 1234.567 * lam);
    EXPECT_TRUE(g.cn0Valid());
    EXPECT_DOUBLE_EQ(g.cn0Dbhz(), 130 * 0.25 + 10.0);
    EXPECT_TRUE(g.lockValid());
    EXPECT_EQ(g.lock_s, 345);
    EXPECT_TRUE(g.half_cycle);
    EXPECT_TRUE(g.smoothed);
    EXPECT_FALSE(r.next(g));
    EXPECT_EQ(r.skipped(), 0);
}

TEST(SbfMeasReader, CnoHasNoTenAddedForGpsL1PAndL2P)
{
    MeasEpochSpec s;
    MeasT1 t = gpsL1(5);
    t.sig(2);                        // L2P master
    t.cn0 = 160;
    MeasT2 u;
    u.sig(1);                        // L1P slave
    u.cn0 = 160;
    t.t2 = {u};
    s.t1 = {t};
    const auto sigs = readAll(sbftest::measEpoch(s));
    ASSERT_EQ(sigs.size(), 2u);
    EXPECT_DOUBLE_EQ(sigs[0].cn0Dbhz(), 40.0);
    EXPECT_DOUBLE_EQ(sigs[1].cn0Dbhz(), 40.0);
}

TEST(SbfMeasReader, FoldsType2OntoItsMaster)
{
    // Galileo with E5a as master and E1 as its slave, as on 3.8 % of the
    // 10/16 Galileo satellite-epochs
    MeasEpochSpec s;
    MeasT1 t;
    t.rx_channel = 12;
    t.sig(20);                       // E5a
    t.svid = 71 + 10;                // E11
    t.codeMm(23456789012LL);
    t.doppler = 9876543;             // 987.6543 Hz
    t.carrier(4321);
    MeasT2 u;
    u.sig(17);                       // E1
    u.lock = 254;
    u.cn0 = 120;
    u.codeOffsetMm(-70000);          // 70 m below the master's code (MSB -2)
    u.dopplerOffset(-1234);          // -0.1234 Hz
    u.carrier(-987654);
    u.obs = 0x04;
    MeasT2 w;
    w.sig(21, 1);                    // E5b on Aux1
    w.codeOffsetMm(262143);          // the largest offset
    w.dopplerOffset(1048575);
    t.t2 = {u, w};
    s.t1 = {t};
    const auto sigs = readAll(sbftest::measEpoch(s));
    ASSERT_EQ(sigs.size(), 3u);

    const SbfMeasSignal& e1 = sigs[1];
    EXPECT_FALSE(e1.master);
    EXPECT_EQ(e1.rx_channel, 12);
    EXPECT_EQ(e1.svid, 81);
    EXPECT_EQ(e1.sig, 17);
    EXPECT_TRUE(e1.pr_valid);
    EXPECT_EQ(e1.pr_mm, 23456789012LL - 70000);
    EXPECT_TRUE(e1.dop_valid);
    EXPECT_NEAR(e1.doppler_hz, 987.6543 * 1575.42 / 1176.45 - 0.1234, 1e-9);
    EXPECT_TRUE(e1.carrier_valid);
    EXPECT_EQ(e1.carrier_mcyc, -987654);
    EXPECT_EQ(e1.lock_s, 254);
    EXPECT_DOUBLE_EQ(e1.cn0Dbhz(), 40.0);
    EXPECT_TRUE(e1.half_cycle);
    EXPECT_EQ(e1.antenna, 0);

    const SbfMeasSignal& e5b = sigs[2];
    EXPECT_EQ(e5b.antenna, 1);
    EXPECT_EQ(e5b.pr_mm, 23456789012LL + 262143);
    EXPECT_NEAR(e5b.doppler_hz, 987.6543 * 1207.14 / 1176.45 + 104.8575, 1e-9);
}

TEST(SbfMeasReader, EveryDoNotUsePairAndOnlyThePair)
{
    MeasEpochSpec s;
    // Type1 code: invalid only with both parts 0
    MeasT1 a = gpsL1(1, 1);
    a.code_msb = 0; a.code_lsb = 0;
    MeasT1 b = gpsL1(2, 2);
    b.code_msb = 0; b.code_lsb = 1;
    MeasT1 c = gpsL1(3, 3);
    c.code_msb = 1; c.code_lsb = 0;
    // Doppler, carrier (-128 with LSB 0), C/N0 255, lock 65535
    MeasT1 d = gpsL1(4, 4);
    d.doppler = -2147483647 - 1;
    d.car_msb = -128; d.car_lsb = 0;
    d.cn0 = 255;
    d.lock = 65535;
    MeasT1 e = gpsL1(5, 5);
    e.car_msb = -128; e.car_lsb = 1;
    MeasT1 f = gpsL1(6, 6);
    f.car_msb = -127; f.car_lsb = 0;
    // Type2: code (-4, 0), Doppler (-16, 0), carrier (-128, 0), lock 255, and their near misses
    MeasT1 g = gpsL1(8, 8);
    g.sig(4);   // L5 master
    MeasT2 g1; g1.sig(0); g1.code_msb = -4; g1.code_lsb = 0; g1.dop_msb = -16; g1.dop_lsb = 0;
    g1.car_msb = -128; g1.car_lsb = 0; g1.lock = 255; g1.cn0 = 255;
    MeasT2 g2; g2.sig(3); g2.code_msb = -4; g2.code_lsb = 1; g2.dop_msb = -16; g2.dop_lsb = 1;
    MeasT2 g3; g3.sig(2); g3.code_msb = -3; g3.code_lsb = 0; g3.dop_msb = -15; g3.dop_lsb = 0;
    g.t2 = {g1, g2, g3};
    s.t1 = {a, b, c, d, e, f, g};
    const auto sigs = readAll(sbftest::measEpoch(s));
    ASSERT_EQ(sigs.size(), 10u);

    EXPECT_FALSE(sigs[0].pr_valid);
    EXPECT_TRUE(sigs[1].pr_valid);
    EXPECT_EQ(sigs[1].pr_mm, 1);
    EXPECT_TRUE(sigs[2].pr_valid);
    EXPECT_EQ(sigs[2].pr_mm, 1LL << 32);
    EXPECT_FALSE(sigs[3].dop_valid);
    EXPECT_FALSE(sigs[3].carrier_valid);
    EXPECT_FALSE(sigs[3].cn0Valid());
    EXPECT_FALSE(sigs[3].lockValid());
    EXPECT_TRUE(sigs[3].pr_valid);
    EXPECT_TRUE(sigs[4].carrier_valid);
    EXPECT_EQ(sigs[4].carrier_mcyc, -128 * 65536 + 1);
    EXPECT_TRUE(sigs[5].carrier_valid);
    EXPECT_EQ(sigs[5].carrier_mcyc, -127 * 65536);

    EXPECT_TRUE(sigs[6].master);
    EXPECT_FALSE(sigs[7].pr_valid);
    EXPECT_FALSE(sigs[7].dop_valid);
    EXPECT_FALSE(sigs[7].carrier_valid);
    EXPECT_FALSE(sigs[7].lockValid());
    EXPECT_EQ(sigs[7].lock_s, 65535);   // a Type2's 255 reads as the Type1's do-not-use
    EXPECT_FALSE(sigs[7].cn0Valid());
    EXPECT_TRUE(sigs[8].pr_valid);
    EXPECT_EQ(sigs[8].pr_mm, sigs[6].pr_mm - 4 * 65536 + 1);
    EXPECT_TRUE(sigs[8].dop_valid);
    EXPECT_TRUE(sigs[9].pr_valid);
    EXPECT_EQ(sigs[9].pr_mm, sigs[6].pr_mm - 3 * 65536);
    EXPECT_TRUE(sigs[9].dop_valid);
}

TEST(SbfMeasReader, AType2NeedsItsMastersCodeAndDoppler)
{
    MeasEpochSpec s;
    MeasT1 t = gpsL1(9);
    t.sig(4);
    t.code_msb = 0; t.code_lsb = 0;        // master code do-not-use
    t.doppler = -2147483647 - 1;          // master Doppler do-not-use
    MeasT2 u;
    u.sig(0);
    u.codeOffsetMm(100);
    u.dopplerOffset(5);
    t.t2 = {u};
    s.t1 = {t};
    const auto sigs = readAll(sbftest::measEpoch(s));
    ASSERT_EQ(sigs.size(), 2u);
    EXPECT_FALSE(sigs[1].pr_valid);
    EXPECT_FALSE(sigs[1].dop_valid);
    EXPECT_TRUE(sigs[1].carrier_valid);
}

TEST(SbfMeasReader, SignalNumbersAbove31ComeFromObsInfo)
{
    MeasEpochSpec s;
    MeasT1 t;
    t.svid = 183;                          // QZSS J03
    t.type = 31;                           // SigIdxLo 31
    t.obs = static_cast<uint8_t>((33 - 32) << 3);   // L1S
    t.codeMm(38000000000LL);
    MeasT2 u;
    u.type = 31;
    u.obs = static_cast<uint8_t>((38 - 32) << 3) | 0x04;   // L1CB, half-cycle
    t.t2 = {u};
    s.t1 = {t};
    const auto sigs = readAll(sbftest::measEpoch(s));
    ASSERT_EQ(sigs.size(), 2u);
    EXPECT_EQ(sigs[0].sig, 33);
    EXPECT_EQ(sigs[1].sig, 38);
    EXPECT_TRUE(sigs[1].half_cycle);
    EXPECT_EQ(sigs[0].glo_fn, kGloFnUnknown);
}

TEST(SbfMeasReader, GlonassType2InheritsTheMastersFrequencyNumber)
{
    MeasEpochSpec s;
    MeasT1 t;
    t.svid = 38 + 4;                       // R05
    t.sig(8);                              // L1CA
    t.obs = static_cast<uint8_t>((-5 + 8) << 3);   // FreqNr 3: frequency number -5
    t.codeMm(20000000000LL);
    t.doppler = 20000000;                  // 2000 Hz
    MeasT2 u;
    u.sig(11);                             // L2CA; its ObsInfo bits 3-7 are 0, as on the captures
    u.dopplerOffset(0);
    t.t2 = {u};
    s.t1 = {t};
    const auto sigs = readAll(sbftest::measEpoch(s));
    ASSERT_EQ(sigs.size(), 2u);
    EXPECT_EQ(sigs[0].glo_fn, -5);
    EXPECT_EQ(sigs[1].glo_fn, -5);
    const double f1 = 1602.0e6 - 5 * 562.5e3, f2 = 1246.0e6 - 5 * 437.5e3;
    EXPECT_DOUBLE_EQ(sbfCarrierHz(sigs[1].sig, sigs[1].glo_fn), f2);
    EXPECT_NEAR(sigs[1].doppler_hz, 2000.0 * f2 / f1, 1e-9);
}

TEST(SbfMeasReader, AnEmptyEpochIsTwentyBytes)
{
    MeasEpochSpec s;
    s.sb1 = 0;                              // nothing to stride over
    s.sb2 = 0;
    const auto blk = sbftest::measEpoch(s);
    EXPECT_EQ(blk.size(), 20u);
    SbfStatus st;
    EXPECT_TRUE(readAll(blk, &st).empty());
    EXPECT_EQ(st, SbfStatus::OK);
}

TEST(SbfMeasReader, StridesComeFromTheSubBlockLengths)
{
    MeasEpochSpec s;
    s.sb1 = 28;                            // padded Type1s and Type2s, as a later revision may send
    s.sb2 = 16;
    MeasT1 a = gpsL1(1, 1);
    a.sig(4);
    MeasT2 u; u.sig(0); u.codeOffsetMm(-1234);
    MeasT2 w; w.sig(3); w.codeOffsetMm(5678);
    a.t2 = {u, w};
    s.t1 = {a, gpsL1(2, 2)};
    const auto sigs = readAll(sbftest::measEpoch(s));
    ASSERT_EQ(sigs.size(), 4u);
    EXPECT_EQ(sigs[1].pr_mm, a.code_lsb + (static_cast<int64_t>(a.code_msb) << 32) - 1234);
    EXPECT_EQ(sigs[2].pr_mm, a.code_lsb + (static_cast<int64_t>(a.code_msb) << 32) + 5678);
    EXPECT_EQ(sigs[3].svid, 2);
    EXPECT_TRUE(sigs[3].master);
}

TEST(SbfMeasReader, RefusesTruncatedAndOverrunBlocks)
{
    MeasEpochSpec s;
    MeasT1 a = gpsL1(1, 1);
    a.t2 = {MeasT2{}};
    s.t1 = {a, gpsL1(2, 2)};
    const auto blk = sbftest::measEpoch(s);
    SbfMeasReader r;
    SbfMeasSignal g;

    // fewer bytes than Length
    EXPECT_EQ(r.begin(blk.data(), blk.size() - 4), SbfStatus::SHORT);
    EXPECT_FALSE(r.next(g));
    EXPECT_EQ(r.begin(blk.data(), 7), SbfStatus::SHORT);
    EXPECT_EQ(r.begin(nullptr, 0), SbfStatus::SHORT);

    // N1 claims a third Type1 the block has no room for
    MeasEpochSpec s3 = s;
    s3.n1 = 3;
    const auto over = sbftest::measEpoch(s3);
    EXPECT_EQ(r.begin(over.data(), over.size()), SbfStatus::BAD_LAYOUT);
    EXPECT_FALSE(r.next(g));

    // an N2 past the end
    auto n2 = sbftest::measEpoch(s);
    n2[20 + 19] = 9;
    const uint16_t crc = crc16Ccitt(n2.data() + 4, n2.size() - 4);
    n2[2] = crc & 0xFF;
    n2[3] = crc >> 8;
    EXPECT_EQ(r.begin(n2.data(), n2.size()), SbfStatus::BAD_LAYOUT);

    // a Length too small for the header
    std::vector<uint8_t> tiny = sbftest::block(4027, 1, 1000, 2440, {0, 20, 12, 0});
    EXPECT_EQ(tiny.size(), 20u);
    tiny[6] = 16;
    EXPECT_EQ(r.begin(tiny.data(), tiny.size()), SbfStatus::SHORT);
}

TEST(SbfMeasReader, RefusesSubBlockLengthsThatCannotBe)
{
    MeasEpochSpec s;
    s.sb1 = 19;
    s.t1 = {gpsL1(1)};
    auto blk = sbftest::measEpoch(s);
    SbfMeasReader r;
    EXPECT_EQ(r.begin(blk.data(), blk.size()), SbfStatus::BAD_LAYOUT);

    MeasEpochSpec t;
    t.sb2 = 11;
    MeasT1 a = gpsL1(1);
    a.t2 = {MeasT2{}};
    t.t1 = {a};
    blk = sbftest::measEpoch(t);
    EXPECT_EQ(r.begin(blk.data(), blk.size()), SbfStatus::BAD_LAYOUT);

    // with no Type2 anywhere, SB2Length does not matter
    t.t1 = {gpsL1(1)};
    blk = sbftest::measEpoch(t);
    EXPECT_EQ(r.begin(blk.data(), blk.size()), SbfStatus::OK);
}

TEST(SbfMeasReader, RevisionZeroHasNoClockJumpCount)
{
    MeasEpochSpec s;
    s.rev = 0;
    s.cum = 77;                            // a reserved byte in revision 0
    s.t1 = {gpsL1(1)};
    const auto blk = sbftest::measEpoch(s);
    SbfMeasReader r;
    ASSERT_EQ(r.begin(blk.data(), blk.size()), SbfStatus::OK);
    EXPECT_FALSE(r.header().has_cum_clk_jumps);
    EXPECT_EQ(r.header().cum_clk_jumps, 0);
}

TEST(SbfMeasReader, ScrambledAndUntimedEpochsGiveNoSignals)
{
    MeasEpochSpec s;
    s.flags = 0x25 | 0x80;
    s.t1 = {gpsL1(1)};
    SbfStatus st;
    EXPECT_TRUE(readAll(sbftest::measEpoch(s), &st).empty());
    EXPECT_EQ(st, SbfStatus::SCRAMBLED);

    MeasEpochSpec t;
    t.t1 = {gpsL1(1)};
    t.tow = kTowDnu;
    EXPECT_TRUE(readAll(sbftest::measEpoch(t), &st).empty());
    EXPECT_EQ(st, SbfStatus::NO_TIME);
    t.tow = 1000;
    t.wnc = kWncDnu;
    EXPECT_TRUE(readAll(sbftest::measEpoch(t), &st).empty());
    EXPECT_EQ(st, SbfStatus::NO_TIME);
}

TEST(SbfMeasReader, RefusesOtherBlocks)
{
    MeasEpochSpec s;
    s.t1 = {gpsL1(1)};
    auto blk = sbftest::measEpoch(s);
    SbfMeasReader r;
    auto other = blk;
    other[4] = SBF_PVT_GEODETIC & 0xFF;
    other[5] = static_cast<uint8_t>((SBF_PVT_GEODETIC >> 8) | (1 << 5));
    EXPECT_EQ(r.begin(other.data(), other.size()), SbfStatus::WRONG_BLOCK);
    other = blk;
    other[1] = '#';
    EXPECT_EQ(r.begin(other.data(), other.size()), SbfStatus::WRONG_BLOCK);
    SbfExtraReader x;
    EXPECT_EQ(x.begin(blk.data(), blk.size()), SbfStatus::WRONG_BLOCK);
}

TEST(SbfMeasReader, SkipsUndefinedSatellitesAndSignals)
{
    MeasEpochSpec s;
    MeasT1 bad_sv = gpsL1(1, 1);
    bad_sv.svid = 70;                      // undefined: it and its Type2s go
    bad_sv.t2 = {MeasT2{}, MeasT2{}};
    MeasT1 bad_sig = gpsL1(2, 2);
    bad_sig.sig(16);                       // reserved master: its Type2s still read
    MeasT2 l1; l1.sig(0); l1.codeOffsetMm(10); l1.dopplerOffset(3);
    MeasT2 rsv; rsv.sig(18);
    bad_sig.t2 = {l1, rsv};
    s.t1 = {bad_sv, bad_sig, gpsL1(3, 3)};
    uint16_t skipped = 0;
    const auto sigs = readAll(sbftest::measEpoch(s), nullptr, &skipped);
    ASSERT_EQ(sigs.size(), 2u);
    EXPECT_EQ(skipped, 5);
    EXPECT_EQ(sigs[0].svid, 2);
    EXPECT_EQ(sigs[0].sig, 0);
    EXPECT_FALSE(sigs[0].master);
    EXPECT_TRUE(sigs[0].pr_valid);
    EXPECT_FALSE(sigs[0].dop_valid);       // no master frequency to scale a Doppler from
    EXPECT_EQ(sigs[1].svid, 3);
}

// ── MeasExtra ───────────────────────────────────────────────────────────

namespace {

struct ExtraSub
{
    uint8_t ch = 1, type = 0;
    int16_t mp = 0, smooth = 0;
    uint16_t code_var = 100, car_var = 50, lock = 1234;
    uint8_t cum = 0;
    int8_t car_mp = 0;
    uint8_t info = 100, misc = 0;
};

std::vector<uint8_t> measExtra(uint8_t rev, uint8_t sb_len, const std::vector<ExtraSub>& subs, float dvf = 651.2f,
                               uint32_t tow = 507606000, int n_override = -1)
{
    sbftest::Bytes b;
    b.u8(static_cast<uint8_t>(n_override >= 0 ? n_override : static_cast<int>(subs.size() & 0xFF)))
     .u8(sb_len).f32(dvf);
    for (const ExtraSub& e : subs)
    {
        sbftest::Bytes x;
        x.u8(e.ch).u8(e.type).i16(e.mp).i16(e.smooth).u16(e.code_var).u16(e.car_var).u16(e.lock)
         .u8(e.cum).i8(e.car_mp).u8(e.info).u8(e.misc);
        x.v.resize(sb_len, 0);
        b.v.insert(b.v.end(), x.v.begin(), x.v.end());
    }
    return sbftest::block(SBF_MEAS_EXTRA, rev, tow, 2440, b.v);
}

}  // namespace

TEST(SbfExtraReader, ReadsRevisionThreeSubBlocks)
{
    ExtraSub a;
    a.ch = 7; a.type = 17 | (0 << 5); a.mp = -321; a.smooth = 45; a.code_var = 65535; a.car_var = 12;
    a.lock = 65535; a.cum = 9; a.car_mp = -3; a.misc = 5;
    ExtraSub q;
    q.ch = 8; q.type = 31; q.misc = static_cast<uint8_t>(((34 - 32) << 3) | 2);   // B2b via the extension
    const auto blk = measExtra(3, 16, {a, q});
    SbfExtraReader r;
    ASSERT_EQ(r.begin(blk.data(), blk.size()), SbfStatus::OK);
    EXPECT_EQ(r.header().n_sb, 2);
    EXPECT_EQ(r.header().sb_len, 16);
    EXPECT_FLOAT_EQ(r.header().doppler_var_factor, 651.2f);
    SbfExtraSignal x;
    ASSERT_TRUE(r.next(x));
    EXPECT_EQ(x.rx_channel, 7);
    EXPECT_EQ(x.sig, 17);
    EXPECT_EQ(x.mp_correction_mm, -321);
    EXPECT_EQ(x.smoothing_corr_mm, 45);
    EXPECT_EQ(x.code_var, 65535);
    EXPECT_EQ(x.carrier_var, 12);
    EXPECT_EQ(x.lock_s, 65535);
    EXPECT_TRUE(x.has_cum_loss_cont);
    EXPECT_EQ(x.cum_loss_cont, 9);
    EXPECT_TRUE(x.has_car_mp_corr);
    EXPECT_EQ(x.car_mp_corr, -3);
    EXPECT_TRUE(x.has_cn0_hires);
    EXPECT_EQ(x.cn0_hires, 5);
    ASSERT_TRUE(r.next(x));
    EXPECT_EQ(x.sig, 34);
    EXPECT_EQ(x.cn0_hires, 2);
    EXPECT_FALSE(r.next(x));
}

TEST(SbfExtraReader, FieldsFollowTheRevisionAndLength)
{
    ExtraSub a;
    a.cum = 3; a.car_mp = 4; a.misc = 7;
    SbfExtraReader r;
    SbfExtraSignal x;

    auto blk = measExtra(0, 12, {a});
    ASSERT_EQ(r.begin(blk.data(), blk.size()), SbfStatus::OK);
    ASSERT_TRUE(r.next(x));
    EXPECT_FALSE(x.has_cum_loss_cont);
    EXPECT_FALSE(x.has_car_mp_corr);
    EXPECT_FALSE(x.has_cn0_hires);
    EXPECT_EQ(x.lock_s, a.lock);

    blk = measExtra(1, 13, {a});
    ASSERT_EQ(r.begin(blk.data(), blk.size()), SbfStatus::OK);
    ASSERT_TRUE(r.next(x));
    EXPECT_TRUE(x.has_cum_loss_cont);
    EXPECT_EQ(x.cum_loss_cont, 3);
    EXPECT_FALSE(x.has_car_mp_corr);
    EXPECT_FALSE(x.has_cn0_hires);

    // a revision-2 block has no Misc, so a SigIdxLo of 31 names no signal
    ExtraSub q;
    q.type = 31;
    blk = measExtra(2, 16, {a, q});
    ASSERT_EQ(r.begin(blk.data(), blk.size()), SbfStatus::OK);
    ASSERT_TRUE(r.next(x));
    EXPECT_TRUE(x.has_car_mp_corr);
    EXPECT_FALSE(x.has_cn0_hires);
    EXPECT_FALSE(r.next(x));
    EXPECT_EQ(r.skipped(), 1);

    blk = measExtra(3, 11, {a});
    EXPECT_EQ(r.begin(blk.data(), blk.size()), SbfStatus::BAD_LAYOUT);
}

TEST(SbfExtraReader, CountsPast255SubBlocks)
{
    // N is the count mod 256: 300 sub-blocks send N = 44
    std::vector<ExtraSub> subs(300);
    for (size_t i = 0; i < subs.size(); i++) subs[i].ch = static_cast<uint8_t>(1 + i % 45);
    const auto blk = measExtra(3, 16, subs);
    EXPECT_EQ(blk[14], 44);
    SbfExtraReader r;
    ASSERT_EQ(r.begin(blk.data(), blk.size()), SbfStatus::OK);
    EXPECT_EQ(r.header().n_sb, 300);
    SbfExtraSignal x;
    int n = 0;
    while (r.next(x)) n++;
    EXPECT_EQ(n, 300);

    // an N that cannot fit in Length
    const auto over = measExtra(3, 16, {ExtraSub{}, ExtraSub{}}, 651.2f, 1000, 5);
    EXPECT_EQ(r.begin(over.data(), over.size()), SbfStatus::BAD_LAYOUT);
}

TEST(SbfExtraReader, DopplerSigmaFromTheCarrierVariance)
{
    const double lam = kC / 1575.42e6;
    // sigma^2 = 100 mcycle^2 * 651.2 Hz^2/cycle^2 = 65120 mHz^2
    EXPECT_NEAR(sbfDopplerSigmaMps(100, 651.2f, lam), std::sqrt(100.0 * 651.2f) * 1e-3 * lam, 1e-12);
    EXPECT_LT(sbfDopplerSigmaMps(65535, 651.2f, lam), 0.0);
    EXPECT_LT(sbfDopplerSigmaMps(100, 0.0f, lam), 0.0);
}

// ── The 10/16 captures (private; TR_SBF_PRIVATE_CAPTURE) ─────────────────

TEST(SbfMeasPrivateCapture, EveryMeasurementBlockReads)
{
    const auto paths = sbftest::privateCapturePaths();
    if (paths.empty()) GTEST_SKIP() << "TR_SBF_PRIVATE_CAPTURE not set";
    for (const std::string& path : paths)
    {
        uint32_t epochs = 0, extras = 0, bad = 0, scrambled = 0, untimed = 0, empty = 0, extra_bad = 0;
        uint64_t skipped = 0;
        uint32_t max_l1 = 0;
        std::map<int, uint32_t> where;   // signal * 2 + master: how often each L1 signal sat at each level
        std::map<SbfStatus, uint32_t> statuses;
        sbftest::CaptureScan scan;
        const bool read = sbftest::scanCapture(path, [&](const uint8_t* b, size_t len) {
            SbfHeader h;
            if (!sbfHeader(b, len, h)) return;
            if (h.number == SBF_MEAS_EPOCH)
            {
                epochs++;
                SbfMeasReader r;
                const SbfStatus st = r.begin(b, len);
                statuses[st]++;
                if (st == SbfStatus::SHORT || st == SbfStatus::BAD_LAYOUT || st == SbfStatus::WRONG_BLOCK) bad++;
                if (st == SbfStatus::SCRAMBLED) scrambled++;
                if (st == SbfStatus::NO_TIME) untimed++;
                if (r.header().n1 == 0) empty++;
                SbfMeasSignal s;
                uint32_t l1 = 0;
                while (r.next(s))
                {
                    uint8_t prn;
                    const SbfSys sys = sbfSvid(s.svid, prn);
                    if (s.antenna != 0) continue;
                    if ((sys == SbfSys::GPS && s.sig == 0) || (sys == SbfSys::GAL && s.sig == 17) ||
                        (sys == SbfSys::BDS && s.sig == 28 && prn > 5 && prn < 59))
                    {
                        l1++;
                        where[s.sig * 2 + (s.master ? 1 : 0)]++;
                    }
                }
                skipped += r.skipped();
                max_l1 = std::max(max_l1, l1);
            }
            else if (h.number == SBF_MEAS_EXTRA)
            {
                extras++;
                SbfExtraReader x;
                const SbfStatus st = x.begin(b, len);
                if (st != SbfStatus::OK && st != SbfStatus::NO_TIME) extra_bad++;
                SbfExtraSignal s;
                while (x.next(s)) {}
            }
        }, scan);
        ASSERT_TRUE(read) << path;
        std::printf("[capture] %s: %u blocks, %u CRC failures, %u bad lengths; MeasEpoch %u (layout bad %u, "
                    "scrambled %u, untimed %u, empty %u, skipped sub-blocks %llu); MeasExtra %u (bad %u); "
                    "L1 picks max %u\n",
                    sbftest::captureName(path).c_str(), scan.blocks, scan.crc_fails, scan.bad_lengths, epochs, bad,
                    scrambled, untimed, empty, static_cast<unsigned long long>(skipped), extras, extra_bad, max_l1);
        std::printf("[capture]   L1CA type1/type2 %u/%u, E1 %u/%u, B1I %u/%u\n", where[1], where[0], where[35],
                    where[34], where[57], where[56]);
        EXPECT_EQ(scan.crc_fails, 0u);
        EXPECT_EQ(scan.bad_lengths, 0u);
        EXPECT_GT(epochs, 1000u);
        EXPECT_EQ(bad, 0u);
        EXPECT_EQ(extra_bad, 0u);
        EXPECT_EQ(scrambled, 0u);
        EXPECT_LE(max_l1, 28u);
        EXPECT_GT(where[0] + where[34] + where[56], 0u);   // L1 signals found as Type2s too
    }
}
