// SkyTraq PX1105R binary stream (TR_GNSSReceiverPX1105R_Serial/SkyTraqBinary.h)
// and its 0xE5 epoch as the raw-GNSS filter's observations (Px1105rRawEpoch.h):
// framing and resync, the decoders on the receiver's own frames (its open-sky
// capture, 2026-09-26: fixtures/skytraq_frames.h), and the conventions the
// filter needs -- the Doppler's units and step, the lock time, and the epoch's
// GPS instant from the 0xDF before it. (The fixture's position is made up and
// its measurements scrambled: fixtures/skytraq_frames.h says how.)
//
// With PX1105R_CAPTURE naming that capture (the '<t> B <hex>' log, .gz read
// through zcat), the whole 20 minutes are run through as well.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "Px1105rRawEpoch.h"
#include "SkyTraqBinary.h"
#include "fixtures/skytraq_frames.h"

using namespace skytraq;

namespace
{

struct Collect : FrameSink
{
    std::vector<std::vector<uint8_t>> bodies;
    std::vector<size_t> index;
    void onFrame(const uint8_t* b, size_t len, size_t i) override
    {
        bodies.emplace_back(b, b + len);
        index.push_back(i);
    }
};

std::vector<uint8_t> framed(const uint8_t* body, size_t n)
{
    std::vector<uint8_t> f(n + 7);
    EXPECT_EQ(buildFrame(body, n, f.data(), f.size()), n + 7);
    return f;
}

}  // namespace

TEST(SkyTraqFrame, BuildsTheFrameTheRigToolsSend)
{
    // skytraq_raw.py frame(b"\x02\x01") = a0 a1 00 02 02 01 03 0d 0a
    const uint8_t q[] = {0x02, 0x01};
    const std::vector<uint8_t> f = framed(q, sizeof q);
    const uint8_t want[] = {0xA0, 0xA1, 0x00, 0x02, 0x02, 0x01, 0x03, 0x0D, 0x0A};
    ASSERT_EQ(f.size(), sizeof want);
    EXPECT_EQ(0, std::memcmp(f.data(), want, sizeof want));
    uint8_t small[8];
    EXPECT_EQ(buildFrame(q, sizeof q, small, sizeof small), 0u);
}

TEST(SkyTraqFrame, ParsesRealFramesInAnySlicing)
{
    std::vector<uint8_t> stream;
    const char* nmea = "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    stream.insert(stream.end(), nmea, nmea + std::strlen(nmea));
    for (const auto* b : {&kE5[0], &kDf[0], &kE7[0]})
    {
        const size_t n = b == kE5 ? sizeof kE5 : (b == kDf ? sizeof kDf : sizeof kE7);
        const std::vector<uint8_t> f = framed(b, n);
        stream.insert(stream.end(), f.begin(), f.end());
    }
    stream.insert(stream.end(), nmea, nmea + std::strlen(nmea));
    for (size_t slice : {size_t(1), size_t(7), size_t(64), size_t(512), stream.size()})
    {
        FrameParser p;
        Collect c;
        for (size_t i = 0; i < stream.size(); i += slice)
            p.feed(stream.data() + i, std::min(slice, stream.size() - i), c);
        ASSERT_EQ(c.bodies.size(), 3u) << "slice " << slice;
        EXPECT_EQ(c.bodies[0].size(), sizeof kE5);
        EXPECT_EQ(0, std::memcmp(c.bodies[0].data(), kE5, sizeof kE5));
        EXPECT_EQ(0, std::memcmp(c.bodies[1].data(), kDf, sizeof kDf));
        EXPECT_EQ(0, std::memcmp(c.bodies[2].data(), kE7, sizeof kE7));
        EXPECT_EQ(p.frames(), 3u);
        EXPECT_EQ(p.badChecksums(), 0u);
        EXPECT_EQ(p.nmeaLines(), 2u);
    }
}

TEST(SkyTraqFrame, ResyncsPastAFalseSyncAndACorruptFrame)
{
    std::vector<uint8_t> stream;
    // a false sync whose length runs into the next real frame
    const uint8_t junk[] = {0x11, 0xA0, 0xA1, 0x00, 0x30, 0x55};
    stream.insert(stream.end(), junk, junk + sizeof junk);
    std::vector<uint8_t> df = framed(kDf, sizeof kDf);
    stream.insert(stream.end(), df.begin(), df.end());
    // a corrupt copy (one body byte flipped), then a good one
    std::vector<uint8_t> bad = df;
    bad[20] ^= 0x40;
    stream.insert(stream.end(), bad.begin(), bad.end());
    stream.insert(stream.end(), df.begin(), df.end());
    // an impossible length
    const uint8_t huge[] = {0xA0, 0xA1, 0xFF, 0xFF};
    stream.insert(stream.end(), huge, huge + sizeof huge);
    stream.insert(stream.end(), df.begin(), df.end());

    FrameParser p;
    Collect c;
    p.feed(stream.data(), stream.size(), c);
    ASSERT_EQ(c.bodies.size(), 3u);
    for (const auto& b : c.bodies)
        EXPECT_EQ(0, std::memcmp(b.data(), kDf, sizeof kDf));
    EXPECT_GE(p.badChecksums(), 2u);   // the false sync and the flipped byte
    EXPECT_EQ(p.badLengths(), 1u);
    // the index is the byte that completed each frame
    EXPECT_EQ(c.index.back(), stream.size() - 1);
}

TEST(SkyTraqDecode, TheRawMeasurementsOfOneOpenSkyEpoch)
{
    RawE5 e;
    ASSERT_TRUE(decodeRawE5(kE5, sizeof kE5, e));
    EXPECT_EQ(e.version, 1);
    EXPECT_EQ(e.week, 2437);
    EXPECT_EQ(e.tow_ms, 569313300u);
    EXPECT_EQ(e.period_ms, 50);
    EXPECT_EQ(e.n, 35);
    // the first signal (its measurements the fixture's scrambled ones)
    const RawCell& c = e.cell[0];
    EXPECT_EQ(c.gnss, kGnssGps);
    EXPECT_EQ(c.sig, kSigL1);
    EXPECT_EQ(c.svid, 23);
    EXPECT_EQ(c.lock_ind, 12);
    EXPECT_EQ(c.cn0, 46);
    EXPECT_DOUBLE_EQ(c.pr_m, 20676544.787412986);
    EXPECT_DOUBLE_EQ(c.carrier_cyc, -1554753.721093995);
    EXPECT_FLOAT_EQ(c.doppler_hz, 2308.0f);
    EXPECT_EQ(c.ind, kIndPr | kIndDoppler | kIndCarrier);
    // too short for its count: refused
    EXPECT_FALSE(decodeRawE5(kE5, sizeof kE5 - 1, e));
    EXPECT_FALSE(decodeRawE5(kDf, sizeof kDf, e));
}

TEST(SkyTraqDecode, TheNavigationStateAndChannels)
{
    RcvState s;
    ASSERT_TRUE(decodeRcvState(kDf, sizeof kDf, s));
    EXPECT_EQ(s.nav_state, kNavFix3d);
    EXPECT_EQ(s.week, 2437);
    EXPECT_NEAR(s.tow_s, 569313.2997001292, 1e-9);
    // the fixture's made-up point: 30 N, 40 W, 50 m
    EXPECT_NEAR(s.ecef_m[0], 4234923.449363, 1e-5);
    EXPECT_NEAR(s.ecef_m[1], -3553522.704425, 1e-5);
    EXPECT_NEAR(s.ecef_m[2], 3170398.735384, 1e-5);
    EXPECT_NEAR(s.clk_bias_m, 89899.006085, 1e-5);
    EXPECT_NEAR(s.pdop, 1.04f, 1e-3);
    ChannelSummary ch;
    ASSERT_TRUE(decodeSvChStatus(kE7, sizeof kE7, ch));
    EXPECT_EQ(ch.channels, 37);
    EXPECT_EQ(ch.used, 18);
    EXPECT_EQ(ch.tracked, 19);
    bool acked = false;
    const uint8_t ack[] = {MSG_ACK, 0x64, 0x17};
    EXPECT_TRUE(isAckOf(ack, sizeof ack, 0x64, 0x17, acked));
    EXPECT_TRUE(acked);
    EXPECT_FALSE(isAckOf(ack, sizeof ack, 0x64, 0x18, acked));
    const uint8_t nack[] = {MSG_NACK, 0x1E};
    EXPECT_TRUE(isAckOf(nack, sizeof nack, 0x1E, -1, acked));
    EXPECT_FALSE(acked);
}

TEST(SkyTraqDecode, LockTimeAndDopplerStep)
{
    EXPECT_EQ(lockTimeMinMs(0), 0u);
    EXPECT_EQ(lockTimeMinMs(1), 100u);
    EXPECT_EQ(lockTimeMinMs(11), 102400u);
    EXPECT_EQ(lockTimeMinMs(13), 409600u);   // the 09-26 capture's indicator-13 locks reached 819.2 s
    EXPECT_DOUBLE_EQ(dopplerHz(2048.0f), 2048.5);
    EXPECT_DOUBLE_EQ(dopplerHz(-1596.0f), -1596.5);
    EXPECT_DOUBLE_EQ(dopplerHz(0.0f), 0.0);
    EXPECT_DOUBLE_EQ(dopplerHz(12.25f), 12.25);   // a fractional value is left alone
}

TEST(Px1105rRawEpoch, OneEpochAsTheFiltersCells)
{
    RawE5 e;
    ASSERT_TRUE(decodeRawE5(kE5, sizeof kE5, e));
    px1105r::Px1105rLockState lk;
    gnss_raw::RawEpoch out;
    const int n = px1105r::rawEpochFromE5(e, lk, out);
    EXPECT_EQ(n, 18);   // the L1-band GPS, Galileo and BeiDou MEO/IGSO signals of the 35
    EXPECT_DOUBLE_EQ(out.tow_s, 569313.3);
    int sys_n[GNSS_SYS_COUNT] = {};
    for (int i = 0; i < out.n; ++i)
    {
        const gnss_raw::RawObs& o = out.obs[i];
        sys_n[o.sys]++;
        if (o.sys == GNSS_SYS_BDS)
            EXPECT_TRUE(o.prn >= 6 && o.prn <= 58);
        EXPECT_TRUE(o.has_pr);
        EXPECT_GT(o.pr_m, 1.9e7);
        EXPECT_LT(o.pr_m, 4.0e7);
    }
    EXPECT_GT(sys_n[GNSS_SYS_GPS], 0);
    EXPECT_GT(sys_n[GNSS_SYS_GAL], 0);
    EXPECT_GT(sys_n[GNSS_SYS_BDS], 0);
    // G23: rate = -lambda * (2308 + 0.5), lock indicator 12 = 204.8 s
    const gnss_raw::RawObs& g = out.obs[0];
    EXPECT_EQ(g.sys, GNSS_SYS_GPS);
    EXPECT_EQ(g.prn, 23);
    EXPECT_NEAR(g.rate_mps, -(299792458.0 / 1575.42e6) * 2308.5, 1e-9);
    EXPECT_EQ(g.lock_ms, 204800u);
    EXPECT_TRUE(g.has_phase);
    EXPECT_NEAR(g.phase_m, (299792458.0 / 1575.42e6) * -1554753.721093995, 1e-6);
    // a BeiDou cell takes B1I's wavelength
    for (int i = 0; i < out.n; ++i)
        if (out.obs[i].sys == GNSS_SYS_BDS && out.obs[i].prn == 11)
            EXPECT_NEAR(out.obs[i].rate_mps, -(299792458.0 / 1561.098e6) * 1424.5, 1e-9);
}

TEST(Px1105rRawEpoch, ASlipHoldsTheLockFromThen)
{
    RawE5 e;
    ASSERT_TRUE(decodeRawE5(kE5, sizeof kE5, e));
    px1105r::Px1105rLockState lk;
    gnss_raw::RawEpoch out;
    e.cell[0].ind |= kIndSlip;
    px1105r::rawEpochFromE5(e, lk, out);
    EXPECT_EQ(out.obs[0].lock_ms, 0u);
    EXPECT_FALSE(out.obs[0].has_phase);
    EXPECT_EQ(lk.slips, 1u);
    // 2 s later, the flag gone, the indicator still 12: the lock counts from the slip
    e.cell[0].ind &= static_cast<uint16_t>(~kIndSlip);
    e.tow_ms += 2000;
    px1105r::rawEpochFromE5(e, lk, out);
    EXPECT_EQ(out.obs[0].lock_ms, 2000u);
    EXPECT_TRUE(out.obs[0].has_phase);
    // a clock jump 500 ms ago holds every lock to it
    lk.have_cut = true;
    lk.cut_ms = px1105r::rxMs(e.week, e.tow_ms) - 500;
    px1105r::rawEpochFromE5(e, lk, out);
    for (int i = 0; i < out.n; ++i)
        EXPECT_LE(out.obs[i].lock_ms, 500u);
}

TEST(Px1105rRawEpoch, TheEpochsGpsInstantFromTheStateBefore)
{
    // the 0xDF before the 0xE5 (its own epoch 50 ms earlier), carried by its
    // drift, places the 0xE5's GPS instant where the 0xDF after it says it is
    RcvState prev, own;
    ASSERT_TRUE(decodeRcvState(kDfPrev, sizeof kDfPrev, prev));
    ASSERT_TRUE(decodeRcvState(kDf, sizeof kDf, own));
    px1105r::Px1105rClock clk;
    EXPECT_FALSE(px1105r::noteRcvState(prev, clk));
    EXPECT_EQ(clk.rx_ms, px1105r::rxMs(2437, 569313250));
    RawE5 e;
    ASSERT_TRUE(decodeRawE5(kE5, sizeof kE5, e));
    double dtr = 0.0;
    ASSERT_TRUE(px1105r::biasAt(clk, px1105r::rxMs(e.week, e.tow_ms), dtr));
    EXPECT_NEAR(dtr, 89899.006085 / 299792458.0 * 1e3, 1e-7);   // ms, within 0.1 ns of the next 0xDF's
    const double gps = px1105r::epochGpsS(e.week, e.tow_ms, dtr);
    EXPECT_NEAR(gps, own.week * 604800.0 + own.tow_s, 5e-7);   // a double at 1.5e9 s: 0.24 us
    // the next 0xDF: no jump
    EXPECT_FALSE(px1105r::noteRcvState(own, clk));
    // a whole millisecond step in the bias is a jump
    RcvState stepped = own;
    stepped.tow_s += 0.05;
    stepped.clk_bias_m = own.clk_bias_m + own.clk_drift_mps * 0.05 - 299792.458;
    EXPECT_TRUE(px1105r::noteRcvState(stepped, clk));
    EXPECT_EQ(clk.jumps, 1u);
    // the fix lost: its bias carried by the drift while the 0xDF keep coming
    // (a reacquiring receiver's first 0xE5 come before its fix); an epoch
    // more than kStateGapMs past the last 0xDF gets nothing
    RcvState nofix = stepped;
    nofix.nav_state = kNavNoFix;
    nofix.clk_bias_m = 0.0;   // not GPS-referenced: never taken
    nofix.tow_s += 0.05;
    const int64_t fix_rx = clk.fix_rx_ms;
    const double fix_bias = clk.bias_ms;
    EXPECT_FALSE(px1105r::noteRcvState(nofix, clk));
    EXPECT_FALSE(clk.fix);
    ASSERT_TRUE(px1105r::biasAt(clk, fix_rx + 2000, dtr));
    EXPECT_NEAR(dtr, fix_bias + clk.drift_ms_per_s * 2.0, 1e-12);
    EXPECT_FALSE(px1105r::biasAt(clk, clk.rx_ms + px1105r::kStateGapMs + 50, dtr));
    // a fix back after more than kFixGapMs: a cut (the jump test cannot tell)
    RcvState back = stepped;
    back.tow_s += 2.5;
    back.clk_bias_m = stepped.clk_bias_m + stepped.clk_drift_mps * 2.5;
    EXPECT_TRUE(px1105r::noteRcvState(back, clk));
    EXPECT_EQ(clk.restarts, 1u);
}

TEST(Px1105rRawEpoch, TheBiasIsCarriedThroughAFixBlock)
{
    // a receiver past its speed limit: no fix for 276 s, its 0xDF still
    // coming at 20 Hz with their bias and drift held (Traveler IV on the rig,
    // 2026-10-23); the last fix's bias carried by its drift the whole way
    RcvState fix;
    ASSERT_TRUE(decodeRcvState(kDf, sizeof kDf, fix));
    px1105r::Px1105rClock clk;
    EXPECT_FALSE(px1105r::noteRcvState(fix, clk));
    const int64_t fix_rx = clk.fix_rx_ms;
    const double fix_bias = clk.bias_ms;
    RcvState nofix = fix;
    nofix.nav_state = kNavNoFix;
    for (int i = 1; i <= 5520; ++i)   // 276 s
    {
        nofix.tow_s = fix.tow_s + 0.05 * i;
        EXPECT_FALSE(px1105r::noteRcvState(nofix, clk));
    }
    double dtr = 0.0;
    ASSERT_TRUE(px1105r::biasAt(clk, clk.rx_ms + 50, dtr));
    EXPECT_NEAR(dtr, fix_bias + clk.drift_ms_per_s * (clk.rx_ms + 50 - fix_rx) * 1e-3, 1e-9);
    EXPECT_EQ(clk.breaks, 0u);
    // but not past kBiasCarryMaxMs, the 0xDF still coming (one a second here)
    for (int s = 277; s * 1000 <= px1105r::kBiasCarryMaxMs + 1000; ++s)
    {
        nofix.tow_s = fix.tow_s + s;
        px1105r::noteRcvState(nofix, clk);
    }
    EXPECT_TRUE(clk.carry);
    EXPECT_FALSE(px1105r::biasAt(clk, clk.rx_ms + 50, dtr));
}

TEST(Px1105rRawEpoch, ABreakInTheStateStreamEndsTheCarry)
{
    // a receiver restarting without a fix (one Traveler flight on the rig:
    // 15.5 s without an 0xDF, then its time going back): nothing carried from
    // there until its next fix, however steadily the 0xDF come again
    RcvState fix;
    ASSERT_TRUE(decodeRcvState(kDf, sizeof kDf, fix));
    for (double step : {2.5, -0.05})   // a gap, then time going back
    {
        px1105r::Px1105rClock clk;
        EXPECT_FALSE(px1105r::noteRcvState(fix, clk));
        RcvState nofix = fix;
        nofix.nav_state = kNavNoFix;
        nofix.tow_s += 0.05;
        px1105r::noteRcvState(nofix, clk);
        nofix.tow_s += step;
        px1105r::noteRcvState(nofix, clk);
        EXPECT_FALSE(clk.carry);
        EXPECT_EQ(clk.breaks, 1u);
        for (int i = 0; i < 20; ++i)
        {
            nofix.tow_s += 0.05;
            px1105r::noteRcvState(nofix, clk);
        }
        double dtr = 0.0;
        EXPECT_FALSE(px1105r::biasAt(clk, clk.rx_ms + 50, dtr));
        // its next fix carries again
        RcvState back = fix;
        back.tow_s = nofix.tow_s + 0.05;
        back.clk_bias_m = fix.clk_bias_m + fix.clk_drift_mps * (back.tow_s - fix.tow_s);
        px1105r::noteRcvState(back, clk);
        EXPECT_TRUE(px1105r::biasAt(clk, clk.rx_ms + 50, dtr));
    }
}

TEST(Px1105rRawEpoch, TheStateBeforeAnyTimeIsNotNoted)
{
    // the receiver's all-zero 0xDF before its first fix (week 0): ~400 a
    // power-up on the RF rig, none a restart
    px1105r::Px1105rClock clk;
    RcvState zero;
    zero.week = 0;
    for (int i = 0; i < 400; ++i)
        EXPECT_FALSE(px1105r::noteRcvState(zero, clk));
    EXPECT_FALSE(clk.have);
    EXPECT_EQ(clk.restarts, 0u);
    double dtr;
    EXPECT_FALSE(px1105r::biasAt(clk, 1000, dtr));
}

// The whole capture, when it is given: every frame parses, every 0xE5 adapts,
// and the bias carried from each 0xDF places the next 0xE5 where its own 0xDF
// says (the time tag's whole arithmetic, over 20 minutes and every clock change).
TEST(SkyTraqCapture, TwentyMinutesOfOpenSky)
{
    const char* path = std::getenv("PX1105R_CAPTURE");
    if (path == nullptr)
        GTEST_SKIP() << "PX1105R_CAPTURE not set";
    const std::string cmd = std::string("zcat < '") + path + "'";
    FILE* f = popen(cmd.c_str(), "r");
    ASSERT_NE(f, nullptr);
    std::vector<char> line(1 << 16);
    std::vector<uint8_t> stream;
    while (std::fgets(line.data(), static_cast<int>(line.size()), f))
    {
        const char* b = std::strstr(line.data(), " B ");
        if (b == nullptr)
            continue;
        std::vector<uint8_t> body;
        for (const char* h = b + 3; h[0] && h[1] && h[0] != '\n'; h += 2)
        {
            unsigned v;
            if (std::sscanf(h, "%2x", &v) != 1)
                break;
            body.push_back(static_cast<uint8_t>(v));
        }
        const std::vector<uint8_t> fr = framed(body.data(), body.size());
        stream.insert(stream.end(), fr.begin(), fr.end());
    }
    pclose(f);
    FrameParser p;
    Collect c;
    p.feed(stream.data(), stream.size(), c);
    EXPECT_EQ(p.badChecksums(), 0u);
    int n_e5 = 0, n_df = 0, cells = 0, checked = 0;
    double worst_us = 0.0;
    px1105r::Px1105rClock clk;
    px1105r::Px1105rLockState lk;
    gnss_raw::RawEpoch out;
    double pending_gps = -1.0;   // the last 0xE5's GPS instant, to be met by its own 0xDF
    RawE5 e;
    for (const auto& b : c.bodies)
    {
        if (b[0] == MSG_EXT_RAW && decodeRawE5(b.data(), b.size(), e))
        {
            n_e5++;
            cells += px1105r::rawEpochFromE5(e, lk, out);
            double dtr;
            pending_gps = px1105r::biasAt(clk, px1105r::rxMs(e.week, e.tow_ms), dtr)
                              ? px1105r::epochGpsS(e.week, e.tow_ms, dtr) : -1.0;
        }
        else if (b[0] == MSG_RCV_STATE)
        {
            RcvState s;
            ASSERT_TRUE(decodeRcvState(b.data(), b.size(), s));
            n_df++;
            if (pending_gps > 0.0)
            {
                const double err_us = std::fabs(pending_gps - (s.week * 604800.0 + s.tow_s)) * 1e6;
                worst_us = std::max(worst_us, err_us);
                checked++;
            }
            pending_gps = -1.0;
            px1105r::noteRcvState(s, clk);
        }
    }
    EXPECT_EQ(n_e5, 24000);
    EXPECT_EQ(n_df, 24000);
    EXPECT_GT(checked, 23900);
    EXPECT_LT(worst_us, 0.5);
    EXPECT_EQ(clk.jumps, 0u);
    EXPECT_EQ(clk.restarts, 0u);
    EXPECT_GT(cells / n_e5, 15);
    std::printf("capture: %d epochs, %.1f cells each, epoch instants within %.3f us of their 0xDF, %u slips\n",
                n_e5, static_cast<double>(cells) / n_e5, worst_us, lk.slips);
}
