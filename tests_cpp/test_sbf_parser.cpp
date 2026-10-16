// Host tests for the mosaic-G5's stream parser and block decoders
// (TR_GNSSReceiverMosaic_Serial/SbfParser.cpp): SBF framing and its CRC, the
// command replies and prompts in between, recovery from a corrupt block or a
// false sync, and PVTGeodetic -> GNSSData. Blocks are built here from their
// fields (mosaic-G5 Reference Guide, firmware 1.0.1, section 4) with made-up
// positions; the block sizes match the receiver's (PVTGeodetic rev 2 = 96 B).
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "SbfParser.h"

using namespace mosaic;

namespace {

struct Bytes
{
    std::vector<uint8_t> v;
    Bytes& u8(uint8_t x) { v.push_back(x); return *this; }
    Bytes& u16(uint16_t x) { u8(x & 0xFF); return u8(x >> 8); }
    Bytes& u32(uint32_t x) { u16(x & 0xFFFF); return u16(x >> 16); }
    Bytes& f32(float f) { uint32_t x; std::memcpy(&x, &f, 4); return u32(x); }
    Bytes& f64(double d) { uint64_t x; std::memcpy(&x, &d, 8); u32(static_cast<uint32_t>(x)); return u32(static_cast<uint32_t>(x >> 32)); }
};

// A whole block: header, time stamp, body, padding to 4, CRC.
std::vector<uint8_t> block(uint16_t number, uint8_t rev, uint32_t tow, uint16_t wnc, const std::vector<uint8_t>& body)
{
    Bytes b;
    b.u8('$').u8('@').u16(0).u16(static_cast<uint16_t>(number | (rev << 13))).u16(0).u32(tow).u16(wnc);
    b.v.insert(b.v.end(), body.begin(), body.end());
    while (b.v.size() % 4) b.v.push_back(0);
    const uint16_t len = static_cast<uint16_t>(b.v.size());
    b.v[6] = len & 0xFF;
    b.v[7] = len >> 8;
    const uint16_t crc = crc16Ccitt(b.v.data() + 4, b.v.size() - 4);
    b.v[2] = crc & 0xFF;
    b.v[3] = crc >> 8;
    return b.v;
}

struct PvtFields
{
    uint8_t mode = 1, error = 0;
    double lat = 0.5, lon = -1.2, height = 1234.567;
    float und = -33.5f, vn = 1.25f, ve = -2.5f, vu = 100.75f;
    uint8_t nr_sv = 27;
    uint16_t latency = 123, hacc = 123, vacc = 345;
};

std::vector<uint8_t> pvtBlock(uint32_t tow, uint16_t wnc, const PvtFields& f = PvtFields())
{
    Bytes b;
    b.u8(f.mode).u8(f.error).f64(f.lat).f64(f.lon).f64(f.height).f32(f.und)
     .f32(f.vn).f32(f.ve).f32(f.vu).f32(45.0f)         // COG
     .f64(0.000123).f32(-0.5f)                        // RxClkBias ms, RxClkDrift ppm
     .u8(0).u8(0).u8(f.nr_sv).u8(0)                   // TimeSystem, Datum, NrSV, WACorrInfo
     .u16(65535).u16(65535).u32(0x00F00011)           // ReferenceID, MeanCorrAge, SignalInfo
     .u8(0).u8(0).u16(0)                              // AlertFlag, NrBases, PPPInfo
     .u16(f.latency).u16(f.hacc).u16(f.vacc).u8(0);   // Latency, HAccuracy, VAccuracy, Misc
    return block(SBF_PVT_GEODETIC, 2, tow, wnc, b.v);
}

std::vector<uint8_t> receiverTimeBlock(uint32_t tow, uint16_t wnc, int8_t delta_ls, uint8_t sync)
{
    Bytes b;
    b.u8(26).u8(10).u8(24).u8(15).u8(30).u8(0).u8(static_cast<uint8_t>(delta_ls)).u8(sync);
    return block(SBF_RECEIVER_TIME, 0, tow, wnc, b.v);
}

std::vector<uint8_t> asBytes(const std::string& s) { return std::vector<uint8_t>(s.begin(), s.end()); }

void append(std::vector<uint8_t>& to, const std::vector<uint8_t>& from) { to.insert(to.end(), from.begin(), from.end()); }

// What came out, in order, as text: "B:<len>", "L:<line>", "P:<port>".
struct Recorder : SbfSink
{
    std::vector<std::string> items;
    std::vector<std::vector<uint8_t>> blocks;
    std::vector<size_t> block_index;
    void onBlock(const uint8_t* b, size_t len, size_t index) override
    {
        items.push_back("B:" + std::to_string(len));
        blocks.emplace_back(b, b + len);
        block_index.push_back(index);
    }
    void onLine(const char* line, size_t) override { items.push_back(std::string("L:") + line); }
    void onPrompt(const char* port, size_t) override { items.push_back(std::string("P:") + port); }
};

std::vector<std::string> feedAll(const std::vector<uint8_t>& data, SbfParser& p)
{
    Recorder r;
    p.feed(data.data(), data.size(), r);
    return r.items;
}

}  // namespace

TEST(SbfCrc, IsCrcCcittXmodem)
{
    // CRC-16/XMODEM's check value: polynomial 0x1021, seed 0, no reflection.
    const std::string s = "123456789";
    EXPECT_EQ(crc16Ccitt(reinterpret_cast<const uint8_t*>(s.data()), s.size()), 0x31C3);
}

TEST(SbfParser, SplitsRepliesPromptsAndBlocksInOrder)
{
    std::vector<uint8_t> s = asBytes("$R: sso, Stream7, COM1, MeasEpoch, msec50\r\n"
                                     "  SBFOutput, Stream7, COM1, MeasEpoch, msec50\r\nCOM1>");
    const auto pvt = pvtBlock(574218250, 2441);
    const auto rt = receiverTimeBlock(574218000, 2441, 18, 7);
    append(s, pvt);
    append(s, rt);
    append(s, asBytes("$R? sfn, DSK1, Bogus: Invalid argument!\r\nCOM1>"));

    SbfParser p;
    const auto items = feedAll(s, p);
    const std::vector<std::string> want = {
        "L:$R: sso, Stream7, COM1, MeasEpoch, msec50",
        "L:  SBFOutput, Stream7, COM1, MeasEpoch, msec50",
        "P:COM1",
        "B:96",
        "B:24",
        "L:$R? sfn, DSK1, Bogus: Invalid argument!",
        "P:COM1",
    };
    EXPECT_EQ(items, want);
    EXPECT_EQ(p.blocks(), 2u);
    EXPECT_EQ(p.crcFails(), 0u);
}

TEST(SbfParser, AnySlicingGivesTheSameItems)
{
    std::vector<uint8_t> s;
    for (int k = 0; k < 5; k++)
    {
        append(s, pvtBlock(574218000 + 50 * k, 2441));
        append(s, asBytes("COM1>"));
        append(s, receiverTimeBlock(574218000, 2441, 18, 7));
        append(s, asBytes("$R: esoc, COM1, GPSNav\r\n  SBFOnce, COM1, GPSNav\r\nCOM1>"));
    }
    SbfParser whole;
    const auto want = feedAll(s, whole);
    ASSERT_EQ(want.size(), 30u);

    std::mt19937 rng(1234);
    for (int trial = 0; trial < 50; trial++)
    {
        SbfParser p;
        Recorder r;
        size_t at = 0;
        while (at < s.size())
        {
            const size_t n = std::min<size_t>(1 + rng() % 97, s.size() - at);
            p.feed(s.data() + at, n, r);
            at += n;
        }
        ASSERT_EQ(r.items, want) << "trial " << trial;
    }
}

TEST(SbfParser, BlockIndexIsItsLastByteInTheSlice)
{
    std::vector<uint8_t> s = asBytes("noise");
    const auto pvt = pvtBlock(1000, 2441);
    append(s, pvt);
    append(s, asBytes("xy"));
    SbfParser p;
    Recorder r;
    p.feed(s.data(), s.size(), r);
    ASSERT_EQ(r.block_index.size(), 1u);
    EXPECT_EQ(r.block_index[0], 5 + pvt.size() - 1);
    EXPECT_EQ(r.blocks[0], pvt);
}

TEST(SbfParser, ACorruptBlockCostsOnlyItself)
{
    auto bad = pvtBlock(1000, 2441);
    bad[40] ^= 0x01;   // one bit in the latitude
    const auto good = receiverTimeBlock(1000, 2441, 18, 7);
    std::vector<uint8_t> s = bad;
    append(s, good);
    append(s, asBytes("COM1>"));

    // Its bytes get a second look, as text: they may make a junk line, never a block.
    SbfParser p;
    Recorder r;
    p.feed(s.data(), s.size(), r);
    ASSERT_EQ(r.blocks.size(), 1u);
    EXPECT_EQ(r.blocks[0], good);
    EXPECT_EQ(r.items.back(), "P:COM1");
    EXPECT_EQ(p.crcFails(), 1u);
}

TEST(SbfParser, AFalseSyncGivesBackWhatItSwallowed)
{
    // "$@" with a plausible Length (64) that is not a block: the real block
    // right behind it is inside those 64 bytes and must still come out.
    std::vector<uint8_t> s = {'$', '@', 0x11, 0x22, 0x33, 0x44, 64, 0};
    const auto rt = receiverTimeBlock(5000, 2441, 18, 7);
    append(s, rt);
    const auto pvt = pvtBlock(5000, 2441);
    append(s, pvt);

    SbfParser p;
    Recorder r;
    p.feed(s.data(), s.size(), r);
    ASSERT_EQ(r.blocks.size(), 2u);
    EXPECT_EQ(r.blocks[0], rt);
    EXPECT_EQ(r.blocks[1], pvt);
    EXPECT_EQ(p.crcFails(), 1u);
}

TEST(SbfParser, AnImpossibleLengthIsAFalseSync)
{
    std::vector<uint8_t> s = {'$', '@', 0, 0, 0, 0, 7, 0};   // not a multiple of 4
    const auto rt = receiverTimeBlock(5000, 2441, 18, 7);
    append(s, rt);
    std::vector<uint8_t> t = {'$', '@', 0, 0, 0, 0, 0x00, 0x20};   // 8192: longer than kept
    append(s, t);
    append(s, rt);

    SbfParser p;
    Recorder r;
    p.feed(s.data(), s.size(), r);
    EXPECT_EQ(r.blocks.size(), 2u);
    EXPECT_EQ(p.badLengths(), 2u);
}

TEST(SbfParser, PromptsAreWholeLinesOfTheirOwnShape)
{
    SbfParser p;
    const auto items = feedAll(asBytes("USB1>IP10>STOP>---->\r\nCOM>\r\nCOM123>\r\ncom1>\r\n"), p);
    EXPECT_EQ(items, (std::vector<std::string>{"P:USB1", "P:IP10", "P:STOP", "L:---->", "L:COM>",
                                               "L:COM123>", "L:com1>"}));
}

TEST(SbfParser, ALongLineIsCutAndCounted)
{
    SbfParser p;
    const std::string long_line(200, 'x');
    const auto items = feedAll(asBytes(long_line + "\r\nCOM1>"), p);
    ASSERT_EQ(items.size(), 2u);
    EXPECT_EQ(items[0], "L:" + std::string(SbfParser::kMaxLine, 'x'));
    EXPECT_EQ(items[1], "P:COM1");
    EXPECT_EQ(p.longLines(), 1u);
}

TEST(SbfParser, ResetDropsAPartialBlock)
{
    const auto pvt = pvtBlock(1000, 2441);
    SbfParser p;
    Recorder r;
    p.feed(pvt.data(), 40, r);
    p.reset();
    p.feed(pvt.data(), pvt.size(), r);
    EXPECT_EQ(r.blocks.size(), 1u);
}

// ── Decoders ────────────────────────────────────────────────────────────

TEST(SbfDecode, PvtGeodeticFieldsAndHeader)
{
    const auto b = pvtBlock(574218250, 2441);
    ASSERT_EQ(b.size(), 96u);
    SbfHeader h;
    ASSERT_TRUE(sbfHeader(b.data(), b.size(), h));
    EXPECT_EQ(h.number, SBF_PVT_GEODETIC);
    EXPECT_EQ(h.revision, 2);
    EXPECT_EQ(h.tow_ms, 574218250u);
    EXPECT_EQ(h.wnc, 2441);

    PvtGeodetic p;
    ASSERT_TRUE(decodePvtGeodetic(b.data(), b.size(), p));
    EXPECT_TRUE(p.has_solution);
    EXPECT_DOUBLE_EQ(p.lat_rad, 0.5);
    EXPECT_DOUBLE_EQ(p.lon_rad, -1.2);
    EXPECT_DOUBLE_EQ(p.height_m, 1234.567);
    EXPECT_FLOAT_EQ(p.undulation_m, -33.5f);
    EXPECT_FLOAT_EQ(p.vu, 100.75f);
    EXPECT_EQ(p.nr_sv, 27);
    EXPECT_EQ(p.latency_100us, 123);
    EXPECT_EQ(p.h_acc_cm, 123);
    EXPECT_EQ(p.v_acc_cm, 345);
    ReceiverTime not_it;
    EXPECT_FALSE(decodeReceiverTime(b.data(), b.size(), not_it));   // not that block
}

TEST(SbfDecode, GnssDataLikeTheUbloxDriver)
{
    PvtGeodetic p;
    const auto b = pvtBlock(574218250, 2441);
    ASSERT_TRUE(decodePvtGeodetic(b.data(), b.size(), p));
    GNSSData d;
    gnssDataFromPvt(p, 18, 157, d);
    EXPECT_EQ(d.time_us, 0u);
    EXPECT_EQ(d.year, 2026);
    EXPECT_EQ(d.month, 10);
    EXPECT_EQ(d.day, 24);
    EXPECT_EQ(d.hour, 15);
    EXPECT_EQ(d.minute, 30);
    EXPECT_EQ(d.second, 0);
    EXPECT_EQ(d.milli_second, 250);
    EXPECT_EQ(d.fix_mode, 3);
    EXPECT_EQ(d.num_sats, 27);
    EXPECT_EQ(d.pdop_x10, 16);
    EXPECT_EQ(d.lat_e7, static_cast<int32_t>(std::lround(0.5 * 180.0 / M_PI * 1e7)));
    EXPECT_EQ(d.lon_e7, static_cast<int32_t>(std::lround(-1.2 * 180.0 / M_PI * 1e7)));
    EXPECT_EQ(d.alt_mm, 1268067);          // above the geoid: height less undulation
    EXPECT_EQ(d.vel_n_mmps, 1250);
    EXPECT_EQ(d.vel_e_mmps, -2500);
    EXPECT_EQ(d.vel_u_mmps, 100750);
    // 95 % figures halved to one sigma, whole meters: 1.23 m -> 0.6 -> 1; 3.45 m -> 1.7 -> 2
    EXPECT_EQ(d.h_acc_m, 1);
    EXPECT_EQ(d.v_acc_m, 2);
}

TEST(SbfDecode, TwoDAndNoSolution)
{
    PvtFields f;
    f.mode = 0x81;
    PvtGeodetic p;
    auto b = pvtBlock(1000, 2441, f);
    ASSERT_TRUE(decodePvtGeodetic(b.data(), b.size(), p));
    GNSSData d;
    gnssDataFromPvt(p, 18, 0, d);
    EXPECT_EQ(d.fix_mode, 2);

    // No solution: the receiver says why and fills the rest with do-not-use.
    PvtFields n;
    n.mode = 0;
    n.error = 1;
    n.lat = n.lon = n.height = -2e10;
    n.und = n.vn = n.ve = n.vu = -2e10f;
    n.nr_sv = 255;
    n.hacc = n.vacc = 65535;
    b = pvtBlock(1000, 2441, n);
    ASSERT_TRUE(decodePvtGeodetic(b.data(), b.size(), p));
    EXPECT_FALSE(p.has_solution);
    gnssDataFromPvt(p, 18, 0, d);
    EXPECT_EQ(d.fix_mode, 0);
    EXPECT_EQ(d.num_sats, 0);
    EXPECT_EQ(d.lat_e7, 0);
    EXPECT_EQ(d.alt_mm, 0);
    EXPECT_EQ(d.h_acc_m, 255);
    EXPECT_EQ(d.v_acc_m, 255);
    EXPECT_EQ(d.year, 2026);   // the receiver's time is still known
}

TEST(SbfDecode, ExportLimitIsNoFix)
{
    // Error 7: position output prohibited by export laws. The u-blox driver
    // zeroes fix_mode for its own COCOM flag the same way (#562).
    PvtFields f;
    f.error = 7;
    PvtGeodetic p;
    const auto b = pvtBlock(1000, 2441, f);
    ASSERT_TRUE(decodePvtGeodetic(b.data(), b.size(), p));
    GNSSData d;
    gnssDataFromPvt(p, 18, 100, d);
    EXPECT_EQ(d.fix_mode, 0);
}

TEST(SbfDecode, StatusTimeDopDiskAndPps)
{
    ReceiverTime t;
    auto b = receiverTimeBlock(1000, 2441, 18, 7);
    ASSERT_TRUE(decodeReceiverTime(b.data(), b.size(), t));
    EXPECT_EQ(t.delta_ls, 18);
    EXPECT_EQ(t.sync_level, 7);

    Bytes dop;
    dop.u8(25).u8(0).u16(157).u16(90).u16(80).u16(130).f32(5.0f).f32(7.0f);
    b = block(SBF_DOP, 0, 1000, 2441, dop.v);
    Dop d;
    ASSERT_TRUE(decodeDop(b.data(), b.size(), d));
    EXPECT_EQ(d.nr_sv, 25);
    EXPECT_EQ(d.pdop_x100, 157);

    Bytes rs;
    rs.u8(45).u8(0).u32(3600).u32(0x2).u32(0).u8(0).u8(4).u8(9).u8(152);   // 52 C
    b = block(SBF_RECEIVER_STATUS, 1, 1000, 2441, rs.v);
    ReceiverStatus s;
    ASSERT_TRUE(decodeReceiverStatus(b.data(), b.size(), s));
    EXPECT_EQ(s.cpu_load, 45);
    EXPECT_EQ(s.uptime_s, 3600u);
    EXPECT_EQ(s.temp_c, 52);

    Bytes ds;
    ds.u8(1).u8(16).u32(0)                                   // N, SBLength, Reserved
      .u8(1).u8(0x09).u16(0).u32(123456789).u32(7600).u8(3).u8(0).u16(0);   // DiskID 1, mounted + logging
    b = block(SBF_DISK_STATUS, 1, 1000, 2441, ds.v);
    DiskStatus disk;
    ASSERT_TRUE(decodeDiskStatus(b.data(), b.size(), disk));
    EXPECT_TRUE(disk.present);
    EXPECT_EQ(disk.status, 0x09);
    EXPECT_EQ(disk.used_bytes, 123456789u);
    EXPECT_EQ(disk.size_mb, 7600u);
    EXPECT_EQ(disk.error, 0);

    Bytes px;
    px.u8(0).u8(1).f32(-1.5f);
    b = block(SBF_XPPS_OFFSET, 0, 574218000, 2441, px.v);
    XppsOffset x;
    ASSERT_TRUE(decodeXppsOffset(b.data(), b.size(), x));
    EXPECT_EQ(x.time_scale, 1);
    EXPECT_EQ(x.tow_ms, 574218000u);
    EXPECT_FLOAT_EQ(x.offset_ns, -1.5f);
}

TEST(SbfDecode, GpsToUtcAcrossTheYear)
{
    UtcTime u;
    gpsToUtc(2451, 432017999, 18, u);   // 2026-12-31 23:59:59.999 UTC
    EXPECT_EQ(u.year, 2026);
    EXPECT_EQ(u.month, 12);
    EXPECT_EQ(u.day, 31);
    EXPECT_EQ(u.hour, 23);
    EXPECT_EQ(u.minute, 59);
    EXPECT_EQ(u.second, 59);
    EXPECT_EQ(u.millisecond, 999);
    gpsToUtc(2451, 432018000, 18, u);   // one ms later: 2027-01-01 00:00:00.000
    EXPECT_EQ(u.year, 2027);
    EXPECT_EQ(u.month, 1);
    EXPECT_EQ(u.day, 1);
    EXPECT_EQ(u.hour, 0);
    EXPECT_EQ(u.second, 0);
    EXPECT_EQ(u.millisecond, 0);
    gpsToUtc(0, 0, 0, u);               // the GPS epoch
    EXPECT_EQ(u.year, 1980);
    EXPECT_EQ(u.month, 1);
    EXPECT_EQ(u.day, 6);
}

TEST(SbfDecode, UnknownLeapSecondsFallBackTo18)
{
    PvtGeodetic p;
    const auto b = pvtBlock(574218250, 2441);
    ASSERT_TRUE(decodePvtGeodetic(b.data(), b.size(), p));
    GNSSData d;
    gnssDataFromPvt(p, -128, 0, d);
    EXPECT_EQ(d.hour, 15);
    EXPECT_EQ(d.minute, 30);
    EXPECT_EQ(d.second, 0);
}
