// The telemetry JSON packer (TR_BLE_To_APP/TelemetryJsonPacker.h, #282 / #283):
// fields at the width they really print, the frame never past the link's
// payload window, "tr":1 whenever anything was left out, and the Tier 1 floor.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include "TelemetryJsonPacker.h"

using tr_ble::TelemetryJsonPacker;

namespace {

bool has(const std::string& s, const char* frag) { return s.find(frag) != std::string::npos; }

// The V10's direct-link frame outside with a fix, field for field as the OC
// builds it (TR_BLE_To_APP::buildTelemetryJSON, Tier 1 to Tier 3): the frame
// captured over BLE on 2026-11-22 (board 2 on 8b2e11f5, 435 B with no fix yet)
// with the position, the accuracy and a live attitude that a fix adds, and
// (#1755) the AUTO mounting's pad confirmation a later image adds beside "h".
void packV10WithFix(TelemetryJsonPacker& pk)
{
    pk.addString("st", "READY");
    pk.addInt("fs", 16);
    pk.addInt("h", 1091445);
    pk.addUint("mo", 1);
    pk.addDouble("lat", 34.12345, 5);
    pk.addDouble("lon", -117.12345, 5);
    pk.addInt("nsat", 15);
    pk.addInt("hacc", 2);
    pk.addFloat("palt", 0.3f, 1);
    pk.addFloat("arate", 0.0f, 1);
    pk.addFloat("galt", 412.6f, 1);
    pk.addFloat("ve", -0.0f, 1);
    pk.addFloat("vn", 0.1f, 1);
    pk.addFloat("vu", -0.1f, 1);
    pk.addFloat("malt", 0.0f, 1);
    pk.addFloat("mspd", 0.0f, 1);
    pk.addFloat("soc", 79.3f, 1);
    pk.addFloat("cur", -285.0f, 1);
    pk.addFloat("vol", 7.90f, 2);
    pk.endTier1();
    pk.addFloat("rcmd", 0.0f, 1);
    pk.addFloat("q0", 0.707f, 3);
    pk.addFloat("q1", -0.012f, 3);
    pk.addFloat("q2", 0.004f, 3);
    pk.addFloat("q3", -0.707f, 3);
    pk.addFloat("gx", 0.0f, 1);
    pk.addFloat("gy", 0.0f, 1);
    pk.addFloat("gz", 0.1f, 1);
    pk.addFloat("lx", -0.6f, 1);
    pk.addFloat("ly", -0.4f, 1);
    pk.addFloat("lz", 9.8f, 1);
    pk.addFloat("ccur", 0.0f, 2);
    pk.addFloat("scur", 0.0f, 2);
    pk.addFloat("scap", 2.50f, 2);
    pk.addUint("hu", 2);
    pk.addUint("gi", 1);
    pk.addUint("mst", 1);
    pk.addString("mfw", "8b2e11f5+20261122-1217");
    pk.addFloat("rxk", 0.0f, 1);
    pk.addFloat("wrk", 0.0f, 1);
    pk.addUint("frx", 0);
    pk.addUint("fdr", 0);
    pk.addFloat("hx", -0.5f, 1);
    pk.addFloat("hy", -1.9f, 1);
    pk.addFloat("hz", 10.3f, 1);
    pk.addString("af", "");
}

}  // namespace

// The V10's direct frame with a fix: ~490 B of a 509 B window (MTU 512), all of
// it in (at 24 B a number its last one had ~5 B to spare).
TEST(TelemetryJsonPacker, V10FrameWithFixFitsMtu512Whole)
{
    char buf[640];
    TelemetryJsonPacker pk(buf, sizeof(buf), 512 - 3);
    packV10WithFix(pk);
    const size_t n = pk.finish();
    const std::string s(buf);
    EXPECT_EQ(n, s.size());
    EXPECT_FALSE(pk.dropped());
    EXPECT_FALSE(has(s, "\"tr\""));
    EXPECT_LE(s.size(), 509u);
    EXPECT_TRUE(has(s, "\"h\":1091445,\"mo\":1,"));
    EXPECT_TRUE(has(s, ",\"hx\":-0.5,\"hy\":-1.9,\"hz\":10.3,\"af\":\"\"}"));
    EXPECT_EQ(s.front(), '{');
    EXPECT_EQ(s.back(), '}');
}

// A number is charged what it prints: a window with room for exactly
// ,"hz":10.3 plus the held-back tail takes it (24 B a number would not have).
TEST(TelemetryJsonPacker, NumberFitsAtItsPrintedWidth)
{
    const size_t first = std::strlen("{\"a\":1");
    const size_t field = std::strlen(",\"hz\":10.3");
    char buf[64];
    TelemetryJsonPacker pk(buf, sizeof(buf), first + field + TelemetryJsonPacker::kReserve - 1);
    pk.addInt("a", 1);
    pk.addFloat("hz", 10.3f, 1);
    pk.finish();
    EXPECT_STREQ(buf, "{\"a\":1,\"hz\":10.3}");
    EXPECT_FALSE(pk.dropped());
}

// One byte short of that and it is left out, the frame flagged, and still
// inside the window.
TEST(TelemetryJsonPacker, OneByteShortDropsAndFlags)
{
    const size_t first = std::strlen("{\"a\":1");
    const size_t field = std::strlen(",\"hz\":10.3");
    const size_t window = first + field + TelemetryJsonPacker::kReserve - 2;
    char buf[64];
    TelemetryJsonPacker pk(buf, sizeof(buf), window);
    pk.addInt("a", 1);
    pk.addFloat("hz", 10.3f, 1);
    pk.finish();
    EXPECT_STREQ(buf, "{\"a\":1,\"tr\":1}");
    EXPECT_TRUE(pk.dropped());
    EXPECT_LE(std::strlen(buf), window);
}

// Within Tiers 2 and 3 it is best-fit: a bulky field that does not fit is
// skipped and a smaller one after it still goes in.
TEST(TelemetryJsonPacker, BestFitPastTier1)
{
    char buf[64];
    TelemetryJsonPacker pk(buf, sizeof(buf), 30);
    pk.addString("st", "READY");
    pk.endTier1();
    pk.addString("mfw", "8b2e11f5+20261122-1217");
    pk.addInt("n", 7);
    pk.finish();
    EXPECT_STREQ(buf, "{\"st\":\"READY\",\"n\":7,\"tr\":1}");
}

// The Tier 1 floor: once anything in Tier 1 was left out, nothing below it
// goes in, however small, though "af" and "n" would fit here (bench
// 2026-08-03: {"af":"","tr":1} at MTU 23, the vital fields all gone).
TEST(TelemetryJsonPacker, Tier1FloorStopsTheRest)
{
    char buf[64];
    TelemetryJsonPacker pk(buf, sizeof(buf), 30);
    pk.addString("st", "READY");
    pk.addDouble("lat", 34.12345, 5);   // does not fit
    pk.endTier1();
    pk.addString("af", "");
    pk.addInt("n", 7);
    pk.finish();
    EXPECT_STREQ(buf, "{\"st\":\"READY\",\"tr\":1}");
}

// NaN says "no value" and costs nothing; an infinity would print "inf", which
// is not JSON and would cost the app the whole frame. Neither flags the frame.
TEST(TelemetryJsonPacker, NonFiniteLeftOutUnflagged)
{
    char buf[64];
    TelemetryJsonPacker pk(buf, sizeof(buf), 509);
    pk.addFloat("a", std::numeric_limits<float>::quiet_NaN(), 1);
    pk.addFloat("b", std::numeric_limits<float>::infinity(), 1);
    pk.addDouble("c", -std::numeric_limits<double>::infinity(), 5);
    pk.addInt("d", 3);
    pk.finish();
    EXPECT_STREQ(buf, "{\"d\":3}");
    EXPECT_FALSE(pk.dropped());
}

// An absurd value too long for the scratch buffer is left out and flagged,
// never written past it.
TEST(TelemetryJsonPacker, AbsurdNumberDroppedNotOverrun)
{
    char buf[640];
    TelemetryJsonPacker pk(buf, sizeof(buf), 509);
    pk.addDouble("lat", 1e300, 5);
    pk.addFloat("x", 3.4e38f, 1);
    pk.addInt("d", 3);
    pk.finish();
    EXPECT_STREQ(buf, "{\"d\":3,\"tr\":1}");
}

// The widest int and uint still print whole.
TEST(TelemetryJsonPacker, IntegerExtremes)
{
    char buf[64];
    TelemetryJsonPacker pk(buf, sizeof(buf), 509);
    pk.addInt("i", std::numeric_limits<int32_t>::min());
    pk.addUint("u", std::numeric_limits<uint32_t>::max());
    pk.finish();
    EXPECT_STREQ(buf, "{\"i\":-2147483648,\"u\":4294967295}");
}

// A buffer smaller than the window caps the frame at the buffer.
TEST(TelemetryJsonPacker, BufferSmallerThanWindow)
{
    char buf[24];
    std::memset(buf, 'X', sizeof(buf));
    TelemetryJsonPacker pk(buf, sizeof(buf), 509);
    for (int i = 0; i < 10; i++) pk.addInt("k", 1234);
    pk.finish();
    EXPECT_LT(std::strlen(buf), sizeof(buf));
    EXPECT_TRUE(has(buf, "\"tr\":1}"));
}

// Every window from the 23-B default to MTU 512, every frame: never past the
// window, always closed, "tr":1 exactly when something was left out.
TEST(TelemetryJsonPacker, NeverPastTheWindow)
{
    uint32_t r = 0x1234567u;
    auto rnd = [&r]() { r = r * 1664525u + 1013904223u; return r >> 8; };
    for (size_t window = 20; window <= 509; window++)
    {
        for (int rep = 0; rep < 4; rep++)
        {
            char buf[640];
            TelemetryJsonPacker pk(buf, sizeof(buf), window);
            const int nfields = 10 + (int)(rnd() % 60);
            for (int i = 0; i < nfields; i++)
            {
                const char* keys[] = {"st", "lat", "q0", "frx", "mfw", "af", "h"};
                const char* key = keys[rnd() % 7];
                switch (rnd() % 5)
                {
                    case 0: pk.addInt(key, (int)(rnd() % 2000000) - 1000000); break;
                    case 1: pk.addUint(key, rnd()); break;
                    case 2: pk.addFloat(key, (float)((int)(rnd() % 200000) - 100000) / 7.0f, 1 + (int)(rnd() % 3)); break;
                    case 3: pk.addDouble(key, ((double)rnd() - 8e6) / 3.0, 5); break;
                    default: pk.addString(key, (rnd() % 2) ? "8b2e11f5+20261122-1217" : ""); break;
                }
                if (i == 8) pk.endTier1();
            }
            const size_t n = pk.finish();
            const std::string s(buf);
            ASSERT_EQ(n, s.size());
            ASSERT_LE(s.size(), window) << "window " << window;
            ASSERT_EQ(s.front(), '{');
            ASSERT_EQ(s.back(), '}');
            ASSERT_EQ(pk.dropped(), has(s, "\"tr\":1}")) << s;
        }
    }
}

// A base station's relay of a V10 with a fix: ~545 B, so it must trim. At real
// widths the dropped-frame count and the high-g X still go in (at 24 B a number
// fdr, hx, hy, hz and af all went); the lowest of Tier 3 is what falls off.
TEST(TelemetryJsonPacker, RelayWithFixKeepsMoreOfTier3)
{
    char buf[640];
    TelemetryJsonPacker pk(buf, sizeof(buf), 512 - 3);
    pk.addInt("rid", 7);
    pk.addString("st", "READY");
    pk.addInt("fs", 16);
    pk.addInt("h", 1091445);
    pk.addDouble("lat", 34.12345, 5);
    pk.addDouble("lon", -117.12345, 5);
    pk.addInt("nsat", 15);
    pk.addInt("hacc", 2);
    pk.addFloat("palt", 0.3f, 1);
    pk.addFloat("arate", 0.0f, 1);
    pk.addFloat("galt", 412.6f, 1);
    pk.addFloat("ve", -0.0f, 1);
    pk.addFloat("vn", 0.1f, 1);
    pk.addFloat("vu", -0.1f, 1);
    pk.addFloat("malt", 0.0f, 1);
    pk.addFloat("mspd", 0.0f, 1);
    pk.addFloat("soc", 79.3f, 1);
    pk.addFloat("cur", -285.0f, 1);
    pk.addFloat("vol", 7.90f, 2);
    pk.endTier1();
    pk.addFloat("rcmd", 0.0f, 1);
    pk.addFloat("q0", 0.707f, 3);
    pk.addFloat("q1", -0.012f, 3);
    pk.addFloat("q2", 0.004f, 3);
    pk.addFloat("q3", -0.707f, 3);
    pk.addFloat("gx", 0.0f, 1);
    pk.addFloat("gy", 0.0f, 1);
    pk.addFloat("gz", 0.1f, 1);
    pk.addFloat("lx", -0.6f, 1);
    pk.addFloat("ly", -0.4f, 1);
    pk.addFloat("lz", 9.8f, 1);
    pk.addFloat("ccur", 0.0f, 2);
    pk.addFloat("scur", 0.0f, 2);
    pk.addUint("gi", 1);
    pk.addFloat("rssi", -62.0f, 0);
    pk.addFloat("snr", 9.8f, 1);
    pk.addFloat("bsoc", 88.1f, 1);
    pk.addFloat("bvol", 4.05f, 2);
    pk.addFloat("bcur", -120.0f, 0);
    pk.addFloat("pvol", 8.11f, 2);
    pk.addFloat("pc1", 4.06f, 2);
    pk.addFloat("pc2", 4.05f, 2);
    pk.addFloat("rxk", 0.0f, 1);
    pk.addFloat("wrk", 0.0f, 1);
    pk.addUint("frx", 0);
    pk.addUint("fdr", 0);
    pk.addFloat("hx", -0.5f, 1);
    pk.addFloat("hy", -1.9f, 1);
    pk.addFloat("hz", 10.3f, 1);
    pk.addString("run", "Mantis V10.2");
    pk.addString("af", "");
    pk.finish();
    const std::string s(buf);
    EXPECT_TRUE(pk.dropped());
    EXPECT_LE(s.size(), 509u);
    EXPECT_TRUE(has(s, "\"rid\":7,\"st\":\"READY\""));
    EXPECT_TRUE(has(s, ",\"frx\":0,\"fdr\":0,\"hx\":-0.5,\"tr\":1}"));
    EXPECT_FALSE(has(s, "\"hy\""));
    EXPECT_FALSE(has(s, "\"run\""));
}
