// Host tests for what the GNSS wire formats and the RTCM3 decoders share:
// the packed GNSSData and GNSS_SAT_MSG records the log and the radio links
// carry, the RTCM3 helpers (Rtcm3Msm.cpp: CRC-24Q, DF407 lock times, the
// MSM7 decode the gnss-sdr-dev driver uses too), and the RTCM3 half of the
// LC86G stream parser (Lc86Parser), which stays as a host decoder for the
// Tinker-Beetle V1's logged raw frames: tools/raw_replay, tinkerrocket-sim
// and the raw-GNSS tests' golden capture read them through it.
//
// Moved here from test_lc86_parser.cpp when the Beetle V1 (TR_BOARD_M1) and
// its LC86G flight driver were retired. That file's tests of the NMEA, PQTM
// and PAIR sentences, which only the flight driver read, went with it.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstring>
#include <string>
#include <vector>

#include "Lc86Parser.h"

using lc86::Event;
using lc86::Lc86Parser;

namespace
{

std::string frame(const std::string& body)
{
    char buf[256];
    const size_t n = lc86::buildSentence(body.c_str(), buf, sizeof(buf));
    EXPECT_GT(n, 0u) << body;
    return std::string(buf, n);
}

std::vector<Event> feedAll(Lc86Parser& p, const std::string& bytes)
{
    std::vector<Event> evs;
    for (char c : bytes)
    {
        const Event ev = p.feed((uint8_t)c);
        if (ev != Event::NONE) evs.push_back(ev);
    }
    return evs;
}

int count(const std::vector<Event>& evs, Event e)
{
    int n = 0;
    for (Event ev : evs)
    {
        if (ev == e) n++;
    }
    return n;
}

// The protocol doc's own published $PQTMPVT example (§2.3.8), checksum *74 as
// printed: the NMEA sentence the RTCM3 tests below put around their frames,
// as the module interleaved the two streams on one UART.
const char* kDocPvtLine =
    "$PQTMPVT,1,459596000,20230505,073938.000,1,3,31,18,31.82176510,"
    "117.11534360,99.357,-0.337,-0.004,0.000,-0.003,0.004,96.91,0.51,0.93*74\r\n";

}  // namespace

// ───────────────────────── packed records ─────────────────────────

TEST(GnssDataPacking, FortyTwoByteLayout)
{
    // The NAND log and LoRa/BLE wire formats serialize GNSSData as raw bytes;
    // RocketComputerTypes.h static_asserts the size, and these offsets pin the
    // exact field order every GNSS driver writes.
    EXPECT_EQ(sizeof(GNSSData), 42u);
    EXPECT_EQ(offsetof(GNSSData, time_us), 0u);
    EXPECT_EQ(offsetof(GNSSData, year), 4u);
    EXPECT_EQ(offsetof(GNSSData, month), 6u);
    EXPECT_EQ(offsetof(GNSSData, day), 7u);
    EXPECT_EQ(offsetof(GNSSData, hour), 8u);
    EXPECT_EQ(offsetof(GNSSData, minute), 9u);
    EXPECT_EQ(offsetof(GNSSData, second), 10u);
    EXPECT_EQ(offsetof(GNSSData, milli_second), 11u);
    EXPECT_EQ(offsetof(GNSSData, fix_mode), 13u);
    EXPECT_EQ(offsetof(GNSSData, num_sats), 14u);
    EXPECT_EQ(offsetof(GNSSData, pdop_x10), 15u);
    EXPECT_EQ(offsetof(GNSSData, lat_e7), 16u);
    EXPECT_EQ(offsetof(GNSSData, lon_e7), 20u);
    EXPECT_EQ(offsetof(GNSSData, alt_mm), 24u);
    EXPECT_EQ(offsetof(GNSSData, vel_e_mmps), 28u);
    EXPECT_EQ(offsetof(GNSSData, vel_n_mmps), 32u);
    EXPECT_EQ(offsetof(GNSSData, vel_u_mmps), 36u);
    EXPECT_EQ(offsetof(GNSSData, h_acc_m), 40u);
    EXPECT_EQ(offsetof(GNSSData, v_acc_m), 41u);
}

TEST(GnssSatPacking, SatRecordPackingMatchesTheWire)
{
    // GNSS_SAT_MSG is serialized raw; gnssSatWireSize is what senders use.
    EXPECT_EQ(sizeof(GNSSSatBlock), 6u);
    EXPECT_EQ(sizeof(GNSSSatData), 202u);
    EXPECT_EQ(offsetof(GNSSSatData, time_us), 0u);
    EXPECT_EQ(offsetof(GNSSSatData, itow_ms), 4u);
    EXPECT_EQ(offsetof(GNSSSatData, num_svs), 8u);
    EXPECT_EQ(offsetof(GNSSSatData, num_blocks), 9u);
    EXPECT_EQ(offsetof(GNSSSatData, sat), GNSS_SAT_HEADER_BYTES);
    EXPECT_EQ(gnssSatWireSize(7), 10u + 6u * 7u);
}

// ───────────────────────── RTCM3 MSM7 raw measurements ─────────────────────────
//
// One receiver epoch as the Beetle's own LC86G sent it ($PAIR432,1, open sky,
// 2026-09-25 22:20 local, gnss/cocom-rig/sdr/results/lc86g_sky_20260925.log.gz):
// GPS, GLONASS, Galileo, BeiDou, then an empty QZSS message that closes the
// epoch, plus the next epoch's GPS message. The capture stores CRC-checked
// payloads; the frames are rebuilt around them. Expected values come from an
// independent Python decode of the same payloads (RTCM 10403.3 field tables),
// not from this code. The binary carries '$', CR, LF and 0xD3 bytes inside
// frames, which is the framing hazard these tests exist for.

namespace
{

const char* kGps =
    "d30099435d187d2eff42002061824a0200000000200000007fd1d1515212131493514000"
    "00000075e22b8a1ace5618efb91cfa7c059043420df923e12ff93e2dfdabde3a0cfa4ca2"
    "ef3db3246bbb8556cc02a3fa6c088c6bff775a433cf1425a043f4ee701a7d34153f04008"
    "21feae56020de8000001374a9e10000072de2c6398a7c40fe484e4b372cd84419459d81b"
    "dec3bd37896a0d8a1bd00a28df3011";
const char* kGlo =
    "d3005043fd18c42874020020001c020000000000200000007a7212120bb2a57275fabf98"
    "4d6064fcec09068e1b1b5306552d7b797f9dca740000079549e7ed19fb0800000003cb4b"
    "fc08910074249df82b9570cc7ae0";
const char* kGal =
    "d30099449d187d2eff4200200805153800000000200000007fd5d4961513d4d5d6558000"
    "000002787ea5a8e96bf77bf1be17013ff0504441e700bfec7f6d81650457d09402fe67f5"
    "d347987f938e01c17fab25750074a6e07f4174c0beac3fd74a01e5933e4e83c070c27eac"
    "387d3ef4c2aa1c80000000000000000000156003b495af887b66eab82af443ff088ef70a"
    "91351b5c7a87f94e103c3b683a49cd";
const char* kBds =
    "d3007c467d187d2e24820020000000e131000000200000007f4a574a48524c5900000001"
    "148d6dc52931c9217f08f6ec108fc6bda1057c23ac84ad2f7c0d3041ecd9511510e276f1"
    "f1f1f20ed40bda77f514a1fb13a5044f67800000079bc37d800698ee0000051935bac9c2"
    "711bceca43010e869d131c20801fdc9a90f220f60a73";
const char* kQzss =
    "d3001645dd187d2eff40002000000000000000000000000000e8aea0";
const char* kGpsNext =
    "d30099435d187d2f0ee2002061824a0200000000200000007fd1d1515212131493514000"
    "00000074e26b9a22cd5598efb11cfa80059043420df923e13ff97e2dfdab730576a5d478"
    "f636dfcb67b20f51a3d1de476c538589fdcac820000001b3e0bda5fbbd9e7743d3353f46"
    "5841ed2f3df4aa4000013b4b9e10000072de3c6368a9c2701496deb412b58dd1a8121848"
    "2f1cbcacfd1ff74f1d681938d18c5d";

std::string unhex(const char* h)
{
    std::string out;
    for (size_t i = 0; h[i] && h[i + 1]; i += 2)
    {
        auto v = [](char c) { return (c <= '9') ? c - '0' : c - 'a' + 10; };
        out.push_back((char)((v(h[i]) << 4) | v(h[i + 1])));
    }
    return out;
}

std::string wholeEpoch()
{
    return unhex(kGps) + unhex(kGlo) + unhex(kGal) + unhex(kBds) + unhex(kQzss);
}

const rtcm3::MsmCell* findCell(const rtcm3::MsmEpoch& e, uint8_t gnss_id, uint8_t sv_id)
{
    for (unsigned i = 0; i < e.n_cells; i++)
    {
        if (e.cells[i].gnss_id == gnss_id && e.cells[i].sv_id == sv_id) return &e.cells[i];
    }
    return nullptr;
}

}  // namespace

TEST(Rtcm3, Crc24qMatchesTheStandardCheckValue)
{
    // CRC-24Q's published check value: "123456789" -> 0xCDE703.
    const char* s = "123456789";
    EXPECT_EQ(rtcm3::crc24q((const uint8_t*)s, 9), 0xCDE703u);
}

TEST(Rtcm3, LockTimeIndicatorBands)
{
    // DF407 (RTCM 10403.3 Table 3.5-74): 1 ms steps to 63, then each band of
    // 32 doubles the step; 704 saturates.
    EXPECT_EQ(rtcm3::lockTimeMs(0), 0u);
    EXPECT_EQ(rtcm3::lockTimeMs(63), 63u);
    EXPECT_EQ(rtcm3::lockTimeMs(64), 64u);       // 2i - 64
    EXPECT_EQ(rtcm3::lockTimeMs(95), 126u);
    EXPECT_EQ(rtcm3::lockTimeMs(96), 128u);      // 4i - 256
    EXPECT_EQ(rtcm3::lockTimeMs(311), 14080u);   // 256i - 65536 (G07 below)
    EXPECT_EQ(rtcm3::lockTimeMs(703), 66060288u);
    EXPECT_EQ(rtcm3::lockTimeMs(704), 67108864u);
}

TEST(Lc86Msm7, RealEpochDecodesAsTheModuleSentIt)
{
    Lc86Parser p;
    const auto evs = feedAll(p, wholeEpoch());
    EXPECT_EQ(count(evs, Event::MSM_EPOCH), 1);
    EXPECT_EQ(count(evs, Event::RTCM), 4);
    EXPECT_EQ(count(evs, Event::BAD), 0);
    EXPECT_EQ(p.rtcmFrames(), 5u);
    EXPECT_EQ(p.msmMessages(), 5u);

    rtcm3::MsmEpoch e;
    ASSERT_TRUE(p.takeMsmEpoch(e));
    EXPECT_FALSE(p.takeMsmEpoch(e));   // taken once
    EXPECT_TRUE(e.complete);
    EXPECT_EQ(e.n_msgs, 5);
    EXPECT_EQ(e.n_cells, 29);          // GPS 9, GLONASS 4, Galileo 9, BeiDou 7
    EXPECT_EQ(e.dropped, 0);
    EXPECT_EQ(e.gnss_mask, (1u << 0) | (1u << 2) | (1u << 3) | (1u << 5) | (1u << 6));
    EXPECT_EQ(e.gps_tow_ms, 525058000u);

    // G30, the strongest GPS satellite: carrier locked for 9 minutes.
    const rtcm3::MsmCell* g30 = findCell(e, 0, 30);
    ASSERT_NE(g30, nullptr);
    EXPECT_NEAR(g30->pseudorange_m, 20768905.4604, 1e-3);
    EXPECT_NEAR(g30->phase_range_m, 20768900.3460, 1e-3);
    EXPECT_NEAR(g30->rate_mps, -149.9350, 2e-3);   // float: ~1 mm/s at 150 m/s
    EXPECT_FLOAT_EQ(g30->cn0_dbhz, 44.125f);
    EXPECT_EQ(g30->lock_ms, 557056u);
    EXPECT_EQ(g30->sig_id, 2);                     // L1 C/A
    EXPECT_FALSE(g30->half_cycle);

    // G01 is newly acquired: no lock time yet, half-cycle unresolved.
    const rtcm3::MsmCell* g01 = findCell(e, 0, 1);
    ASSERT_NE(g01, nullptr);
    EXPECT_NEAR(g01->pseudorange_m, 21319498.8365, 1e-3);
    EXPECT_EQ(g01->lock_ms, 0u);
    EXPECT_TRUE(g01->half_cycle);

    // GLONASS carries its frequency channel (+7) in DF419.
    const rtcm3::MsmCell* r11 = findCell(e, 6, 11);
    ASSERT_NE(r11, nullptr);
    EXPECT_NEAR(r11->pseudorange_m, 23587763.9219, 1e-3);
    EXPECT_NEAR(r11->rate_mps, 619.0116, 2e-3);
    EXPECT_EQ(r11->ext_info, 7);
    EXPECT_EQ(findCell(e, 6, 13)->ext_info, 5);

    const rtcm3::MsmCell* e28 = findCell(e, 2, 28);
    ASSERT_NE(e28, nullptr);
    EXPECT_NEAR(e28->pseudorange_m, 25938876.6578, 1e-3);
    EXPECT_NEAR(e28->phase_range_m, 25938878.7460, 1e-3);
    EXPECT_EQ(e28->lock_ms, 27648u);

    const rtcm3::MsmCell* c24 = findCell(e, 3, 24);
    ASSERT_NE(c24, nullptr);
    EXPECT_NEAR(c24->pseudorange_m, 22204715.3342, 1e-3);
    EXPECT_NEAR(c24->rate_mps, -247.8918, 2e-3);
    EXPECT_FLOAT_EQ(c24->cn0_dbhz, 45.8125f);

    // Every cell the module sent here has a valid pseudorange and Doppler.
    for (unsigned i = 0; i < e.n_cells; i++)
    {
        EXPECT_FALSE(std::isnan(e.cells[i].pseudorange_m)) << i;
        EXPECT_FALSE(std::isnan(e.cells[i].rate_mps)) << i;
    }
}

TEST(Lc86Msm7, BeiDouEpochIsShiftedToGpsTime)
{
    // BeiDou's epoch field counts BDT, 14 s behind GPS time: on its own the
    // BeiDou message must land on the same GPS TOW as the others.
    const std::string f = unhex(kBds);
    rtcm3::MsmHeader hdr;
    ASSERT_TRUE(rtcm3::peekMsm7((const uint8_t*)f.data() + 3, f.size() - 6, hdr));
    EXPECT_EQ(hdr.epoch_field, 525044000u);
    EXPECT_EQ(hdr.gps_tow_ms, 525058000u);
    EXPECT_TRUE(hdr.multiple);
}

TEST(Lc86Msm7, InterleavedWithNmeaBothStreamsParse)
{
    // The module writes its NMEA and its frames back to back; neither may
    // cost the other anything, fed byte by byte or in odd-sized slices.
    std::string stream;
    stream += kDocPvtLine;
    stream += wholeEpoch();
    stream += frame("PQTMEPE,2,1.0,1.0,2.0,1.5,2.5");
    stream += frame("GNGGA,073938.000,3149.30591,N,11706.92062,E,1,31,0.51,99.4,M,-0.3,M,,");
    stream += kDocPvtLine;

    for (size_t slice : {1u, 7u, 64u})
    {
        Lc86Parser p;
        std::vector<Event> evs;
        for (size_t off = 0; off < stream.size(); off += slice)
        {
            for (size_t i = off; i < off + slice && i < stream.size(); i++)
            {
                const Event ev = p.feed((uint8_t)stream[i]);
                if (ev != Event::NONE) evs.push_back(ev);
            }
        }
        EXPECT_EQ(count(evs, Event::PVT), 2) << slice;
        EXPECT_EQ(count(evs, Event::EPE), 1) << slice;
        EXPECT_EQ(count(evs, Event::GGA), 1) << slice;
        EXPECT_EQ(count(evs, Event::MSM_EPOCH), 1) << slice;
        EXPECT_EQ(count(evs, Event::BAD), 0) << slice;
        EXPECT_EQ(p.badLines(), 0u) << slice;
        EXPECT_EQ(p.truncatedLines(), 0u) << slice;
        EXPECT_EQ(p.msmEpoch().n_cells, 29) << slice;
    }
}

TEST(Lc86Msm7, CorruptFrameIsDroppedAndTheNextSentenceParses)
{
    Lc86Parser p;
    std::string bad = unhex(kGps);
    bad[40] ^= 0x10;   // one payload bit
    const auto evs = feedAll(p, bad + kDocPvtLine);
    EXPECT_EQ(count(evs, Event::BAD), 1);
    EXPECT_EQ(count(evs, Event::PVT), 1);
    EXPECT_EQ(p.rtcmCrcFails(), 1u);
    EXPECT_EQ(p.msmMessages(), 0u);
}

TEST(Lc86Msm7, StrayPreambleBeforeASentenceCostsNothing)
{
    // Line noise that reads as 0xD3 just before a '$': the reserved bits fail
    // on the very next byte, which is read as NMEA again.
    Lc86Parser p;
    const auto evs = feedAll(p, std::string("\xD3", 1) + kDocPvtLine);
    EXPECT_EQ(count(evs, Event::PVT), 1);
    EXPECT_EQ(p.rtcmBadHeaders(), 1u);
    EXPECT_EQ(p.rtcmFrames(), 0u);
}

TEST(Lc86Msm7, PreambleMidLineEndsThatLine)
{
    // A frame can only start between sentences, so a 0xD3 inside one means
    // the line was cut short (as a '$' would); the frame itself still counts.
    Lc86Parser p;
    const auto evs = feedAll(p, std::string("$PQTMPVT,1,459596") + wholeEpoch() + kDocPvtLine);
    EXPECT_EQ(p.truncatedLines(), 1u);
    EXPECT_EQ(count(evs, Event::MSM_EPOCH), 1);
    EXPECT_EQ(count(evs, Event::PVT), 1);
}

TEST(Lc86Msm7, LostClosingMessageIsClosedByTheNextEpoch)
{
    // Without the QZSS message (DF393 = 0) the epoch stays open until the
    // next epoch's GPS message arrives; it is handed out marked incomplete.
    Lc86Parser p;
    auto evs = feedAll(p, unhex(kGps) + unhex(kGlo) + unhex(kGal) + unhex(kBds));
    EXPECT_EQ(count(evs, Event::MSM_EPOCH), 0);
    evs = feedAll(p, unhex(kGpsNext));
    EXPECT_EQ(count(evs, Event::MSM_EPOCH), 1);

    rtcm3::MsmEpoch e;
    ASSERT_TRUE(p.takeMsmEpoch(e));
    EXPECT_FALSE(e.complete);
    EXPECT_EQ(e.n_cells, 29);
    EXPECT_EQ(e.gps_tow_ms, 525058000u);
    EXPECT_EQ(p.msmEpochsIncomplete(), 1u);

    // The GPS message that closed it opened the next epoch, which is not
    // handed out before its own close.
    EXPECT_EQ(p.msmEpochs(), 1u);
    EXPECT_FALSE(p.takeMsmEpoch(e));
}


TEST(Lc86Msm7, EveryCheckedFrameIsAvailableVerbatimForTheLog)
{
    // The log keeps the frames exactly as the module sent them, so after each
    // RTCM / MSM_EPOCH event rtcmFrame() must hold that frame byte for byte.
    Lc86Parser p;
    std::vector<std::string> got;
    for (const std::string& f : {unhex(kGps), unhex(kGlo), unhex(kGal), unhex(kBds), unhex(kQzss)})
    {
        for (char c : std::string(kDocPvtLine)) p.feed((uint8_t)c);   // NMEA between frames
        for (char c : f)
        {
            const Event ev = p.feed((uint8_t)c);
            if (ev == Event::RTCM || ev == Event::MSM_EPOCH)
            {
                got.emplace_back((const char*)p.rtcmFrame(), p.rtcmFrameLen());
            }
        }
    }
    ASSERT_EQ(got.size(), 5u);
    EXPECT_EQ(got[0], unhex(kGps));
    EXPECT_EQ(got[3], unhex(kBds));
    EXPECT_EQ(got[4], unhex(kQzss));
}

TEST(Lc86Msm7, AFrameThatChecksButDoesNotDecodeIsStillReported)
{
    // An MSM7 whose length disagrees with its own masks: CRC fine, decode
    // refused. It is counted as malformed but still offered for the log.
    std::string body = unhex(kQzss).substr(3, 22) + std::string(4, '\0');  // 4 stray bytes
    std::string f;
    f.push_back((char)0xD3);
    f.push_back((char)0x00);
    f.push_back((char)body.size());
    f += body;
    const uint32_t crc = rtcm3::crc24q((const uint8_t*)f.data(), f.size());
    f.push_back((char)(crc >> 16));
    f.push_back((char)(crc >> 8));
    f.push_back((char)crc);

    Lc86Parser p;
    const auto evs = feedAll(p, f);
    EXPECT_EQ(count(evs, Event::RTCM), 1);
    EXPECT_EQ(p.msmMalformed(), 1u);
    EXPECT_EQ(p.rtcmFrameLen(), f.size());
}

TEST(Lc86Msm7, EphemerisFramesAreCountedAndOfferedForTheLog)
{
    // A 1019 (GPS ephemeris) frame is not MSM7: it is reported as RTCM, kept
    // verbatim for the log and counted, and changes no MSM7 epoch.
    std::string body(61, '\0');
    body[0] = (char)(1019 >> 4);
    body[1] = (char)((1019 & 0x0F) << 4);
    std::string f;
    f.push_back((char)0xD3);
    f.push_back((char)0x00);
    f.push_back((char)body.size());
    f += body;
    const uint32_t crc = rtcm3::crc24q((const uint8_t*)f.data(), f.size());
    f.push_back((char)(crc >> 16));
    f.push_back((char)(crc >> 8));
    f.push_back((char)crc);

    Lc86Parser p;
    const auto evs = feedAll(p, f + kDocPvtLine);
    EXPECT_EQ(count(evs, Event::RTCM), 1);
    EXPECT_EQ(count(evs, Event::PVT), 1);
    EXPECT_EQ(p.rtcmEphFrames(), 1u);
    EXPECT_EQ(p.msmMessages(), 0u);
    EXPECT_EQ(std::string((const char*)p.rtcmFrame(), p.rtcmFrameLen()), f);
    EXPECT_TRUE(rtcm3::isEphemeris(1046));
    EXPECT_FALSE(rtcm3::isEphemeris(1077));
}

TEST(Lc86Msm7, ResetLineDropsAPartialFrame)
{
    Lc86Parser p;
    const std::string f = unhex(kGps);
    feedAll(p, f.substr(0, 60));
    p.resetLine();
    const auto evs = feedAll(p, kDocPvtLine);
    EXPECT_EQ(count(evs, Event::PVT), 1);
    EXPECT_EQ(p.rtcmFrames(), 0u);
}
