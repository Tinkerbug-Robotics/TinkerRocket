// Host tests for the Quectel LC86G NMEA/PQTM stream parser (Lc86Parser) —
// the protocol half of the mini's TR_GNSSReceiverLC86_Serial driver.
//
// What is pinned here and why it matters in flight:
//   - the NMEA checksum algorithm, anchored against the protocol doc's OWN
//     published example checksums (not against this codebase's helpers);
//   - the $PQTMPVT → GNSSData field mapping, including vel_u_mmps = -VelD —
//     PQTMPVT is the only LC86G sentence with vertical velocity, the EKF
//     hard-fuses velD at sigma 0.3 m/s, and a flipped sign reads a climbing
//     rocket as diving;
//   - the fix-trust gate (Quality==0 forces fix_mode=0 — the #562 gnssFixOK
//     analogue);
//   - the $PQTMEPE h/v-accuracy merge into subsequent PVT publishes;
//   - framing robustness: junk bytes, corrupted checksums, truncated lines,
//     overlong lines, and reassembly of sentences split across feeds (the
//     driver's pollNewPVT drains bounded chunks, so splits are routine);
//   - the 42-byte packed GNSSData layout the log/wire formats depend on.
//
// Protocol reference: Quectel LC26G/LC76G/LC86G GNSS Protocol Specification
// V1.4 (doc section numbers cited inline).

#include <gtest/gtest.h>

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
// printed. Parsing it end-to-end anchors both the checksum algorithm and the
// field mapping against the DOCUMENT rather than against our own helpers.
const char* kDocPvtLine =
    "$PQTMPVT,1,459596000,20230505,073938.000,1,3,31,18,31.82176510,"
    "117.11534360,99.357,-0.337,-0.004,0.000,-0.003,0.004,96.91,0.51,0.93*74\r\n";

}  // namespace

// ───────────────────────── checksum / framing ─────────────────────────

TEST(Lc86Checksum, MatchesProtocolDocExamples)
{
    // Every expected value below is printed verbatim in the protocol doc —
    // these anchor the XOR-between-$-and-* rule (commas included) externally.
    struct { const char* body; uint8_t cs; } cases[] = {
        {"PAIR050,1000",        0x12},  // §2.4.10
        {"PAIR001,004,0",       0x3F},  // §2.4.1
        {"PAIR002",             0x38},  // §2.4.2
        {"PAIR062,0,3",         0x3D},  // §2.4.14
        {"PAIR066,1,1,1,1,0,0", 0x3A},  // §2.4.16
        {"PAIR513",             0x3D},  // §2.4.49
        {"PAIR864,0,0,115200",  0x1B},  // §2.4.69
    };
    for (const auto& c : cases)
    {
        EXPECT_EQ(lc86::checksum(c.body, strlen(c.body)), c.cs) << c.body;
    }
}

TEST(Lc86Checksum, BuildSentenceFramesDocExample)
{
    char buf[64];
    const size_t n = lc86::buildSentence("PAIR050,1000", buf, sizeof(buf));
    EXPECT_EQ(std::string(buf, n), "$PAIR050,1000*12\r\n");
}

TEST(Lc86Checksum, BuildSentenceRejectsTinyBuffer)
{
    char buf[8];
    EXPECT_EQ(lc86::buildSentence("PAIR050,1000", buf, sizeof(buf)), 0u);
}

TEST(Lc86Checksum, BuiltSentencesRoundTripThroughParser)
{
    Lc86Parser p;
    const auto evs = feedAll(p, frame("PAIR050,100"));
    // Address PAIR050 is not a sentence we decode — but its checksum must
    // verify, proving build and verify agree.
    EXPECT_EQ(count(evs, Event::OTHER_VALID), 1);
    EXPECT_EQ(p.badLines(), 0u);
}

// ───────────────────────── $PQTMPVT mapping ─────────────────────────

TEST(Lc86Pvt, DocExampleMapsToGNSSData)
{
    Lc86Parser p;
    const auto evs = feedAll(p, kDocPvtLine);
    ASSERT_EQ(count(evs, Event::PVT), 1);

    const GNSSData& d = p.pvt();
    EXPECT_EQ(d.time_us, 0u);  // owner stamps MCU time, parser must not
    EXPECT_EQ(d.year, 2023);
    EXPECT_EQ(d.month, 5);
    EXPECT_EQ(d.day, 5);
    EXPECT_EQ(d.hour, 7);
    EXPECT_EQ(d.minute, 39);
    EXPECT_EQ(d.second, 38);
    EXPECT_EQ(d.milli_second, 0);
    EXPECT_EQ(d.fix_mode, 3);          // FixMode 3, Quality 1 → trusted
    EXPECT_EQ(d.num_sats, 31);
    EXPECT_EQ(d.pdop_x10, 9);          // PDOP 0.93 → 9.3 → 9
    EXPECT_EQ(d.lat_e7, 318217651);    // 31.82176510 deg
    EXPECT_EQ(d.lon_e7, 1171153436);   // 117.11534360 deg
    EXPECT_EQ(d.alt_mm, 99357);        // 99.357 m MSL
    EXPECT_EQ(d.vel_n_mmps, -4);       // VelN -0.004 m/s
    EXPECT_EQ(d.vel_e_mmps, 0);        // VelE  0.000 m/s
    EXPECT_EQ(d.vel_u_mmps, 3);        // -VelD = -(-0.003) m/s
    EXPECT_EQ(d.h_acc_m, 255);         // no $PQTMEPE seen yet → unknown
    EXPECT_EQ(d.v_acc_m, 255);
}

TEST(Lc86Pvt, VelUSignIsNegatedVelD)
{
    // Descending at VelD = +12.5 m/s (down-positive NED) must publish as
    // vel_u_mmps = -12500.
    Lc86Parser p;
    const auto evs = feedAll(p, frame(
        "PQTMPVT,1,1000,20260816,120000.000,1,3,12,18,40.0,-105.0,1000.0,0.0,"
        "1.0,2.0,12.5,2.236,90.0,1.0,1.5"));
    ASSERT_EQ(count(evs, Event::PVT), 1);

    const GNSSData& d = p.pvt();
    EXPECT_EQ(d.vel_n_mmps, 1000);
    EXPECT_EQ(d.vel_e_mmps, 2000);
    EXPECT_EQ(d.vel_u_mmps, -12500);
    EXPECT_EQ(d.lat_e7, 400000000);
    EXPECT_EQ(d.lon_e7, -1050000000);  // west longitude stays signed
    EXPECT_EQ(d.alt_mm, 1000000);
    EXPECT_EQ(d.pdop_x10, 15);
}

TEST(Lc86Pvt, QualityZeroForcesNoFix)
{
    // FixMode=3 but Quality=0: the module's own validity word wins (the #562
    // gnssFixOK analogue) — publish fix_mode 0 so downstream `fix_mode >= 3`
    // gates fail. Sat count still parses for acquisition visibility.
    Lc86Parser p;
    const auto evs = feedAll(p, frame(
        "PQTMPVT,1,1000,20260816,120000.000,0,3,9,18,40.0,-105.0,1000.0,0.0,"
        "0.0,0.0,0.0,0.0,0.0,1.0,1.5"));
    ASSERT_EQ(count(evs, Event::PVT), 1);
    EXPECT_EQ(p.pvt().fix_mode, 0);
    EXPECT_EQ(p.pvt().num_sats, 9);
}

TEST(Lc86Pvt, EmptyFieldsParseAsZeroNoFix)
{
    // A fixless module emits empty numeric fields; must not crash and must
    // not publish anything that looks like a trusted fix.
    Lc86Parser p;
    const auto evs = feedAll(p, frame("PQTMPVT,1,,,,0,0,0,,,,,,,,,,,,"));
    ASSERT_EQ(count(evs, Event::PVT), 1);
    EXPECT_EQ(p.pvt().fix_mode, 0);
    EXPECT_EQ(p.pvt().lat_e7, 0);
    EXPECT_EQ(p.pvt().alt_mm, 0);
    EXPECT_EQ(p.pvt().vel_u_mmps, 0);
    EXPECT_EQ(p.pvt().year, 0);
}

TEST(Lc86Pvt, UnknownMsgVerRejected)
{
    // MsgVer is documented "always 1"; a different version means the field
    // positions can't be trusted — reject rather than guess.
    Lc86Parser p;
    const auto evs = feedAll(p, frame(
        "PQTMPVT,2,1000,20260816,120000.000,1,3,9,18,40.0,-105.0,1000.0,0.0,"
        "0.0,0.0,0.0,0.0,0.0,1.0,1.5"));
    EXPECT_EQ(count(evs, Event::PVT), 0);
    EXPECT_EQ(count(evs, Event::BAD), 1);
    EXPECT_EQ(p.badLines(), 1u);
}

TEST(Lc86Pvt, InvalidPdopSaturates)
{
    // 99.99 is the documented "invalid" PDOP marker → 999.9 → saturates 255.
    Lc86Parser p;
    const auto evs = feedAll(p, frame(
        "PQTMPVT,1,1000,20260816,120000.000,1,2,5,18,40.0,-105.0,1000.0,0.0,"
        "0.0,0.0,0.0,0.0,0.0,99.99,99.99"));
    ASSERT_EQ(count(evs, Event::PVT), 1);
    EXPECT_EQ(p.pvt().pdop_x10, 255);
    EXPECT_EQ(p.pvt().fix_mode, 2);
}

TEST(Lc86Pvt, FractionalSecondsToMilliseconds)
{
    Lc86Parser p;
    feedAll(p, frame(
        "PQTMPVT,1,1000,20260816,235959.250,1,3,9,18,40.0,-105.0,1000.0,0.0,"
        "0.0,0.0,0.0,0.0,0.0,1.0,1.5"));
    EXPECT_EQ(p.pvt().hour, 23);
    EXPECT_EQ(p.pvt().minute, 59);
    EXPECT_EQ(p.pvt().second, 59);
    EXPECT_EQ(p.pvt().milli_second, 250);
}

// ───────────────────────── $PQTMEPE merge ─────────────────────────

TEST(Lc86Epe, MergesIntoSubsequentPvt)
{
    Lc86Parser p;

    // Before any EPE, accuracies read 255 (unknown/worst).
    feedAll(p, kDocPvtLine);
    EXPECT_EQ(p.pvt().h_acc_m, 255);
    EXPECT_EQ(p.pvt().v_acc_m, 255);

    // $PQTMEPE,<MsgVer=2>,<EPE_North>,<EPE_East>,<EPE_Down>,<EPE_2D>,<EPE_3D>
    const auto evs = feedAll(p, frame("PQTMEPE,2,1.2,1.4,5.6,3.4,6.5"));
    EXPECT_EQ(count(evs, Event::EPE), 1);

    feedAll(p, kDocPvtLine);
    EXPECT_EQ(p.pvt().h_acc_m, 3);  // EPE_2D   3.4 m → 3
    EXPECT_EQ(p.pvt().v_acc_m, 6);  // EPE_Down 5.6 m → 6
}

TEST(Lc86Epe, SaturatesAt255)
{
    Lc86Parser p;
    feedAll(p, frame("PQTMEPE,2,900.0,900.0,1234.5,999.9,1500.0"));
    feedAll(p, kDocPvtLine);
    EXPECT_EQ(p.pvt().h_acc_m, 255);
    EXPECT_EQ(p.pvt().v_acc_m, 255);
}

// ───────────────────────── acks ─────────────────────────

TEST(Lc86Ack, PairAckParsed)
{
    Lc86Parser p;
    // Doc §2.4.10's published ack for $PAIR050.
    const auto evs = feedAll(p, "$PAIR001,050,0*3E\r\n");
    ASSERT_EQ(count(evs, Event::PAIR_ACK), 1);
    EXPECT_EQ(p.ackCommandId(), 50);
    EXPECT_EQ(p.ackResult(), 0);

    // Doc §2.4.1's example: $PAIR001,004,0*3F.
    const auto evs2 = feedAll(p, "$PAIR001,004,0*3F\r\n");
    ASSERT_EQ(count(evs2, Event::PAIR_ACK), 1);
    EXPECT_EQ(p.ackCommandId(), 4);
}

TEST(Lc86Ack, QtmCfgMsgRateResponses)
{
    Lc86Parser p;
    auto evs = feedAll(p, frame("PQTMCFGMSGRATE,OK"));
    EXPECT_EQ(count(evs, Event::QTM_OK), 1);

    evs = feedAll(p, frame("PQTMCFGMSGRATE,ERROR,2"));
    EXPECT_EQ(count(evs, Event::QTM_ERROR), 1);
    EXPECT_EQ(p.qtmErrorCode(), 2);
}

// ───────────────────────── GGA (debug/liveness only) ─────────────────────────

TEST(Lc86Gga, NumSatsAndQuality)
{
    Lc86Parser p;
    const auto evs = feedAll(p, frame(
        "GNGGA,073938.000,3149.30591,N,11706.92062,E,1,31,0.51,99.4,M,-0.3,M,,"));
    ASSERT_EQ(count(evs, Event::GGA), 1);
    EXPECT_EQ(p.ggaNumSats(), 31);
    EXPECT_EQ(p.ggaQuality(), 1);
}

// ───────────────────────── robustness ─────────────────────────

TEST(Lc86Robustness, ChecksumMismatchIsDiscarded)
{
    Lc86Parser p;
    std::string corrupted = kDocPvtLine;
    corrupted[10] ^= 0x01;  // flip one payload character
    const auto evs = feedAll(p, corrupted);
    EXPECT_EQ(count(evs, Event::PVT), 0);
    EXPECT_EQ(count(evs, Event::BAD), 1);
    EXPECT_EQ(p.badLines(), 1u);

    // A clean line right after parses normally.
    const auto evs2 = feedAll(p, kDocPvtLine);
    EXPECT_EQ(count(evs2, Event::PVT), 1);
}

TEST(Lc86Robustness, BinaryJunkBetweenSentences)
{
    Lc86Parser p;
    std::string stream;
    stream.append("\xFF", 1);
    stream.append("\x00", 1);  // embedded NUL must not confuse assembly
    stream.append("\x7E\x12 garbage without a dollar\n");
    stream += kDocPvtLine;
    stream.append("\xB5\x62\x05\x01", 4);  // stray UBX-looking bytes
    stream += frame("PAIR001,050,0");
    const auto evs = feedAll(p, stream);
    EXPECT_EQ(count(evs, Event::PVT), 1);
    EXPECT_EQ(count(evs, Event::PAIR_ACK), 1);
}

TEST(Lc86Robustness, PartialLineReassemblyAcrossFeeds)
{
    // pollNewPVT drains bounded chunks, so a sentence routinely straddles
    // calls — feed the doc line in 7-byte slices and expect exactly one PVT.
    Lc86Parser p;
    const std::string line = kDocPvtLine;
    int pvts = 0;
    for (size_t off = 0; off < line.size(); off += 7)
    {
        const size_t end = (off + 7 < line.size()) ? off + 7 : line.size();
        for (size_t i = off; i < end; i++)
        {
            if (p.feed((uint8_t)line[i]) == Event::PVT) pvts++;
        }
    }
    EXPECT_EQ(pvts, 1);
    EXPECT_EQ(p.pvt().lat_e7, 318217651);
}

TEST(Lc86Robustness, TruncatedLineResyncsOnDollar)
{
    // A power glitch / RX overflow cuts a sentence short; the next '$' must
    // resynchronize with no residue from the dead line.
    Lc86Parser p;
    std::string stream = "$PQTMPVT,1,459596000,20230505,073";  // cut mid-field
    stream += kDocPvtLine;
    const auto evs = feedAll(p, stream);
    EXPECT_EQ(count(evs, Event::PVT), 1);
    EXPECT_EQ(p.truncatedLines(), 1u);
    EXPECT_EQ(p.pvt().lat_e7, 318217651);
}

TEST(Lc86Robustness, OverlongLineDiscarded)
{
    // A wrong-baud stream can look like one endless line; it must be dropped
    // at the length cap and the parser must recover for real sentences.
    Lc86Parser p;
    std::string stream = "$PQTMPVT,";
    stream.append(400, '9');
    const auto evs = feedAll(p, stream);
    EXPECT_EQ(count(evs, Event::BAD), 1);
    EXPECT_EQ(p.overlongLines(), 1u);

    const auto evs2 = feedAll(p, kDocPvtLine);
    EXPECT_EQ(count(evs2, Event::PVT), 1);
}

TEST(Lc86Robustness, ResetLineDropsPartialAssembly)
{
    // The driver calls resetLine() after a baud change; a half-line from the
    // old baud must not glue onto the first sentence at the new baud.
    Lc86Parser p;
    feedAll(p, "$PQTMPVT,1,459596");
    p.resetLine();
    const auto evs = feedAll(p, kDocPvtLine);
    EXPECT_EQ(count(evs, Event::PVT), 1);
    EXPECT_EQ(p.badLines(), 0u);
}

TEST(Lc86Robustness, FullEpochBurstYieldsAllEvents)
{
    // A configured module at 10 Hz emits PQTMPVT + PQTMEPE + GGA per epoch;
    // one bounded drain may deliver all of them back to back.
    Lc86Parser p;
    std::string stream;
    stream += frame("PQTMEPE,2,1.0,1.0,2.0,1.5,2.5");
    stream += kDocPvtLine;
    stream += frame(
        "GNGGA,073938.000,3149.30591,N,11706.92062,E,1,31,0.51,99.4,M,-0.3,M,,");
    const auto evs = feedAll(p, stream);
    EXPECT_EQ(count(evs, Event::EPE), 1);
    EXPECT_EQ(count(evs, Event::PVT), 1);
    EXPECT_EQ(count(evs, Event::GGA), 1);
    EXPECT_EQ(p.pvt().h_acc_m, 2);  // EPE_2D  1.5 → 2 (round half up)
    EXPECT_EQ(p.pvt().v_acc_m, 2);  // EPE_Down 2.0 → 2
}

// ───────────────────────── packed GNSSData invariants ─────────────────────────

TEST(GnssDataPacking, FortyTwoByteLayout)
{
    // The NAND log and LoRa/BLE wire formats serialize GNSSData as raw bytes;
    // RocketComputerTypes.h static_asserts the size, and these offsets pin the
    // exact field order the parser writes.
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
