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
// printed. Parsing it end-to-end anchors both the checksum algorithm and the
// field mapping against the DOCUMENT rather than against our own helpers.
const char* kDocPvtLine =
    "$PQTMPVT,1,459596000,20230505,073938.000,1,3,31,18,31.82176510,"
    "117.11534360,99.357,-0.337,-0.004,0.000,-0.003,0.004,96.91,0.51,0.93*74\r\n";

}  // namespace

// ───────────────────────── checksum / framing ─────────────────────────

TEST(Lc86Checksum, MatchesProtocolDocExamples)
{
    // Every expected value below is printed verbatim in the protocol doc (or,
    // where marked, the hardware design) — these anchor the XOR-between-$-
    // and-* rule (commas included) externally.
    struct { const char* body; uint8_t cs; } cases[] = {
        {"PAIR050,1000",        0x12},  // §2.4.10
        {"PAIR001,004,0",       0x3F},  // §2.4.1
        {"PAIR002",             0x38},  // §2.4.2
        {"PAIR062,0,3",         0x3D},  // §2.4.14
        {"PAIR066,1,1,1,1,0,0", 0x3A},  // §2.4.16
        {"PAIR513",             0x3D},  // §2.4.49
        {"PAIR864,0,0,115200",  0x1B},  // §2.4.69
        {"PAIR732,0",           0x20},  // hardware design V1.5 §3.3.3 (ALP exit)
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

TEST(Lc86Pvt, KeepsGpsTimeOfWeekAndNamesTheWeek)
{
    // The doc's own example: 2023-05-05 07:39:38 UTC + 18 leap seconds is
    // 459596 s into GPS week 2260 (#1600: the time sync needs GPS time).
    Lc86Parser p;
    ASSERT_EQ(count(feedAll(p, kDocPvtLine), Event::PVT), 1);
    EXPECT_EQ(p.pvtTowMs(), 459596000u);
    EXPECT_EQ(p.pvtGpsWeek(), 2260);
}

TEST(Lc86Pvt, TheWeekTurnsWithGpsTimeNotUtc)
{
    // Saturday 2026-08-22 23:59:50 UTC is already 8 s into GPS week 2433
    // (18 leap seconds); 20 s earlier it is still the end of week 2432.
    Lc86Parser p;
    ASSERT_EQ(count(feedAll(p, frame(
        "PQTMPVT,1,8000,20260822,235950.000,1,3,12,18,40.0,-105.0,1000.0,0.0,"
        "0.0,0.0,0.0,0.0,0.0,1.0,1.5")), Event::PVT), 1);
    EXPECT_EQ(p.pvtTowMs(), 8000u);
    EXPECT_EQ(p.pvtGpsWeek(), 2433);
    ASSERT_EQ(count(feedAll(p, frame(
        "PQTMPVT,1,604788000,20260822,235930.000,1,3,12,18,40.0,-105.0,1000.0,0.0,"
        "0.0,0.0,0.0,0.0,0.0,1.0,1.5")), Event::PVT), 1);
    EXPECT_EQ(p.pvtTowMs(), 604788000u);
    EXPECT_EQ(p.pvtGpsWeek(), 2432);
}

TEST(Lc86Pvt, NoDateNoWeek)
{
    // A fixless module sends empty fields: no week, so nothing is synced to it.
    Lc86Parser p;
    feedAll(p, kDocPvtLine);
    ASSERT_EQ(p.pvtGpsWeek(), 2260);
    ASSERT_EQ(count(feedAll(p, frame("PQTMPVT,1,,,,0,0,0,,,,,,,,,,,,")), Event::PVT), 1);
    EXPECT_EQ(p.pvtGpsWeek(), 0);
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

TEST(Lc86Ack, NavModeSetAndReadback)
{
    // begin() sets Balloon with $PAIR080 and waits for this ack: doc
    // §2.4.24's published one, as printed.
    Lc86Parser p;
    const auto set_evs = feedAll(p, "$PAIR001,080,0*33\r\n");
    ASSERT_EQ(count(set_evs, Event::PAIR_ACK), 1);
    EXPECT_EQ(p.ackCommandId(), 80);
    EXPECT_EQ(p.ackResult(), 0);

    // Then reads the mode back: doc §2.4.25's exchange, as printed, the ack
    // and then the answer (0 = Normal). begin() logs it on every boot.
    EXPECT_EQ(p.navMode(), 0xFF);   // nothing read yet
    const auto evs = feedAll(p, "$PAIR001,081,0*32\r\n$PAIR081,0*2F\r\n");
    ASSERT_EQ(count(evs, Event::PAIR_ACK), 1);
    EXPECT_EQ(p.ackCommandId(), 81);
    ASSERT_EQ(count(evs, Event::NAV_MODE), 1);
    EXPECT_EQ(p.navMode(), 0);

    EXPECT_EQ(count(feedAll(p, frame("PAIR081,3")), Event::NAV_MODE), 1);
    EXPECT_EQ(p.navMode(), 3);      // Balloon

    // An empty or out-of-range mode is rejected and leaves the last answer.
    EXPECT_EQ(count(feedAll(p, frame("PAIR081,")), Event::BAD), 1);
    EXPECT_EQ(count(feedAll(p, frame("PAIR081,9")), Event::BAD), 1);
    EXPECT_EQ(p.navMode(), 3);
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

// ───────────────────── satellites in view ($--GSV, #1032) ────────────────────
//
// GSV is the ONLY C/N0 source on this part — no UBX, so no NAV-SAT — and
// #1032's LoRa/GNSS coexistence test is answered by per-constellation C/N0.
// What is pinned here: the talker → gnssId map and the NMEA extended-PRN
// folding (so a mini record is comparable to a V9's NAV-SAT one field for
// field), anchored on the protocol doc's own GSV example, where QZSS rides
// the GP talker; the burst framing (a burst is closed by the next non-GSV
// sentence, and MsgNum 1 retires what that talker contributed); BeiDou
// recorded on B1I only; the itow_ms pairing against the epoch's $PQTMPVT;
// and the truncation order: a satellite with signal is never dropped in
// favour of one without, and among those the lowest go first.

namespace
{

// Pull one satellite out of a record by (gnssId, svId); nullptr if absent.
const GNSSSatBlock* findSat(const GNSSSatData& d, uint8_t gnss_id, uint8_t sv_id)
{
    for (uint8_t i = 0; i < d.num_blocks; i++)
    {
        if (d.sat[i].gnss_id == gnss_id && d.sat[i].sv_id == sv_id) return &d.sat[i];
    }
    return nullptr;
}

// A burst plus the non-GSV sentence that closes it. GGA is what the driver
// actually leaves enabled alongside PQTMPVT, so this is the real shape.
const char* kCloser = "GPGGA,073938.000,3149.30591,N,11706.93450,E,1,31,0.5,"
                      "100.0,M,0.0,M,,";

}  // namespace

TEST(Lc86Gsv, ParsesOneGpsBurstAndClosesOnNextSentence)
{
    Lc86Parser p;
    GNSSSatData sat = {};

    // Two-sentence GPS set: 4 satellites then 3, the last with a trailing
    // NMEA 4.10 <SignalID> field that must NOT be read as a fifth block.
    const auto evs = feedAll(p,
        frame("GPGSV,2,1,07,01,40,083,42,02,17,308,37,03,07,344,00,04,22,228,41") +
        frame("GPGSV,2,2,07,05,66,012,45,06,32,147,33,07,11,199,00,1"));

    EXPECT_EQ(count(evs, Event::GSV), 2);
    EXPECT_EQ(p.gsvSentences(), 2u);
    // Still open: nothing has closed the burst yet.
    EXPECT_FALSE(p.takeSat(sat));

    feedAll(p, frame(kCloser));
    ASSERT_TRUE(p.takeSat(sat));

    EXPECT_EQ(sat.num_svs, 7);
    EXPECT_EQ(sat.num_blocks, 7);

    const GNSSSatBlock* s1 = findSat(sat, 0, 1);
    ASSERT_NE(s1, nullptr);
    EXPECT_EQ(s1->cno_dbhz, 42);
    EXPECT_EQ(s1->elev_deg, 40);
    EXPECT_EQ(s1->azim_2deg, 83 / 2);    // azimuth is stored in 2-degree steps
    EXPECT_EQ(s1->flags, 0);             // GSV reports none of the flag bits

    // An empty-signal satellite is still in the table, at cno 0.
    const GNSSSatBlock* s3 = findSat(sat, 0, 3);
    ASSERT_NE(s3, nullptr);
    EXPECT_EQ(s3->cno_dbhz, 0);

    // The <SignalID> on the second sentence produced no phantom satellite.
    EXPECT_EQ(findSat(sat, 0, 1), s1);
    for (uint8_t i = 0; i < sat.num_blocks; i++) EXPECT_GT(sat.sat[i].sv_id, 0);

    // Consumed: a second take finds nothing until the next burst.
    GNSSSatData again = {};
    EXPECT_FALSE(p.takeSat(again));
}

TEST(Lc86Gsv, TalkerIdsMapToUbxConstellationsAndFoldExtendedPrns)
{
    Lc86Parser p;
    GNSSSatData sat = {};

    feedAll(p,
        // SBAS rides INSIDE the GP set — a receiver emits one set per talker
        // per burst, so PRN 10 (GPS) and PRN 40 (SBAS) share this sentence.
        frame("GPGSV,1,1,02,10,45,100,40,40,20,200,35") +
        frame("GLGSV,1,1,01,70,30,050,38") +        // GLONASS, extended PRN
        frame("GAGSV,1,1,01,12,55,120,44") +        // Galileo
        frame("GBGSV,1,1,01,210,25,300,31") +       // BeiDou, extended PRN
        frame("GQGSV,1,1,01,195,60,090,36") +       // QZSS, extended PRN
        frame("GIGSV,1,1,01,04,35,270,29") +        // NavIC
        frame("XXGSV,1,1,01,09,15,015,22") +        // unknown talker: kept, id 255
        frame(kCloser));
    ASSERT_TRUE(p.takeSat(sat));

    EXPECT_EQ(sat.num_blocks, 8);
    EXPECT_NE(findSat(sat, 0, 10), nullptr);   // GPS PRN passes through
    ASSERT_NE(findSat(sat, 1, 127), nullptr);  // SBAS 40 → gnssId 1, svId 40+87
    EXPECT_EQ(findSat(sat, 1, 127)->cno_dbhz, 35);
    EXPECT_NE(findSat(sat, 6, 6), nullptr);    // GLONASS 70 → slot 6
    EXPECT_NE(findSat(sat, 2, 12), nullptr);   // Galileo passes through
    EXPECT_NE(findSat(sat, 3, 10), nullptr);   // BeiDou 210 → 10
    EXPECT_NE(findSat(sat, 5, 3), nullptr);    // QZSS 195 → 3
    EXPECT_NE(findSat(sat, 7, 4), nullptr);    // NavIC passes through
    EXPECT_NE(findSat(sat, 255, 9), nullptr);  // unknown talker recorded, not dropped
}

TEST(Lc86Gsv, MsgNumOneRetiresThatConstellationsStaleEntries)
{
    Lc86Parser p;
    GNSSSatData sat = {};

    // First burst: two GPS satellites and one Galileo.
    feedAll(p, frame("GPGSV,1,1,02,01,40,083,42,02,17,308,37") +
               frame("GAGSV,1,1,01,12,55,120,44") +
               frame(kCloser));
    ASSERT_TRUE(p.takeSat(sat));
    EXPECT_EQ(sat.num_blocks, 3);

    // Second burst: GPS 02 has set. The GPS set must be replaced wholesale,
    // and Galileo — which did not report again — must not be dragged along
    // from the previous burst either.
    feedAll(p, frame("GPGSV,1,1,01,01,41,084,43") + frame(kCloser));
    ASSERT_TRUE(p.takeSat(sat));
    EXPECT_EQ(sat.num_blocks, 1);
    ASSERT_NE(findSat(sat, 0, 1), nullptr);
    EXPECT_EQ(findSat(sat, 0, 1)->cno_dbhz, 43);
    EXPECT_EQ(findSat(sat, 0, 2), nullptr);
    EXPECT_EQ(findSat(sat, 2, 12), nullptr);
}

TEST(Lc86Gsv, RecordCarriesTheEpochTowSoItPairsWithThePvt)
{
    Lc86Parser p;
    GNSSSatData sat = {};

    // No PVT yet → the pairing key is 0, honestly.
    feedAll(p, frame("GPGSV,1,1,01,01,40,083,42") + frame(kCloser));
    ASSERT_TRUE(p.takeSat(sat));
    EXPECT_EQ(sat.itow_ms, 0u);
    EXPECT_EQ(sat.time_us, 0u);   // the owner stamps MCU time, not the parser

    // The doc's own PVT example carries TOW 459596000 ms.
    feedAll(p, std::string(kDocPvtLine));
    feedAll(p, frame("GPGSV,1,1,01,01,40,083,42") + frame(kCloser));
    ASSERT_TRUE(p.takeSat(sat));
    EXPECT_EQ(sat.itow_ms, 459596000u);
}

TEST(Lc86Gsv, TruncationDropsOnlySatellitesWithNoSignal)
{
    Lc86Parser p;
    GNSSSatData sat = {};
    std::string burst;

    // 40 satellites in view — the build table's capacity and well over the
    // 32 the record carries. Every EVEN PRN is being searched for (cno 0).
    // gnssSatSelect must keep all 20 tracked ones and fill the remaining 12
    // slots from the silent ones.
    for (int i = 0; i < 10; i++)
    {
        char body[128];
        const int base = i * 4 + 1;
        snprintf(body, sizeof(body),
                 "GPGSV,10,%d,40,%02d,10,020,%02d,%02d,10,020,%02d,"
                 "%02d,10,020,%02d,%02d,10,020,%02d",
                 i + 1,
                 base,     (base     % 2) ? 40 : 0,
                 base + 1, ((base + 1) % 2) ? 40 : 0,
                 base + 2, ((base + 2) % 2) ? 40 : 0,
                 base + 3, ((base + 3) % 2) ? 40 : 0);
        burst += frame(body);
    }
    feedAll(p, burst + frame(kCloser));
    ASSERT_TRUE(p.takeSat(sat));

    EXPECT_EQ(p.gsvOverflows(), 0u);     // 40 fits the build table exactly
    EXPECT_EQ(sat.num_svs, 40);          // what the receiver showed
    EXPECT_EQ(sat.num_blocks, 32);       // what the record carries

    int tracked = 0;
    for (uint8_t i = 0; i < sat.num_blocks; i++)
    {
        if (sat.sat[i].cno_dbhz > 0) tracked++;
    }
    EXPECT_EQ(tracked, 20) << "every satellite with signal must survive truncation";

    // And they come first, which is the property gnssSatSelect promises.
    bool seen_silent = false;
    for (uint8_t i = 0; i < sat.num_blocks; i++)
    {
        if (sat.sat[i].cno_dbhz == 0) seen_silent = true;
        else EXPECT_FALSE(seen_silent) << "tracked entry after a silent one at " << (int)i;
    }
}

TEST(Lc86Gsv, MalformedSentencesAreRejectedWithoutPoisoningTheTable)
{
    Lc86Parser p;
    GNSSSatData sat = {};

    const auto evs = feedAll(p,
        frame("GPGSV,1,1,01,01,40,083,42") +
        frame("GPGSV,0,1,01,02,40,083,42") +   // NumMsg 0 is impossible
        frame("GPGSV,2,3,01,03,40,083,42") +   // MsgNum past NumMsg
        frame("GPGSV,1") +                     // too few fields
        frame("GPGSV,1,1,01,09,15,015,22"));   // valid, different set start

    EXPECT_EQ(count(evs, Event::BAD), 3);
    EXPECT_EQ(p.gsvSentences(), 2u);

    feedAll(p, frame(kCloser));
    ASSERT_TRUE(p.takeSat(sat));
    // The last valid sentence was a MsgNum 1, so it retired PRN 01's set:
    // only PRN 09 survives, and none of the rejected PRNs ever landed.
    EXPECT_EQ(sat.num_blocks, 1);
    EXPECT_NE(findSat(sat, 0, 9), nullptr);
    EXPECT_EQ(findSat(sat, 0, 2), nullptr);
    EXPECT_EQ(findSat(sat, 0, 3), nullptr);
}

TEST(Lc86Gsv, BurstSurvivesBeingSplitAcrossFeeds)
{
    // pollNewPVT drains bounded chunks, so a GSV burst routinely straddles
    // calls exactly as PVT does.
    Lc86Parser p;
    GNSSSatData sat = {};
    const std::string stream =
        frame("GPGSV,2,1,05,01,40,083,42,02,17,308,37,03,07,344,00,04,22,228,41") +
        frame("GPGSV,2,2,05,05,66,012,45") +
        frame(kCloser);

    for (size_t i = 0; i < stream.size(); i++) p.feed((uint8_t)stream[i]);
    ASSERT_TRUE(p.takeSat(sat));
    EXPECT_EQ(sat.num_blocks, 5);
    EXPECT_EQ(p.badLines(), 0u);
}

namespace
{

// The protocol doc's own published GSV example (§2.2.3), byte for byte with
// its printed checksums, so the talker map and the PRN folding are anchored
// against the DOCUMENT, as the $PQTMPVT example is above. It was captured on
// an LC76G (AB), which tracks the same signals as the LC86G (LA) (Table 1).
// Two things in it that a generic NMEA table gets wrong: QZSS rides the GP
// talker (Table 4) as PRNs 193-199 (Table 13), so 195, 194, 199 and 196 below
// are QZSS, not GPS; and BeiDou is numbered 1-63 as-is.
const char* kDocGsvBurst =
    "$GPGSV,3,1,12,195,72,076,42,01,69,158,45,194,66,111,29,21,61,060,44,1*6D\r\n"
    "$GPGSV,3,2,12,07,61,233,42,30,52,284,44,199,51,162,37,08,39,045,42,1*59\r\n"
    "$GPGSV,3,3,12,14,29,312,29,196,20,148,36,17,18,258,36,27,07,061,36,1*53\r\n"
    "$GLGSV,2,1,05,79,80,068,47,82,62,248,44,81,56,014,38,78,31,137,24,1*7F\r\n"
    "$GLGSV,2,2,05,88,07,034,29,1*46\r\n"
    "$GAGSV,2,1,06,26,80,095,42,01,69,353,13,21,49,106,26,33,42,207,41,7*72\r\n"
    "$GAGSV,2,2,06,13,28,040,34,31,19,313,34,7*72\r\n"
    "$GBGSV,4,1,16,46,81,194,38,07,68,349,31,40,61,016,40,30,60,259,43,1*71\r\n"
    "$GBGSV,4,2,16,10,59,321,,03,51,192,36,36,41,314,38,02,37,229,32,1*71\r\n"
    "$GBGSV,4,3,16,09,31,219,26,08,27,175,31,37,25,146,29,06,23,202,29,1*78\r\n"
    "$GBGSV,4,4,16,16,20,199,31,13,17,186,26,39,12,192,29,28,09,048,30,1*7C\r\n";

}  // namespace

TEST(Lc86Gsv, DocExampleBurstPutsQzssOffTheGpTalker)
{
    Lc86Parser p;
    GNSSSatData sat = {};

    const auto evs = feedAll(p, std::string(kDocGsvBurst) + frame(kCloser));
    EXPECT_EQ(count(evs, Event::GSV), 11);
    EXPECT_EQ(p.badLines(), 0u);              // every printed checksum verifies
    EXPECT_EQ(p.gsvSkipped(), 0u);            // B1I only: nothing to leave out
    ASSERT_TRUE(p.takeSat(sat));

    // QZSS lands as gnssId 5, svId = PRN - 192: what a u-blox NAV-SAT says
    // for the same satellite. Never GPS with an svId of 195.
    ASSERT_NE(findSat(sat, 5, 3), nullptr);   // PRN 195
    EXPECT_EQ(findSat(sat, 5, 3)->cno_dbhz, 42);
    EXPECT_EQ(findSat(sat, 5, 3)->elev_deg, 72);
    EXPECT_NE(findSat(sat, 5, 2), nullptr);   // PRN 194
    EXPECT_NE(findSat(sat, 5, 7), nullptr);   // PRN 199
    EXPECT_NE(findSat(sat, 5, 4), nullptr);   // PRN 196
    for (uint8_t i = 0; i < sat.num_blocks; i++)
    {
        if (sat.sat[i].gnss_id == 0)
        {
            EXPECT_LE(sat.sat[i].sv_id, 32) << "a GPS entry with svId "
                                            << (int)sat.sat[i].sv_id;
        }
    }

    // One satellite from each of the other constellations.
    ASSERT_NE(findSat(sat, 0, 1), nullptr);   // GPS 01
    EXPECT_EQ(findSat(sat, 0, 1)->cno_dbhz, 45);
    ASSERT_NE(findSat(sat, 6, 15), nullptr);  // GLONASS 79 -> slot 15
    EXPECT_EQ(findSat(sat, 6, 15)->cno_dbhz, 47);
    ASSERT_NE(findSat(sat, 2, 26), nullptr);  // Galileo 26
    EXPECT_EQ(findSat(sat, 2, 26)->elev_deg, 80);
    ASSERT_NE(findSat(sat, 3, 46), nullptr);  // BeiDou 46, numbered as-is
    EXPECT_EQ(findSat(sat, 3, 46)->cno_dbhz, 38);

    // 39 in view and 38 with signal, on the doc's own sample sky: more than
    // the record's 32 slots. The cut takes the six LOWEST satellites with
    // signal, wherever they are listed: GPS 27 (7 deg), GLONASS 88 (7),
    // BeiDou 28 (9), 39 (12) and 13 (17), GPS 17 (18). Galileo 31 at 19 deg
    // is the lowest one kept. Cut in list order instead, all six would have
    // come off BeiDou, the talker listed last.
    EXPECT_EQ(sat.num_svs, 39);
    EXPECT_EQ(sat.num_blocks, GNSS_SAT_MAX_BLOCKS);
    int per_gnss[8] = {};
    for (uint8_t i = 0; i < sat.num_blocks; i++)
    {
        EXPECT_GT(sat.sat[i].cno_dbhz, 0);
        EXPECT_GE(sat.sat[i].elev_deg, 19);
        if (sat.sat[i].gnss_id < 8) per_gnss[sat.sat[i].gnss_id]++;
    }
    EXPECT_EQ(findSat(sat, 0, 27), nullptr);
    EXPECT_EQ(findSat(sat, 6, 24), nullptr);   // GLONASS 88 -> slot 24
    EXPECT_EQ(findSat(sat, 3, 28), nullptr);
    EXPECT_EQ(findSat(sat, 3, 39), nullptr);
    EXPECT_EQ(findSat(sat, 3, 13), nullptr);
    EXPECT_EQ(findSat(sat, 0, 17), nullptr);
    EXPECT_NE(findSat(sat, 2, 31), nullptr);
    EXPECT_EQ(per_gnss[0], 6);    // GPS: 8 in view
    EXPECT_EQ(per_gnss[5], 4);    // QZSS: 4
    EXPECT_EQ(per_gnss[6], 4);    // GLONASS: 5
    EXPECT_EQ(per_gnss[2], 6);    // Galileo: 6
    EXPECT_EQ(per_gnss[3], 12);   // BeiDou: 15 with signal
}

TEST(Lc86Gsv, TruncationTakesTheLowestSatellitesNotTheLastTalker)
{
    // 32 GPS satellites with signal at 11..42 degrees, then BeiDou: one with
    // signal at 80 degrees and one being searched for at 85. Thirty-three
    // have signal, so one must go: the lowest GPS satellite, not the BeiDou
    // one listed last. And the silent BeiDou entry, although it is the
    // highest of all, still loses to every satellite with signal.
    Lc86Parser p;
    GNSSSatData sat = {};
    std::string burst;
    for (int s = 0; s < 8; s++)
    {
        char body[128];
        const int prn = s * 4 + 1;
        snprintf(body, sizeof(body),
                 "GPGSV,8,%d,32,%02d,%d,100,40,%02d,%d,100,40,%02d,%d,100,40,"
                 "%02d,%d,100,40,1",
                 s + 1, prn, 10 + prn, prn + 1, 11 + prn, prn + 2, 12 + prn,
                 prn + 3, 13 + prn);
        burst += frame(body);
    }
    burst += frame("GBGSV,1,1,02,05,80,200,35,06,85,210,,1");
    feedAll(p, burst + frame(kCloser));
    ASSERT_TRUE(p.takeSat(sat));

    EXPECT_EQ(sat.num_svs, 34);
    EXPECT_EQ(sat.num_blocks, GNSS_SAT_MAX_BLOCKS);
    EXPECT_EQ(findSat(sat, 0, 1), nullptr);    // GPS 01 at 11 deg: the cut
    EXPECT_NE(findSat(sat, 0, 2), nullptr);    // GPS 02 at 12 deg stays
    EXPECT_NE(findSat(sat, 3, 5), nullptr);    // BeiDou, listed last, stays
    EXPECT_EQ(findSat(sat, 3, 6), nullptr);    // silent, however high

    // Highest first, which is what made the cut land on the lowest.
    EXPECT_EQ(sat.sat[0].gnss_id, 3);
    EXPECT_EQ(sat.sat[0].sv_id, 5);
    for (uint8_t i = 1; i < sat.num_blocks; i++)
    {
        EXPECT_LE(sat.sat[i].elev_deg, sat.sat[i - 1].elev_deg) << "at " << (int)i;
    }
}

TEST(Lc86Gsv, RealBeetleBurstParsesAsTheModuleSentIt)
{
    // One epoch captured 2026-09-24 from the Beetle's own LC86G (firmware
    // LC86GLANR12A03S, the flight configuration at 10 Hz) on the SDR rig,
    // byte for byte. The rig simulates GPS only, hence the three empty sets,
    // which the module sends for every constellation it has nothing in. It
    // also shows three satellites tracked before their elevation is known
    // (21, 12, 11: empty elevation and azimuth, C/N0 44) and SBAS PRN 46 on
    // the GP talker, predicted but not tracked. The burst follows the
    // epoch's $PQTMPVT and is closed by its $PQTMEPE.
    Lc86Parser p;
    GNSSSatData sat = {};
    const auto evs = feedAll(p,
        "$PQTMPVT,1,203446300,20260818,083028.300,1,3,9,18,0.00004400,"
        "-119.00005050,1199.039,-22.916,-0.004,-0.002,-0.009,0.005,0.00,"
        "0.76,1.15*7A\r\n"
        "$GPGSV,4,1,13,21,,,44,24,51,303,44,06,32,029,43,05,28,183,43,1*5B\r\n"
        "$GPGSV,4,2,13,22,26,079,44,15,20,230,44,14,17,100,44,19,13,025,44,1*62\r\n"
        "$GPGSV,4,3,13,30,12,147,43,29,05,259,44,12,,,44,11,,,44,1*67\r\n"
        "$GPGSV,4,4,13,46,78,269,,1*56\r\n"
        "$GLGSV,1,1,00,1*78\r\n"
        "$GAGSV,1,1,00,7*73\r\n"
        "$GBGSV,1,1,00,1*76\r\n"
        "$PQTMEPE,2,1.593,1.295,3.759,2.053,4.283*54\r\n");
    EXPECT_EQ(p.badLines(), 0u);
    EXPECT_EQ(count(evs, Event::GSV), 7);
    EXPECT_EQ(p.gsvBursts(), 1u);
    ASSERT_TRUE(p.takeSat(sat));

    EXPECT_EQ(sat.itow_ms, 203446300u);     // pairs with the $PQTMPVT above
    EXPECT_EQ(sat.num_svs, 13);
    EXPECT_EQ(sat.num_blocks, 13);
    ASSERT_NE(findSat(sat, 0, 21), nullptr);
    EXPECT_EQ(findSat(sat, 0, 21)->cno_dbhz, 44);
    EXPECT_EQ(findSat(sat, 0, 21)->elev_deg, 0);   // not known yet: lands as 0
    ASSERT_NE(findSat(sat, 1, 133), nullptr);      // SBAS 46 -> svId 133
    EXPECT_EQ(findSat(sat, 1, 133)->cno_dbhz, 0);
    EXPECT_EQ(findSat(sat, 1, 133)->elev_deg, 78);

    // Tracked satellites highest first; the three with no elevation yet sit
    // with the horizon, in the order the module sent them; the untracked
    // SBAS entry comes last, however high it is.
    const uint8_t want_sv[13] = {24, 6, 5, 22, 15, 14, 19, 30, 29, 21, 12, 11, 133};
    for (uint8_t i = 0; i < 13; i++)
    {
        EXPECT_EQ(sat.sat[i].sv_id, want_sv[i]) << "at " << (int)i;
    }
}

TEST(Lc86Gsv, EmptyBurstIsCountedButMakesNoRecord)
{
    // gsvBursts() is the GSV rate the driver reports at boot, so a burst with
    // no satellites in it (a board indoors) must count, without producing a
    // record that says nothing.
    Lc86Parser p;
    GNSSSatData sat = {};

    feedAll(p, frame("GPGSV,1,1,00,1") + frame("GLGSV,1,1,00,1") + frame(kCloser));
    EXPECT_EQ(p.gsvBursts(), 1u);
    EXPECT_EQ(p.gsvBurstsWithSats(), 0u);
    EXPECT_FALSE(p.takeSat(sat));

    feedAll(p, frame("GPGSV,1,1,01,01,40,083,42,1") + frame(kCloser));
    EXPECT_EQ(p.gsvBursts(), 2u);
    EXPECT_EQ(p.gsvBurstsWithSats(), 1u);   // counted at the close, not the take
    EXPECT_TRUE(p.takeSat(sat));

    // A closer with no burst open is not a burst.
    feedAll(p, frame(kCloser));
    EXPECT_EQ(p.gsvBursts(), 2u);
    EXPECT_EQ(p.gsvBurstsWithSats(), 1u);
}

TEST(Lc86Gsv, BeiDouB1cSetNeverReplacesTheB1iOne)
{
    // The LC86G (LA) tracks BeiDou on B1I and B1C (Table 1), and GSV reports
    // each signal as its own set (Table 13: SignalID 1 = B1I, 3 = B1C), each
    // starting at MsgNum 1. The record has no signal field, so it carries
    // B1I, the 1561 MHz band #1032's crystal harmonic lands in. A B1C set
    // must neither replace it nor mix into it, whichever set comes first.
    for (int b1c_first = 0; b1c_first < 2; b1c_first++)
    {
        SCOPED_TRACE(b1c_first ? "B1C set first" : "B1I set first");
        Lc86Parser p;
        GNSSSatData sat = {};
        const std::string b1i = frame("GBGSV,1,1,02,19,60,100,40,20,45,200,38,1");
        const std::string b1c = frame("GBGSV,1,1,02,19,60,100,31,20,45,200,29,3");
        feedAll(p, frame("GPGSV,1,1,01,01,40,083,42,1") +
                   (b1c_first ? b1c + b1i : b1i + b1c) +
                   frame(kCloser));
        ASSERT_TRUE(p.takeSat(sat));

        EXPECT_EQ(sat.num_blocks, 3);
        ASSERT_NE(findSat(sat, 3, 19), nullptr);
        EXPECT_EQ(findSat(sat, 3, 19)->cno_dbhz, 40);   // B1I's, not B1C's 31
        ASSERT_NE(findSat(sat, 3, 20), nullptr);
        EXPECT_EQ(findSat(sat, 3, 20)->cno_dbhz, 38);
        EXPECT_EQ(p.gsvSkipped(), 1u);     // the B1C sentence, counted
        EXPECT_EQ(p.gsvSentences(), 2u);   // GP and B1I folded in
    }
}

TEST(Lc86Gsv, MsgNumOneRetiresEverythingItsTalkerContributed)
{
    // GP carries GPS, SBAS and QZSS. A GP set that starts again inside one
    // burst retires all three, because the key is the talker that supplied an
    // entry, not the gnssId its PRN folded to. Other talkers are untouched.
    Lc86Parser p;
    GNSSSatData sat = {};
    feedAll(p, frame("GPGSV,1,1,03,01,40,083,42,40,20,200,35,195,60,090,36,1") +
               frame("GAGSV,1,1,01,12,55,120,44,7") +
               frame("GPGSV,1,1,01,01,41,084,43,1") +
               frame(kCloser));
    ASSERT_TRUE(p.takeSat(sat));

    EXPECT_EQ(sat.num_blocks, 2);
    ASSERT_NE(findSat(sat, 0, 1), nullptr);
    EXPECT_EQ(findSat(sat, 0, 1)->cno_dbhz, 43);
    EXPECT_EQ(findSat(sat, 1, 127), nullptr);  // SBAS went with its set
    EXPECT_EQ(findSat(sat, 5, 3), nullptr);    // and so did QZSS
    EXPECT_NE(findSat(sat, 2, 12), nullptr);   // Galileo stays
}

TEST(Lc86Gsv, SatRecordPackingMatchesTheWire)
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

TEST(Lc86Msm7, RtcmModeReadback)
{
    // $PAIR433,<Mode> answers a $PAIR433 query: -1 off, 0 MSM4, 1 MSM7.
    Lc86Parser p;
    EXPECT_EQ(p.rtcmMode(), -2);   // nothing heard yet
    auto evs = feedAll(p, frame("PAIR433,1"));
    EXPECT_EQ(count(evs, Event::RTCM_MODE), 1);
    EXPECT_EQ(p.rtcmMode(), 1);
    evs = feedAll(p, frame("PAIR433,-1"));
    EXPECT_EQ(p.rtcmMode(), -1);
    evs = feedAll(p, frame("PAIR433,7") + frame("PAIR433,"));
    EXPECT_EQ(count(evs, Event::BAD), 2);
    EXPECT_EQ(p.rtcmMode(), -1);   // a bad answer changes nothing
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

TEST(Lc86Msm7, RtcmEphemerisModeReadback)
{
    // $PAIR437,<Enable> answers a $PAIR437 query: 0 off, 1 on
    // (spec V1.5 §2.4.48's own example is "$PAIR437,1*27").
    Lc86Parser p;
    EXPECT_EQ(p.rtcmEphMode(), -2);
    auto evs = feedAll(p, "$PAIR437,1*27\r\n");
    EXPECT_EQ(count(evs, Event::RTCM_EPH_MODE), 1);
    EXPECT_EQ(p.rtcmEphMode(), 1);
    evs = feedAll(p, frame("PAIR437,2"));
    EXPECT_EQ(count(evs, Event::BAD), 1);
    EXPECT_EQ(p.rtcmEphMode(), 1);
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
