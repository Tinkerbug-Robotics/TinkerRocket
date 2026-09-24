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
    EXPECT_FALSE(p.takeSat(sat));

    feedAll(p, frame("GPGSV,1,1,01,01,40,083,42,1") + frame(kCloser));
    EXPECT_EQ(p.gsvBursts(), 2u);
    EXPECT_TRUE(p.takeSat(sat));

    // A closer with no burst open is not a burst.
    feedAll(p, frame(kCloser));
    EXPECT_EQ(p.gsvBursts(), 2u);
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
