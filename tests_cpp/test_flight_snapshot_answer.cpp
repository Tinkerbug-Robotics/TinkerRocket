// The OC's answer to the FC's boot-time GET_FLIGHT_SNAPSHOT.
//
// flight_snapshot_answer.h is how the FC reads that answer (and what "no
// flight" looks like on the wire); snapshot_answer_policy.h is when the OC may
// say "no flight" instead of serving a frame or staying silent.
//
// The bug behind both: an OC holding no flight said nothing. The FC's read
// then returned the OC's last staged I2C reply, a status frame
// (AA 55 AA 55 A9 ...), which is not an answer, so every FC boot spent all
// five tries (about 1.5 s) and logged five "[RECOVERY] read OK but no
// SNAPSHOT frame" warnings. Seen on the V8 bench on 2026-10-17.

#include <gtest/gtest.h>

#include <CRC.h>
#include <CRC32.h>

#include <cstring>
#include <vector>

#include "flight_snapshot_answer.h"
#include "snapshot_answer_policy.h"

namespace fsa = flight_snapshot_answer;
using SnapshotAnswerPolicy::Reply;
using SnapshotAnswerPolicy::decide;

namespace {

// Both ends' snapshot CRC: CRC32 over the bytes in front of crc32
// (flight_computer computeSnapshotCRC, out_computer snapshotCrc).
uint32_t snapshotCrc(const FlightSnapshotData& s)
{
    CRC32 crc;
    crc.add(reinterpret_cast<const uint8_t*>(&s), fsa::kCrcBytes);
    return crc.calc();
}

FlightSnapshotData stamped(FlightSnapshotData s)
{
    s.crc32 = snapshotCrc(s);
    return s;
}

FlightSnapshotData snapshotIn(uint8_t state, bool sim = false)
{
    FlightSnapshotData s = {};
    s.magic             = FlightSnapshotData::MAGIC;
    s.version           = FlightSnapshotData::VERSION;
    s.rocket_state      = state;
    s.sim_flight        = sim ? 1 : 0;
    s.flight_elapsed_ms = 12345;
    return stamped(s);
}

fsa::Verdict verdictOf(const FlightSnapshotData& s)
{
    return fsa::classify(s, s.crc32 == snapshotCrc(s));
}

// The wire frame as TR_I2C_Interface::packMessage builds it (mirrored, as in
// test_snapshot_tail_scan and wire_fixtures_lib): SOF, type, len, payload,
// CRC16 over type..payload.
std::vector<uint8_t> wireFrame(uint8_t type, const void* payload, size_t len)
{
    std::vector<uint8_t> f{0xAA, 0x55, 0xAA, 0x55, type, static_cast<uint8_t>(len)};
    const auto* p = static_cast<const uint8_t*>(payload);
    f.insert(f.end(), p, p + len);
    const uint16_t crc = calcCRC16(f.data() + 4, static_cast<int>(2 + len));
    f.push_back((crc >> 8) & 0xFF);
    f.push_back(crc & 0xFF);
    return f;
}

// The FC's decision before classify() existed, copied from
// fcAskOcForFlightSnapshot as it stood, so the grid below can pin that the
// shared classifier makes the same call everywhere.
struct FcDecision
{
    bool valid;
    bool definitive;
    bool answered_no_flight;
};

FcDecision previousFcDecision(const FlightSnapshotData& snap, bool crc_ok)
{
    FcDecision d = {false, false, false};
    if (snap.magic == FlightSnapshotData::MAGIC &&
        snap.version == FlightSnapshotData::VERSION &&
        snap.rocket_state == (uint8_t)INFLIGHT && crc_ok)
    {
        if (!snap.sim_flight) d.valid = true;
        if (!d.valid) d.answered_no_flight = true;
        d.definitive = true;
    }
    else if (snap.magic == FlightSnapshotData::MAGIC && crc_ok)
    {
        d.definitive = true;
        d.answered_no_flight = true;
    }
    return d;
}

FcDecision decisionFrom(fsa::Verdict v)
{
    return {v == fsa::Verdict::Restore, fsa::definitive(v),
            v == fsa::Verdict::SimFlight || v == fsa::Verdict::NoFlight};
}

}  // namespace

// ---------------------------------------------------------------------------
// How the FC reads the answer
// ---------------------------------------------------------------------------

TEST(FlightSnapshotAnswer, ALiveInflightSnapshotRestores)
{
    EXPECT_EQ(verdictOf(snapshotIn(INFLIGHT)), fsa::Verdict::Restore);
    EXPECT_TRUE(fsa::definitive(fsa::Verdict::Restore));
}

TEST(FlightSnapshotAnswer, ASimulatedFlightIsRefusedButDefinitive)
{
    EXPECT_EQ(verdictOf(snapshotIn(INFLIGHT, /*sim=*/true)), fsa::Verdict::SimFlight);
    EXPECT_TRUE(fsa::definitive(fsa::Verdict::SimFlight));
}

TEST(FlightSnapshotAnswer, TheOcsNoFlightAnswerEndsTheAskAtOnce)
{
    const FlightSnapshotData none = stamped(fsa::noFlightPayload());
    EXPECT_EQ(verdictOf(none), fsa::Verdict::NoFlight);
    EXPECT_TRUE(fsa::definitive(verdictOf(none)));
    EXPECT_TRUE(fsa::isNoFlightAnswer(none, true));
    // It must never read as a flight to restore, whatever its other fields.
    EXPECT_NE(verdictOf(none), fsa::Verdict::Restore);
    FlightSnapshotData inflight_none = fsa::noFlightPayload();
    inflight_none.rocket_state = INFLIGHT;
    inflight_none = stamped(inflight_none);
    EXPECT_EQ(verdictOf(inflight_none), fsa::Verdict::NoFlight);
}

TEST(FlightSnapshotAnswer, AnFcFromBeforeTheAnswerReadsItAsGarbage)
{
    // So it keeps its five tries and never sends the boot clear (whose uptime
    // elapsed the OC's replay filter would hold against the next flight),
    // exactly as when the OC said nothing.
    const FlightSnapshotData none = stamped(fsa::noFlightPayload());
    const FcDecision old = previousFcDecision(none, none.crc32 == snapshotCrc(none));
    EXPECT_FALSE(old.valid);
    EXPECT_FALSE(old.definitive);
    EXPECT_FALSE(old.answered_no_flight);
}

TEST(FlightSnapshotAnswer, ATornNoFlightAnswerIsGarbage)
{
    FlightSnapshotData none = stamped(fsa::noFlightPayload());
    none.rocket_state ^= 0x7;   // after the CRC was stamped
    EXPECT_EQ(verdictOf(none), fsa::Verdict::Garbage);
    EXPECT_FALSE(fsa::isNoFlightAnswer(none, none.crc32 == snapshotCrc(none)));
}

TEST(FlightSnapshotAnswer, TheFcsOwnLandedClearIsNoFlight)
{
    EXPECT_EQ(verdictOf(snapshotIn(LANDED)), fsa::Verdict::NoFlight);
}

TEST(FlightSnapshotAnswer, EveryOtherStateIsNoFlight)
{
    for (uint8_t st : {(uint8_t)INITIALIZATION, (uint8_t)READY, (uint8_t)PRELAUNCH,
                       (uint8_t)LANDED, (uint8_t)MAG_CALIBRATION})
    {
        EXPECT_EQ(verdictOf(snapshotIn(st)), fsa::Verdict::NoFlight) << "state " << int(st);
    }
}

TEST(FlightSnapshotAnswer, AnotherVersionsInflightFrameIsNoFlightNotARestore)
{
    FlightSnapshotData s = snapshotIn(INFLIGHT);
    s.version = FlightSnapshotData::VERSION + 1;
    s = stamped(s);
    EXPECT_EQ(verdictOf(s), fsa::Verdict::NoFlight);
}

TEST(FlightSnapshotAnswer, ABadCrcOrMagicIsGarbageAndTheFcAsksAgain)
{
    FlightSnapshotData torn = snapshotIn(INFLIGHT);
    torn.flight_elapsed_ms ^= 1;   // after the CRC was stamped
    EXPECT_EQ(verdictOf(torn), fsa::Verdict::Garbage);
    EXPECT_FALSE(fsa::definitive(fsa::Verdict::Garbage));

    FlightSnapshotData other = snapshotIn(LANDED);
    other.magic ^= 0x1;
    other = stamped(other);
    EXPECT_EQ(verdictOf(other), fsa::Verdict::Garbage);

    // An all-zero payload (a retired MRAM slot, a read of nothing) is garbage
    // too, never "no flight": its magic is wrong.
    const FlightSnapshotData zero = {};
    EXPECT_EQ(verdictOf(zero), fsa::Verdict::Garbage);
}

TEST(FlightSnapshotAnswer, ClassifyMakesTheFcsPreviousCallEverywhere)
{
    // Every frame but the OC's new no-flight answer, whose own magic is the
    // one deliberate difference (tested above).
    const uint32_t magics[]  = {FlightSnapshotData::MAGIC, FlightSnapshotData::MAGIC ^ 1u, 0u};
    const uint8_t versions[] = {FlightSnapshotData::VERSION,
                                (uint8_t)(FlightSnapshotData::VERSION + 1), 0};
    int cases = 0;
    for (uint32_t magic : magics)
        for (uint8_t version : versions)
            for (uint8_t state = 0; state <= (uint8_t)MAG_CALIBRATION + 1; ++state)
                for (uint8_t sim : {0, 1})
                    for (bool crc_ok : {true, false})
                    {
                        FlightSnapshotData s = {};
                        s.magic = magic;
                        s.version = version;
                        s.rocket_state = state;
                        s.sim_flight = sim;
                        const FcDecision was = previousFcDecision(s, crc_ok);
                        const FcDecision now = decisionFrom(fsa::classify(s, crc_ok));
                        SCOPED_TRACE(::testing::Message()
                                     << "magic " << std::hex << magic << std::dec
                                     << " version " << int(version) << " state "
                                     << int(state) << " sim " << int(sim)
                                     << " crc_ok " << crc_ok);
                        EXPECT_EQ(now.valid, was.valid);
                        EXPECT_EQ(now.definitive, was.definitive);
                        EXPECT_EQ(now.answered_no_flight, was.answered_no_flight);
                        ++cases;
                    }
    EXPECT_EQ(cases, 3 * 3 * 7 * 2 * 2);
}

TEST(FlightSnapshotAnswer, OnlyARefusedInflightFrameWantsTheBootClear)
{
    // The boot clear carries the FC's uptime as flight_elapsed_ms, and the
    // OC's replay filter holds that against the next flight's first INFLIGHT
    // frames, so the OC's own "no flight" must never draw it.
    const FlightSnapshotData none = stamped(fsa::noFlightPayload());
    EXPECT_FALSE(fsa::refusedInflight(verdictOf(none), none));

    const FlightSnapshotData landed = snapshotIn(LANDED);
    EXPECT_FALSE(fsa::refusedInflight(verdictOf(landed), landed));

    const FlightSnapshotData live = snapshotIn(INFLIGHT);
    EXPECT_FALSE(fsa::refusedInflight(verdictOf(live), live));   // restored, not cleared

    const FlightSnapshotData sim = snapshotIn(INFLIGHT, /*sim=*/true);
    EXPECT_TRUE(fsa::refusedInflight(verdictOf(sim), sim));

    FlightSnapshotData other_version = snapshotIn(INFLIGHT);
    other_version.version = FlightSnapshotData::VERSION + 1;
    other_version = stamped(other_version);
    EXPECT_TRUE(fsa::refusedInflight(verdictOf(other_version), other_version));

    FlightSnapshotData torn = snapshotIn(INFLIGHT, /*sim=*/true);
    torn.flight_elapsed_ms ^= 1;
    EXPECT_FALSE(fsa::refusedInflight(verdictOf(torn), torn));   // garbage: retried, not cleared
}

TEST(FlightSnapshotAnswer, TheBootClearIsUnchangedForEveryFrameAnOcCouldServeBefore)
{
    // An OC before the explicit answer served only INFLIGHT frames
    // (snapshotServable), and the FC cleared after every definitive non-restore
    // answer. For those frames the clear is exactly what it was.
    for (uint8_t version : {FlightSnapshotData::VERSION, (uint8_t)(FlightSnapshotData::VERSION + 1)})
        for (uint8_t sim : {0, 1})
            for (bool crc_ok : {true, false})
            {
                FlightSnapshotData s = {};
                s.magic = FlightSnapshotData::MAGIC;
                s.version = version;
                s.rocket_state = INFLIGHT;
                s.sim_flight = sim;
                const fsa::Verdict v = fsa::classify(s, crc_ok);
                const bool cleared_before = previousFcDecision(s, crc_ok).answered_no_flight;
                const bool clears_now = decisionFrom(v).answered_no_flight && fsa::refusedInflight(v, s);
                EXPECT_EQ(clears_now, cleared_before)
                    << "version " << int(version) << " sim " << int(sim) << " crc_ok " << crc_ok;
            }
}

TEST(FlightSnapshotAnswer, TheNoFlightFrameIsExactlyTheFrameTheFcReads)
{
    const FlightSnapshotData none = stamped(fsa::noFlightPayload());
    const std::vector<uint8_t> f = wireFrame(SNAPSHOT_MSG, &none, sizeof(none));

    // The FC reads exactly kFrameLen bytes and checks SOF + type at offset 0.
    ASSERT_EQ(f.size(), fsa::kFrameLen);
    EXPECT_EQ(f[0], 0xAA);
    EXPECT_EQ(f[1], 0x55);
    EXPECT_EQ(f[2], 0xAA);
    EXPECT_EQ(f[3], 0x55);
    EXPECT_EQ(f[4], SNAPSHOT_MSG);
    EXPECT_EQ(f[5], sizeof(FlightSnapshotData));

    FlightSnapshotData back = {};
    std::memcpy(&back, f.data() + 6, sizeof(back));
    EXPECT_EQ(verdictOf(back), fsa::Verdict::NoFlight);
}

TEST(FlightSnapshotAnswer, TheBugsStatusReplyIsNotASnapshotFrame)
{
    // What the FC read five times per boot: the OC's last staged status reply.
    // Its type byte is not SNAPSHOT_MSG, so the FC never even classifies it,
    // and it is shorter than the snapshot frame the FC clocks out.
    const uint8_t status_payload[OUT_STATUS_PAYLOAD_BYTES] = {1};
    const std::vector<uint8_t> f =
        wireFrame(OUT_STATUS_RESPONSE, status_payload, sizeof(status_payload));
    EXPECT_EQ(f[4], 0xA9);
    EXPECT_NE(f[4], SNAPSHOT_MSG);
    EXPECT_LT(f.size(), fsa::kFrameLen);
}

// ---------------------------------------------------------------------------
// When the OC may say "no flight"
// ---------------------------------------------------------------------------

TEST(SnapshotAnswerPolicy, AServableFrameIsAlwaysServed)
{
    for (bool garbled : {false, true})
        for (bool recovery : {false, true})
            for (bool settled : {false, true})
                EXPECT_EQ(decide(true, garbled, recovery, settled), Reply::Serve);
}

TEST(SnapshotAnswerPolicy, AColdOcBootAnswersNoFlightEvenBeforeItsLoggerIsUp)
{
    // Not a recovery context: the boot-time stores were retired, so the RAM
    // cache the caller just checked is the only place a flight could be.
    EXPECT_EQ(decide(false, false, false, false), Reply::NoFlight);
    EXPECT_EQ(decide(false, false, false, true), Reply::NoFlight);
}

TEST(SnapshotAnswerPolicy, ARecoveryBootWaitsForItsStores)
{
    // A rail restore or a token restore whose logger has not come up: the
    // NAND re-seed or the MRAM slot could still hold the flight.
    EXPECT_EQ(decide(false, false, true, false), Reply::Silent);
    // Once they have had their chance, nothing servable means no flight.
    EXPECT_EQ(decide(false, false, true, true), Reply::NoFlight);
}

TEST(SnapshotAnswerPolicy, AGarbledMramSlotIsNeverNoFlight)
{
    for (bool recovery : {false, true})
        for (bool settled : {false, true})
            EXPECT_EQ(decide(false, true, recovery, settled), Reply::Silent);
}
