#include <gtest/gtest.h>

#include "bs_dup_detect.h"

using namespace bs_dup_detect;

// Every rocket computer ships as rocket_id 1. Two of them on one base station
// look like one rocket whose name and seq jump around; the BS flags the rid
// ("dup":1 in Tier 1) so the operator renumbers one of them.

namespace {

// Feed a beacon the way main.cpp does: detector first, then the tracker write.
Edge beacon(State& s, char (&tracker_name)[24], const char* name)
{
    char cut[24];
    strncpy(cut, name, sizeof(cut) - 1);
    cut[sizeof(cut) - 1] = '\0';
    const Edge e = onBeacon(s, tracker_name, cut);
    strcpy(tracker_name, cut);
    return e;
}

// One rocket's telemetry: consecutive seqs, state READY unless told otherwise.
struct SeqFeed
{
    int32_t last = -1;
    Edge frame(State& s, uint16_t seq, uint8_t state = READY)
    {
        const Edge e = onTelemetry(s, last, seq, state);
        last = seq;
        return e;
    }
};

}  // namespace

TEST(BsDupDetect, AlternatingNamesFlag) {
    State s;
    char name[24] = "";
    EXPECT_EQ(beacon(s, name, "Mantis"), Edge::None);
    EXPECT_EQ(beacon(s, name, "Beetle"), Edge::None);   // could still be a rename
    EXPECT_EQ(beacon(s, name, "Mantis"), Edge::Set);    // back again: two rockets
    EXPECT_TRUE(s.flagged);
    EXPECT_TRUE(s.reasons & kReasonNames);
}

TEST(BsDupDetect, NameThatChangesTwiceBeforeSettlingFlags) {
    State s;
    char name[24] = "";
    beacon(s, name, "A");
    beacon(s, name, "A");
    beacon(s, name, "A");
    beacon(s, name, "B");                                // rename
    EXPECT_FALSE(s.flagged);
    EXPECT_EQ(beacon(s, name, "C"), Edge::Set);          // B never settled
}

TEST(BsDupDetect, SingleRenameDoesNotFlag) {
    State s;
    char name[24] = "";
    for (int i = 0; i < 5; i++) beacon(s, name, "Mantis V5");
    for (int i = 0; i < 5; i++) EXPECT_EQ(beacon(s, name, "Mantis Red"), Edge::None);
    EXPECT_FALSE(s.flagged);
    // A second rename after the first one settled is still a rename.
    for (int i = 0; i < 3; i++) EXPECT_EQ(beacon(s, name, "Mantis Blue"), Edge::None);
    EXPECT_FALSE(s.flagged);
}

TEST(BsDupDetect, RenameRightAfterTheFirstBeaconDoesNotFlag) {
    // Heard once under the old name, then renamed: one change, not a pair.
    State s;
    char name[24] = "";
    beacon(s, name, "Old");
    for (int i = 0; i < 4; i++) EXPECT_EQ(beacon(s, name, "New"), Edge::None);
    EXPECT_FALSE(s.flagged);
}

TEST(BsDupDetect, FirstNameAfterAllocationIsNotAChange) {
    State s;
    char name[24] = "";
    EXPECT_EQ(beacon(s, name, "Beetle"), Edge::None);
    EXPECT_FALSE(s.changed);
    EXPECT_EQ(s.agree, 1);
}

TEST(BsDupDetect, NamesAreComparedAfterTheTrackerCut) {
    // The tracker keeps 23 characters; two beacons that only differ past that
    // are the same name to the base station.
    State s;
    char name[24] = "";
    beacon(s, name, "ABCDEFGHIJKLMNOPQRSTUVW-one");
    beacon(s, name, "ABCDEFGHIJKLMNOPQRSTUVW-two");
    beacon(s, name, "ABCDEFGHIJKLMNOPQRSTUVW-one");
    EXPECT_FALSE(s.flagged);
    EXPECT_EQ(s.agree, 3);
}

TEST(BsDupDetect, InterleavedSeqStreamsFlag) {
    // Two rockets booted at different times: 100.. and 5000.. on one rid.
    State s;
    int32_t last = -1;
    Edge seen = Edge::None;
    uint16_t a = 100, b = 5000;
    for (int i = 0; i < 6; i++) {
        const uint16_t seq = (i % 2 == 0) ? a++ : b++;
        const Edge e = onTelemetry(s, last, seq, READY);
        last = seq;
        if (e == Edge::Set) seen = e;
    }
    EXPECT_EQ(seen, Edge::Set);
    EXPECT_TRUE(s.flagged);
    EXPECT_TRUE(s.reasons & kReasonSeq);
}

TEST(BsDupDetect, InterleavedSeqStreamsWithCloseCountersFlag) {
    // Booted together: 100.. and 110.. — the forward jumps look like loss, the
    // backward ones are -1, and every other frame is one of those.
    State s;
    int32_t last = -1;
    uint16_t a = 100, b = 110;
    for (int i = 0; i < 10; i++) {
        const uint16_t seq = (i % 2 == 0) ? a++ : b++;
        onTelemetry(s, last, seq, READY);
        last = seq;
    }
    EXPECT_TRUE(s.flagged);
}

TEST(BsDupDetect, OneRebootOrOneDuplicateDoesNotFlag) {
    State s;
    SeqFeed f;
    for (uint16_t q = 200; q < 220; q++) f.frame(s, q);
    f.frame(s, 0);       // reboot: seq restarts
    f.frame(s, 1);
    f.frame(s, 1);       // a duplicate receive
    for (uint16_t q = 2; q < 30; q++) f.frame(s, q);
    EXPECT_FALSE(s.flagged);
    EXPECT_LT(seqAnomalyCount(s), kSeqAnomalies);
}

TEST(BsDupDetect, FirstContactIsNotAnAnomaly) {
    State s;
    EXPECT_EQ(onTelemetry(s, -1, 4321, READY), Edge::None);
    EXPECT_EQ(s.seq_frames, 0);
    EXPECT_EQ(s.seq_hist, 0);
}

TEST(BsDupDetect, ThreeAgreeingBeaconsClear) {
    State s;
    char name[24] = "";
    beacon(s, name, "A");
    beacon(s, name, "B");
    ASSERT_EQ(beacon(s, name, "A"), Edge::Set);
    // The other rocket is gone; "A" keeps beaconing.
    EXPECT_EQ(beacon(s, name, "A"), Edge::None);         // 2 agree
    EXPECT_EQ(beacon(s, name, "A"), Edge::Cleared);      // 3 agree
    EXPECT_FALSE(s.flagged);
    EXPECT_EQ(s.reasons, 0);
}

TEST(BsDupDetect, TwoAgreeingBeaconsDoNotClear) {
    State s;
    char name[24] = "";
    beacon(s, name, "A");
    beacon(s, name, "B");
    beacon(s, name, "A");
    beacon(s, name, "A");
    EXPECT_TRUE(s.flagged);
    EXPECT_EQ(beacon(s, name, "B"), Edge::None);         // still taking turns
    beacon(s, name, "B");
    EXPECT_TRUE(s.flagged);
}

TEST(BsDupDetect, SeqFlagNeedsCleanSeqAndFreshBeaconsToClear) {
    State s;
    char name[24] = "";
    beacon(s, name, "Rocket");                           // both share the default name
    int32_t last = -1;
    uint16_t a = 100, b = 9000;
    for (int i = 0; i < 6; i++) {
        const uint16_t seq = (i % 2 == 0) ? a++ : b++;
        onTelemetry(s, last, seq, READY);
        last = seq;
    }
    ASSERT_TRUE(s.flagged);
    // Same name from both: agreeing beacons alone must not clear a seq flag.
    for (int i = 0; i < 4; i++) EXPECT_EQ(beacon(s, name, "Rocket"), Edge::None);
    EXPECT_TRUE(s.flagged);
    // One rocket switched off: its stream is clean for a full window...
    for (int i = 0; i < kSeqWindow; i++) { onTelemetry(s, last, a, READY); last = a++; }
    EXPECT_FALSE(seqDirty(s));
    EXPECT_TRUE(s.flagged);                              // ...but beacons decide
    beacon(s, name, "Rocket");
    beacon(s, name, "Rocket");
    EXPECT_EQ(beacon(s, name, "Rocket"), Edge::Cleared);
}

TEST(BsDupDetect, NoClearInFlight) {
    State s;
    char name[24] = "";
    beacon(s, name, "A");
    beacon(s, name, "B");
    beacon(s, name, "A");
    ASSERT_TRUE(s.flagged);

    SeqFeed f;
    f.frame(s, 10, PRELAUNCH);
    f.frame(s, 11, INFLIGHT);
    // Beacons from the pad rocket while the other one flies: no clear.
    for (int i = 0; i < 5; i++) EXPECT_EQ(beacon(s, name, "A"), Edge::None);
    for (uint16_t q = 12; q < 40; q++) EXPECT_EQ(f.frame(s, q, INFLIGHT), Edge::None);
    EXPECT_TRUE(s.flagged);

    // LANDED with a clean stream ends the hold.
    EXPECT_EQ(f.frame(s, 40, LANDED), Edge::Cleared);
    EXPECT_FALSE(s.flagged);
}

TEST(BsDupDetect, LandedDoesNotClearWhileTheSeqStreamIsStillDoubled) {
    State s;
    int32_t last = -1;
    uint16_t a = 100, b = 9000;
    for (int i = 0; i < 8; i++) {
        const uint16_t seq = (i % 2 == 0) ? a++ : b++;
        onTelemetry(s, last, seq, LANDED);
        last = seq;
    }
    EXPECT_TRUE(s.flagged);   // two landed rockets on one rid: still a duplicate
}

TEST(BsDupDetect, ResetIsAFreshRocket) {
    // Eviction: the slot is reused for another rid and must not inherit the
    // flag, the name history or the seq history.
    State s;
    char name[24] = "";
    beacon(s, name, "A");
    beacon(s, name, "B");
    beacon(s, name, "A");
    int32_t last = -1;
    for (uint16_t q : {1, 900, 2, 901, 3}) { onTelemetry(s, last, q, INFLIGHT); last = q; }
    ASSERT_TRUE(s.flagged);

    s.reset();
    char fresh[24] = "";
    EXPECT_FALSE(s.flagged);
    EXPECT_EQ(s.seq_hist, 0);
    EXPECT_FALSE(s.in_flight);
    EXPECT_EQ(beacon(s, fresh, "C"), Edge::None);         // first write, not a change
    EXPECT_EQ(beacon(s, fresh, "D"), Edge::None);         // first change: a rename
    EXPECT_FALSE(s.flagged);
}

TEST(BsDupDetect, RocketLostInFlightThatRebootsCanClearByBeacons) {
    // Flag raised on the pad, the rocket flies, is lost, reboots into READY:
    // its beacons resume and must be able to clear it — no LANDED ever comes.
    State s;
    char name[24] = "";
    beacon(s, name, "A");
    beacon(s, name, "B");
    beacon(s, name, "A");
    ASSERT_TRUE(s.flagged);
    SeqFeed f;
    for (uint16_t q = 500; q < 520; q++) f.frame(s, q, INFLIGHT);
    EXPECT_TRUE(s.in_flight);
    for (uint16_t q = 0; q < 20; q++) f.frame(s, q, READY);   // rebooted: one -1
    EXPECT_FALSE(s.in_flight);
    // The flagging beacon was the first "A"; two more make three.
    EXPECT_EQ(beacon(s, name, "A"), Edge::None);
    EXPECT_EQ(beacon(s, name, "A"), Edge::Cleared);
}

// ---- identitySettled: the slot controller's gate (bench 2026-10-21) ---------
// The Beetle and the V5 were both rid 1 when the base station first heard
// them; it slotted "rocket 1" within a few frames, both followed the adjust
// onto one grid point, and they wiped out each other's frames from then on.
// An adjust must wait until the rid looks like one rocket.

TEST(BsDupDetectSettled, OneRocketSettlesAfterTwoAgreeingBeacons) {
    State s;
    char name[24] = "";
    SeqFeed f;
    EXPECT_FALSE(identitySettled(s));          // nothing heard
    f.frame(s, 10);
    beacon(s, name, "Beetle");                 // first name: one vote
    EXPECT_FALSE(identitySettled(s));
    f.frame(s, 11);
    beacon(s, name, "Beetle");
    EXPECT_TRUE(identitySettled(s));
}

TEST(BsDupDetectSettled, TwoRocketsOnOneRidNeverSettle) {
    State s;
    char name[24] = "";
    const char* names[] = {"Beetle", "TR-R-e150"};
    for (int i = 0; i < 12; i++)
    {
        beacon(s, name, names[i % 2]);
        EXPECT_FALSE(identitySettled(s)) << i;
    }
    EXPECT_TRUE(s.flagged);
}

TEST(BsDupDetectSettled, TwoRocketsWithOneNameNeverSettleOnTheirSeqs) {
    // Same name too (both renamed alike): only the interleaved seq gives
    // them away, and dirty seq withholds every beacon's vote.
    State s;
    char name[24] = "";
    int32_t last = -1;
    for (int i = 0; i < 16; i++)
    {
        const uint16_t seq = (i % 2) ? (uint16_t)(4000 + i) : (uint16_t)(20 + i);
        onTelemetry(s, last, seq, READY);
        last = seq;
        if (i % 3 == 0) beacon(s, name, "Rocket");
    }
    EXPECT_FALSE(identitySettled(s));
}

TEST(BsDupDetectSettled, AFlaggedRidIsNeverSettled) {
    State s;
    s.flagged = true;
    s.votes = 9;
    EXPECT_FALSE(identitySettled(s));
}

TEST(BsDupDetectSettled, InFlightNeedsOnlyACleanSeqStream) {
    // A rocket rebooted in flight sends no beacons; its clean seq is enough.
    State s;
    SeqFeed f;
    for (uint16_t q = 100; q < 104; q++) f.frame(s, q, INFLIGHT);
    EXPECT_TRUE(identitySettled(s));
}
