// Host tests for FcRescuePolicy: when a flight computer that cannot finish
// booting comes up in rescue mode, where the out computer can still deliver a
// firmware update to it.
//
// The imperative half (the RTC record, the setup watchdog's esp_timer, the OC
// probe and the rescue loop itself) is not testable off-target. What is, and
// what these pin down, is the decision: which boots count, what clears them,
// what a power cycle does, and above all that rescue mode is never entered
// while the out computer could hand back a flight to carry on.

#include <gtest/gtest.h>

#include "fc_rescue_policy.h"
#include "fc_ota_session_policy.h"   // kStableRunMs, for the ordering check

using namespace FcRescuePolicy;

namespace {

constexpr uint32_t kImageA = 0x00010000;   // ota_0
constexpr uint32_t kImageB = 0x00310000;   // ota_1

// One boot of image `img`: decide, then end it the way `outcome` says.
enum class Outcome { Crash, Stall, Proven };

struct Board
{
    Record rtc{};   // garbage until the first boot writes it, like real RTC memory

    BootDecision boot(ResetClass reset, uint32_t img = kImageA)
    {
        const BootDecision d = atBoot(rtc, reset, img);
        rtc = d.record;
        return d;
    }

    void end(Outcome o)
    {
        if (o == Outcome::Stall)  rtc = onSetupStall(rtc);
        if (o == Outcome::Proven) rtc = onProven(rtc);
        // Crash: the record is left exactly as the boot stored it.
    }
};

const OcCheck kOcNoFlight{true, false};
const OcCheck kOcFlight{true, true};
const OcCheck kOcSilent{false, false};

}  // namespace

// ---------------------------------------------------------------------------
// The constants against the boot they guard
// ---------------------------------------------------------------------------
TEST(FcRescuePolicy, TheSetupWatchdogNeverCatchesASlowButHealthyBoot)
{
    // Healthy boot 25.4 s (bench 2026-08-19); a dead GNSS module makes begin()
    // block to its ~35 s deadline, about 50 s in all. Twice that, at least.
    EXPECT_GE(kSetupStallMs, 2u * 50000u);
}

TEST(FcRescuePolicy, AProvenBootOutlastsAnUpdateAndTheOtaValidation)
{
    // A 678 kB V9 image at the app's 20 kB/s relay cap is ~34 s on the wire,
    // plus the erase and the verify. A boot that dies before an update could
    // finish must keep counting.
    EXPECT_GT(kProvenRunMs, 678000u / 20u + 5000u);
    // And it is a stronger claim than the #1123 OTA validation window, which
    // marks an image good after 10 s.
    EXPECT_GT(kProvenRunMs, FcOtaSessionPolicy::kStableRunMs);
}

TEST(FcRescuePolicy, OneCrashIsNotEnough)
{
    EXPECT_GE(kMaxUnprovenBoots, 2u);
}

// ---------------------------------------------------------------------------
// The record
// ---------------------------------------------------------------------------
TEST(FcRescuePolicy, ColdBootGarbageIsNeverTrusted)
{
    // A power-on leaves RTC memory random. Anything not sealed by atBoot is
    // invalid, including a record that happens to carry the magic.
    Record junk{};
    junk.magic = kMagic;
    junk.image_id = kImageA;
    junk.unproven_boots = 200;
    junk.setup_stalled = 1;
    junk.check = 0x1234;
    EXPECT_FALSE(valid(junk));

    // Even on an abnormal reset, an invalid record starts the counts over.
    const BootDecision d = atBoot(junk, ResetClass::Abnormal, kImageA);
    EXPECT_EQ(d.mode, BootMode::Normal);
    EXPECT_EQ(d.record.unproven_boots, 1u);
    EXPECT_EQ(d.record.setup_stalled, 0u);
    EXPECT_TRUE(valid(d.record));
}

TEST(FcRescuePolicy, EveryTransitionLeavesASealedRecord)
{
    const Record r0 = freshRecord(kImageA);
    EXPECT_TRUE(valid(r0));
    EXPECT_TRUE(valid(onSetupStall(r0)));
    EXPECT_TRUE(valid(onProven(onSetupStall(r0))));
    EXPECT_TRUE(valid(onRescueRefused(onSetupStall(r0))));
    EXPECT_TRUE(valid(onUpdateInstalled(r0)));
    EXPECT_TRUE(valid(atBoot(r0, ResetClass::Abnormal, kImageA).record));
}

TEST(FcRescuePolicy, ASingleFlippedBitIsCaught)
{
    const Record r = onSetupStall(freshRecord(kImageA));
    for (int bit = 0; bit < 8; ++bit)
    {
        Record a = r; a.unproven_boots ^= (uint8_t)(1u << bit);
        Record b = r; b.setup_stalled ^= (uint8_t)(1u << bit);
        EXPECT_FALSE(valid(a)) << "unproven_boots bit " << bit;
        EXPECT_FALSE(valid(b)) << "setup_stalled bit " << bit;
    }
    for (int bit = 0; bit < 32; ++bit)
    {
        Record c = r; c.image_id ^= (1u << bit);
        EXPECT_FALSE(valid(c)) << "image_id bit " << bit;
    }
}

// ---------------------------------------------------------------------------
// Crash loops
// ---------------------------------------------------------------------------
TEST(FcRescuePolicy, ACrashLoopReachesRescueModeAfterTheLimit)
{
    Board b;
    // The power-on that starts it all.
    EXPECT_EQ(b.boot(ResetClass::FreshStart).mode, BootMode::Normal);
    b.end(Outcome::Crash);
    for (uint8_t i = 1; i < kMaxUnprovenBoots; ++i)
    {
        EXPECT_EQ(b.boot(ResetClass::Abnormal).mode, BootMode::Normal) << "boot " << (int)i + 1;
        b.end(Outcome::Crash);
    }
    const BootDecision d = b.boot(ResetClass::Abnormal);
    EXPECT_EQ(d.mode, BootMode::RescueCandidate);
    EXPECT_EQ(d.reason, Reason::BootsUnproven);
}

TEST(FcRescuePolicy, ACandidateIsNotCountedAgain)
{
    // Rescue mode itself can reset (a wedge inside an update panics). The
    // counts must still say "rescue" when it comes back, not run on and wrap.
    Board b;
    b.boot(ResetClass::FreshStart);
    for (int i = 0; i < 10; ++i) { b.end(Outcome::Crash); b.boot(ResetClass::Abnormal); }
    const uint8_t at_limit = b.rtc.unproven_boots;
    EXPECT_EQ(at_limit, kMaxUnprovenBoots);
    EXPECT_EQ(b.boot(ResetClass::Abnormal).mode, BootMode::RescueCandidate);
    EXPECT_EQ(b.rtc.unproven_boots, at_limit);
}

TEST(FcRescuePolicy, AProvenBootClearsTheCount)
{
    Board b;
    b.boot(ResetClass::FreshStart);
    b.end(Outcome::Crash);
    b.boot(ResetClass::Abnormal);
    b.end(Outcome::Proven);   // ran loop_fc() for a minute: this image works
    // A later crash (a watchdog in the field) starts again from one.
    const BootDecision d = b.boot(ResetClass::Abnormal);
    EXPECT_EQ(d.mode, BootMode::Normal);
    EXPECT_EQ(d.record.unproven_boots, 1u);
}

TEST(FcRescuePolicy, ProvenIsTheRunNotTheSetup)
{
    EXPECT_FALSE(provenAfter(0));
    EXPECT_FALSE(provenAfter(kProvenRunMs - 1));
    EXPECT_TRUE(provenAfter(kProvenRunMs));
}

// ---------------------------------------------------------------------------
// Stalls
// ---------------------------------------------------------------------------
TEST(FcRescuePolicy, OneStallIsEnough)
{
    // The bench case: parked on "Starting sensors". The setup watchdog has
    // already waited over twice the slowest legitimate boot, so the very next
    // boot is a candidate; there is no reason to sit through three more.
    Board b;
    b.boot(ResetClass::FreshStart);
    b.end(Outcome::Stall);
    const BootDecision d = b.boot(ResetClass::Abnormal);   // esp_restart() is ESP_RST_SW
    EXPECT_EQ(d.mode, BootMode::RescueCandidate);
    EXPECT_EQ(d.reason, Reason::SetupStalled);
}

TEST(FcRescuePolicy, AStallTakesPrecedenceInTheLog)
{
    Board b;
    b.boot(ResetClass::FreshStart);
    for (int i = 0; i < 5; ++i) { b.end(Outcome::Crash); b.boot(ResetClass::Abnormal); }
    b.end(Outcome::Stall);
    EXPECT_EQ(b.boot(ResetClass::Abnormal).reason, Reason::SetupStalled);
}

// ---------------------------------------------------------------------------
// Fresh starts: the operator's way out
// ---------------------------------------------------------------------------
TEST(FcRescuePolicy, APowerCycleAlwaysTriesANormalBoot)
{
    Board b;
    b.boot(ResetClass::FreshStart);
    b.end(Outcome::Stall);
    ASSERT_EQ(b.boot(ResetClass::Abnormal).mode, BootMode::RescueCandidate);
    // Battery pull, or the app's power button through the OC's rail.
    const BootDecision d = b.boot(ResetClass::FreshStart);
    EXPECT_EQ(d.mode, BootMode::Normal);
    EXPECT_EQ(d.record.unproven_boots, 1u);
    EXPECT_EQ(d.record.setup_stalled, 0u);
}

TEST(FcRescuePolicy, RtcRemanenceAcrossAPowerOnIsIgnored)
{
    // A quick battery reseat can leave RTC memory intact (#848's note). A
    // power-on still starts over, valid-looking record or not.
    Record survivor = onSetupStall(freshRecord(kImageA));
    ASSERT_TRUE(valid(survivor));
    const BootDecision d = atBoot(survivor, ResetClass::FreshStart, kImageA);
    EXPECT_EQ(d.mode, BootMode::Normal);
    EXPECT_EQ(d.record.setup_stalled, 0u);
}

// ---------------------------------------------------------------------------
// The counts belong to one image
// ---------------------------------------------------------------------------
TEST(FcRescuePolicy, ARollbackStartsTheOldImageFresh)
{
    // A new image stalls on its first boot; the setup watchdog restarts it
    // while it is still PENDING_VERIFY, so the bootloader rolls back. The
    // image that comes up is a different one and owes nothing to the stall.
    Board b;
    b.boot(ResetClass::Abnormal, kImageB);   // the update's restart is a SW reset
    b.end(Outcome::Stall);
    const BootDecision d = b.boot(ResetClass::Abnormal, kImageA);
    EXPECT_EQ(d.mode, BootMode::Normal);
    EXPECT_EQ(d.record.image_id, kImageA);
    EXPECT_EQ(d.record.unproven_boots, 1u);
}

TEST(FcRescuePolicy, AnInstalledUpdateClearsTheCountsBeforeItRestarts)
{
    Board b;
    b.boot(ResetClass::FreshStart);
    b.end(Outcome::Stall);
    ASSERT_EQ(b.boot(ResetClass::Abnormal).mode, BootMode::RescueCandidate);
    // Rescue mode takes the update and restarts into it.
    b.rtc = onUpdateInstalled(b.rtc);
    EXPECT_EQ(b.rtc.unproven_boots, 0u);
    EXPECT_EQ(b.rtc.setup_stalled, 0u);
    // If that image is refused and the old one returns, it gets a normal boot.
    EXPECT_EQ(b.boot(ResetClass::Abnormal, kImageA).mode, BootMode::Normal);
}

// ---------------------------------------------------------------------------
// Never at the cost of a flight
// ---------------------------------------------------------------------------
TEST(FcRescuePolicy, RescueModeNeedsALiveOcWithNoFlightToHandBack)
{
    Board b;
    b.boot(ResetClass::FreshStart);
    b.end(Outcome::Stall);
    const BootDecision d = b.boot(ResetClass::Abnormal);
    ASSERT_EQ(d.mode, BootMode::RescueCandidate);

    EXPECT_TRUE(mayEnterRescue(d, kOcNoFlight));
    // The OC hands back an in-flight snapshot: the normal boot would resume
    // that flight, so the normal boot it is.
    EXPECT_FALSE(mayEnterRescue(d, kOcFlight));
    // No answer at all: it may be still booting with a flight in its NAND, and
    // a silent OC could not relay an update anyway.
    EXPECT_FALSE(mayEnterRescue(d, kOcSilent));
    EXPECT_FALSE(mayEnterRescue(d, OcCheck{false, true}));
}

TEST(FcRescuePolicy, ANormalBootNeverEntersRescueMode)
{
    Board b;
    const BootDecision d = b.boot(ResetClass::FreshStart);
    ASSERT_EQ(d.mode, BootMode::Normal);
    EXPECT_FALSE(mayEnterRescue(d, kOcNoFlight));
}

TEST(FcRescuePolicy, ARefusedCandidateBootsNormallyAndKeepsCounting)
{
    // A flight in progress (a crash loop in the air): every boot is a
    // candidate, every one is refused, and every one runs the normal boot that
    // restores the flight. None of them may wedge the board in rescue mode.
    Board b;
    b.boot(ResetClass::FreshStart);
    for (int i = 0; i < 6; ++i) { b.end(Outcome::Crash); b.boot(ResetClass::Abnormal); }
    for (int i = 0; i < 5; ++i)
    {
        const BootDecision d = b.boot(ResetClass::Abnormal);
        ASSERT_EQ(d.mode, BootMode::RescueCandidate);
        ASSERT_FALSE(mayEnterRescue(d, kOcFlight));
        b.rtc = onRescueRefused(b.rtc);
        b.end(Outcome::Crash);
    }
    EXPECT_GE(b.rtc.unproven_boots, kMaxUnprovenBoots);
    // Once the flight is over and the OC has nothing to hand back, the next
    // candidate is admitted.
    const BootDecision d = b.boot(ResetClass::Abnormal);
    EXPECT_TRUE(mayEnterRescue(d, kOcNoFlight));
}

TEST(FcRescuePolicy, ARefusalSpendsTheStallFlag)
{
    Board b;
    b.boot(ResetClass::FreshStart);
    b.end(Outcome::Stall);
    b.boot(ResetClass::Abnormal);
    b.rtc = onRescueRefused(b.rtc);
    EXPECT_EQ(b.rtc.setup_stalled, 0u);
    // A refused boot that then proves itself clears everything.
    b.end(Outcome::Proven);
    EXPECT_EQ(b.boot(ResetClass::Abnormal).record.unproven_boots, 1u);
}

TEST(FcRescuePolicy, RefusalCountSaturates)
{
    Record r = freshRecord(kImageA);
    r.unproven_boots = 0xFF;
    r = onRescueRefused(sealed(r));
    EXPECT_EQ(r.unproven_boots, 0xFFu);
    EXPECT_TRUE(valid(r));
}

// ---------------------------------------------------------------------------
// What rescue mode answers
// ---------------------------------------------------------------------------
TEST(FcRescuePolicy, OnlyTheUpdateSessionIsServed)
{
    EXPECT_TRUE(servesInRescue(OTA_BEGIN_PENDING));
    EXPECT_TRUE(servesInRescue(OTA_FINISH_CMD));
    EXPECT_TRUE(servesInRescue(OTA_ABORT_CMD));
    int served = 0;
    for (int c = 0; c < 256; ++c)
    {
        if (servesInRescue((uint8_t)c)) ++served;
        // Above all: nothing that fires a channel, moves a fin or starts a
        // sequence, whatever the OC happens to be serving.
        if (cmdIsOneShotActuating((uint8_t)c))
            EXPECT_FALSE(servesInRescue((uint8_t)c)) << "cmd 0x" << std::hex << c;
    }
    EXPECT_EQ(served, 3);
    EXPECT_FALSE(servesInRescue(0));
    EXPECT_FALSE(servesInRescue(PYRO_FIRE_TEST));
    EXPECT_FALSE(servesInRescue(SERVO_CONFIG_PENDING));
}
