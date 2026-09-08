// #1162: does the OC's latched INFLIGHT belief still bind the cmd-8 power-off
// and the self-OTA veto?
//
// The load-bearing rows are the SILENT ones. The term this policy replaces was
// "refuse only while an FC frame is under 3 s old", which meant that an FC
// panic/WDT reboot mid-flight (~10 s of setup_fc with no NonSensorData) opened
// the gate while the rocket was still under canopy — on V7/V8 the power-off
// then cut the rebooting FC's rail. A silent FC must hold the gate until no
// flight could remain; a live one must hold it for as long as it says so.

#include <gtest/gtest.h>

#include "inflight_refusal_policy.h"

using InflightRefusalPolicy::refuse;
using InflightRefusalPolicy::holdRemainingMs;
using InflightRefusalPolicy::kMaxFlightTimeMs;

namespace {
constexpr uint32_t kSec = 1000U;
}

TEST(InflightRefusalPolicy, BoundIsTheFcFlightTimeout) {
    // flight_computer main.cpp MAX_FLIGHT_TIME_MS, and kSnapshotServeTtlMs in
    // the OC (a static_assert there ties the two). Ten minutes.
    EXPECT_EQ(kMaxFlightTimeMs, 600U * kSec);
}

TEST(InflightRefusalPolicy, NotInflightNeverRefuses) {
    // Pad, LANDED, INITIALIZATION: nothing to protect, whatever the stamps say.
    EXPECT_FALSE(refuse(false, true, 0));
    EXPECT_FALSE(refuse(false, false, 0));
    EXPECT_FALSE(refuse(false, true, kMaxFlightTimeMs - 1));
    EXPECT_FALSE(refuse(false, false, kMaxFlightTimeMs + 1));
}

TEST(InflightRefusalPolicy, LiveFcSayingInflightAlwaysRefuses) {
    // Unchanged from before #1162: fresh frames that say INFLIGHT refuse. The
    // flight-time bound never overrides a live FC — a sim held INFLIGHT past
    // the bound stays refused until the operator stops it.
    EXPECT_TRUE(refuse(true, true, 0));
    EXPECT_TRUE(refuse(true, true, 30 * kSec));
    EXPECT_TRUE(refuse(true, true, kMaxFlightTimeMs - 1));
    EXPECT_TRUE(refuse(true, true, kMaxFlightTimeMs));
    EXPECT_TRUE(refuse(true, true, 3U * kMaxFlightTimeMs));
}

TEST(InflightRefusalPolicy, SilentFcHoldsThroughAnFcReboot) {
    // The #1162 scenario. INFLIGHT seen at launch; at T+40 s the FC takes a
    // task-WDT reset and sends nothing for ~10 s; the operator, seeing the
    // telemetry freeze, presses power-off at T+48 s. The old 3 s freshness
    // term opened the gate here. The FC frame is stale, the flight is 48 s
    // old: refuse.
    EXPECT_TRUE(refuse(true, false, 48 * kSec));
}

TEST(InflightRefusalPolicy, SilentFcHoldsFromTheFirstInstant) {
    // FC dies on the very frame the OC learned INFLIGHT from.
    EXPECT_TRUE(refuse(true, false, 0));
}

TEST(InflightRefusalPolicy, SilentFcHoldsForTheWholeFlightBound) {
    // An I2S RX break or a paused ingest (#917) that outlives the descent:
    // the gate stays shut for as long as a flight could still be going.
    EXPECT_TRUE(refuse(true, false, 5U * 60U * kSec));
    EXPECT_TRUE(refuse(true, false, kMaxFlightTimeMs - 1));
}

TEST(InflightRefusalPolicy, SilentFcReleasesOnceNoFlightCouldRemain) {
    // The escape hatch that replaces the 3 s term: past the FC's own timeout,
    // measured from the OC's first sighting of INFLIGHT (at or after launch),
    // a live FC would already have forced LANDED and a dead one has no
    // deployment left to protect. The OC becomes recoverable without a
    // battery pull.
    EXPECT_FALSE(refuse(true, false, kMaxFlightTimeMs));
    EXPECT_FALSE(refuse(true, false, kMaxFlightTimeMs + 1));
    EXPECT_FALSE(refuse(true, false, 0xFFFFFFFFu));
}

TEST(InflightRefusalPolicy, HoldRemainingCountsDownToZero) {
    EXPECT_EQ(holdRemainingMs(0), kMaxFlightTimeMs);
    EXPECT_EQ(holdRemainingMs(48 * kSec), kMaxFlightTimeMs - 48 * kSec);
    EXPECT_EQ(holdRemainingMs(kMaxFlightTimeMs - 1), 1U);
    EXPECT_EQ(holdRemainingMs(kMaxFlightTimeMs), 0U);
    EXPECT_EQ(holdRemainingMs(kMaxFlightTimeMs + 5 * kSec), 0U);
    EXPECT_EQ(holdRemainingMs(0xFFFFFFFFu), 0U);
}

TEST(InflightRefusalPolicy, ReleaseAndRemainingAgree) {
    // The log line's "hold N s left" must read 0 exactly when the gate opens.
    for (uint32_t age = kMaxFlightTimeMs - 3; age <= kMaxFlightTimeMs + 3; ++age) {
        EXPECT_EQ(refuse(true, false, age), holdRemainingMs(age) > 0) << age;
    }
}

// ---------------------------------------------------------------------------
// #1147 items 8 and 9 — WHICH commands the INFLIGHT rule covers.
//
// The #383 rule was written into processUplinkCommand's refusal list and never
// applied to the BLE half, so the same command was refused over LoRa and
// accepted over Bluetooth. The list has already been extended once (#1130 added
// 5 and 6) with only the LoRa site moving, which is the drift this predicate
// exists to stop.
// ---------------------------------------------------------------------------

TEST(InflightRefusedSet, CoversEveryCommandOnTheRule) {
    for (uint8_t cmd : {1, 5, 6, 23, 28, 35, 36}) {
        EXPECT_TRUE(InflightRefusalPolicy::refusedInflight(cmd))
            << "cmd " << (int)cmd << " is on the INFLIGHT rule and must be refused";
    }
}

TEST(InflightRefusedSet, LeavesEverythingElseAlone) {
    // A sample across the id space, including neighbours of the refused ids so
    // an off-by-one in a future edit shows up here.
    for (uint8_t cmd : {0, 2, 3, 4, 7, 8, 14, 22, 24, 27, 29, 34, 37, 45, 67, 70}) {
        EXPECT_FALSE(InflightRefusalPolicy::refusedInflight(cmd))
            << "cmd " << (int)cmd << " must not be blanket-refused";
    }
}

TEST(InflightRefusedSet, SimCommandsAreOnTheRule) {
    // #1130's reason, pinned: a sim START delivered during a real flight resets
    // the flight state that the #317 terminal-LANDED lockout exists to protect.
    EXPECT_TRUE(InflightRefusalPolicy::refusedInflight(5));
    EXPECT_TRUE(InflightRefusalPolicy::refusedInflight(6));
}

TEST(InflightRefusedSet, PyroTestsAreOnTheRule) {
    EXPECT_TRUE(InflightRefusalPolicy::refusedInflight(35));
    EXPECT_TRUE(InflightRefusalPolicy::refusedInflight(36));
}

TEST(InflightRefusedSet, TwentyEightIsMembershipNotAnInstruction) {
    // 28 (guidance target) IS on the rule and the LoRa path refuses it. The BLE
    // path deliberately does not: its rejection is echoed to the app via
    // guid_target, which is feedback the LoRa path cannot give. This test pins
    // that 28 is a member, so a future sweep reading the predicate does not
    // "discover" the BLE exemption and remove it as a bug.
    EXPECT_TRUE(InflightRefusalPolicy::refusedInflight(28));
}

TEST(InflightRefusedSet, MembershipIsIndependentOfTheHoldDecision) {
    // Two orthogonal questions: IS this command on the rule, and DOES the rule
    // currently bind. Composing them is the caller's job.
    EXPECT_TRUE(InflightRefusalPolicy::refusedInflight(36));
    EXPECT_FALSE(InflightRefusalPolicy::refuse(/*state_inflight=*/false,
                                               /*fc_frame_fresh=*/true, 0));
    EXPECT_TRUE(InflightRefusalPolicy::refuse(true, true, 0));
}
