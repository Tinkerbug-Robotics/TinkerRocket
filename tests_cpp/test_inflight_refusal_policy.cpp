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
