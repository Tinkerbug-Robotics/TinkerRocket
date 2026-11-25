// What the direct BLE link shows while the FC reboots and the OC stays up
// (fc_reboot_display_policy.h).
//
// The OC keeps the previous NonSensorData's state through an FC-only reset,
// on purpose, and used to publish it as is: on the Beetle (2026-10-26) the
// apps read a green PRELAUNCH over "Flight computer not running — no flight
// loop". On the ground the published state now reads INITIALIZATION (and the
// data status SYNCING) while the FC reports boot steps; in flight and after a
// landing it is left alone.

#include <gtest/gtest.h>

#include "fc_reboot_display_policy.h"

using FcRebootDisplay::groundReboot;
using FcRebootDisplay::publishedState;

TEST(FcRebootDisplay, NoBootReportPublishesTheStoredState) {
    for (RocketState s : {INITIALIZATION, READY, PRELAUNCH, INFLIGHT, LANDED, MAG_CALIBRATION}) {
        EXPECT_FALSE(groundReboot(s, false)) << (int)s;
        EXPECT_EQ(publishedState(s, false), s) << (int)s;
    }
}

TEST(FcRebootDisplay, AGroundRebootReadsAsAFreshBoot) {
    // The pad states describe the FC's own setup, which the reboot ended. An
    // armed READY must not stay on screen while the FC is starting again.
    for (RocketState s : {INITIALIZATION, READY, PRELAUNCH, MAG_CALIBRATION}) {
        EXPECT_TRUE(groundReboot(s, true)) << (int)s;
        EXPECT_EQ(publishedState(s, true), INITIALIZATION) << (int)s;
    }
}

TEST(FcRebootDisplay, FlightPhasesAreKept) {
    // An FC that reboots in flight is still flying, and resumes from the
    // snapshot the OC hands it. After a landing, the apps' landing and
    // recovery views must not reset.
    for (RocketState s : {INFLIGHT, LANDED}) {
        EXPECT_FALSE(groundReboot(s, true)) << (int)s;
        EXPECT_EQ(publishedState(s, true), s) << (int)s;
    }
}
