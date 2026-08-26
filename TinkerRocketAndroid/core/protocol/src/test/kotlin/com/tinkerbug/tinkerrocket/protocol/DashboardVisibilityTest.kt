package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * The two meanings of SYNCING must not be collapsed. iOS twin:
 * `DashboardVisibilityTests.swift`.
 */
class DashboardVisibilityTest {

    @Test
    fun `direct link draws the banner while syncing`() {
        // SYNCING here means the FC has not sent NonSensorData yet — still in
        // setup_fc(). The banner and its boot line are the only news there is.
        // iOS regressed exactly here and rendered nothing for the whole boot.
        assertTrue(showStateBanner(TelemetryData.DataStatus.SYNCING, isBaseStation = false))
    }

    @Test
    fun `direct link always draws the banner`() {
        for (s in TelemetryData.DataStatus.values()) {
            assertTrue(showStateBanner(s, isBaseStation = false),
                "direct link has a real state to show (status $s)")
        }
    }

    @Test
    fun `syncing base station hides the banner`() {
        // No rocket caught: the BS frame's rocket half is zero-init and state 0
        // is INITIALIZATION, so drawing it would invent a rocket on the pad.
        assertFalse(showStateBanner(TelemetryData.DataStatus.SYNCING, isBaseStation = true))
    }

    @Test
    fun `base station tracking a rocket draws the banner`() {
        // Once a rocket is caught the BS reports LIVE or STALE and the state is
        // the rocket's own. Hiding it there was never the intent.
        assertTrue(showStateBanner(TelemetryData.DataStatus.LIVE, isBaseStation = true))
        assertTrue(showStateBanner(TelemetryData.DataStatus.STALE, isBaseStation = true))
    }
}
