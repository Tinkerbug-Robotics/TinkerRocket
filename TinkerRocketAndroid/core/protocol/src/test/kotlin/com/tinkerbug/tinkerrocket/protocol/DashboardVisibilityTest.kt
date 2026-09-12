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

    // ── #1047: the value cards ──────────────────────────────────────────

    @Test
    fun `a base station with no rocket caught draws no rocket value cards`() {
        // The reported case: phone on the base station during pad setup,
        // before the rocket is powered. The BS still pushes a frame so its own
        // battery/logging/RSSI stay live, and the rocket half is a zero-init
        // struct — but nsat/palt/soc/vol are emitted unconditionally, so those
        // zeros arrive as readings rather than as absent keys.
        assertFalse(showValueViews(TelemetryData.DataStatus.SYNCING))
    }

    @Test
    fun `a live or stale stream still draws them`() {
        // Staleness is handled by the dim/hold treatment elsewhere; this gate
        // is only about fabricated zeros, so it must not also hide real data
        // that has merely gone old.
        assertTrue(showValueViews(TelemetryData.DataStatus.LIVE))
        assertTrue(showValueViews(TelemetryData.DataStatus.STALE))
    }

    @Test
    fun `the value gate ignores link type where the banner gate does not`() {
        // Deliberate asymmetry, and the reason is worth pinning. On a DIRECT
        // link SYNCING means the FC is still booting, and there the state
        // banner is the only real information there is — so showStateBanner
        // draws it. The value cards are zeros in BOTH readings of SYNCING, so
        // they are hidden in both.
        assertTrue(showStateBanner(TelemetryData.DataStatus.SYNCING, isBaseStation = false))
        assertFalse(showValueViews(TelemetryData.DataStatus.SYNCING))
    }

    @Test
    fun `only the rocket battery row goes, never the base station row`() {
        // The BS's own pack is a true live reading and is what tells the
        // operator the base station is up. Dropping the whole card would take
        // that with it.
        assertFalse(
            showRocketBatteryRow(TelemetryData.DataStatus.SYNCING, isBaseStation = true),
        )
        // Direct link mid-boot: the rocket row is the thing being waited on.
        assertTrue(
            showRocketBatteryRow(TelemetryData.DataStatus.SYNCING, isBaseStation = false),
        )
        assertTrue(
            showRocketBatteryRow(TelemetryData.DataStatus.LIVE, isBaseStation = true),
        )
    }
}
