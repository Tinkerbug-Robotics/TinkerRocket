package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * #1039 — Android twin of TinkerRocketAppTests/StaleTelemetryTreatmentTests.
 *
 * The bug these pin: `flightReadiness` is derived from the sensor-health bits
 * and has no freshness term, and the out computer keeps republishing the last
 * good bits forever after the flight computer dies. So the card that answers
 * "is it safe to fly right now" answered yes, in green, about a rocket that had
 * stopped talking.
 */
class ReadinessHoldTest {

    @Test
    fun `a live stream is never held and keeps the real verdict`() {
        assertFalse(ReadinessHold.isHeld(null))
        assertEquals(
            "Ready to fly",
            ReadinessHold.label(TelemetryData.FlightReadiness.READY, null),
        )
    }

    @Test
    fun `any staleness age holds the verdict, whatever the bits say`() {
        // The reported case: green bits captured before the failure must NOT
        // keep rendering "Ready to fly" once the frame is stale.
        assertTrue(ReadinessHold.isHeld(47.0))
        assertEquals(
            "Held — data 47 s old",
            ReadinessHold.label(TelemetryData.FlightReadiness.READY, 47.0),
        )
        // A NOT_READY verdict is held too — the app stops judging in both
        // directions, rather than keeping the scary half of a stale scorecard.
        assertEquals(
            "Held — data 47 s old",
            ReadinessHold.label(TelemetryData.FlightReadiness.NOT_READY, 47.0),
        )
    }

    @Test
    fun `the held wording is not a verdict word`() {
        // "Unknown" would read as a measurement that was taken and failed.
        // "Held" says the app stopped judging because the input stopped.
        val held = ReadinessHold.label(TelemetryData.FlightReadiness.READY, 5.0)
        assertTrue(held.startsWith("Held — "))
        assertFalse(held.contains("Ready"))
        assertFalse(held.contains("unknown", ignoreCase = true))
    }

    @Test
    fun `ageText matches the iOS formatter at and around the minute boundary`() {
        assertEquals("0 s", ReadinessHold.ageText(0.0))
        assertEquals("47 s", ReadinessHold.ageText(47.9))   // truncates, like Int(age)
        assertEquals("59 s", ReadinessHold.ageText(59.0))
        assertEquals("1 min", ReadinessHold.ageText(60.0))
        assertEquals("2 min", ReadinessHold.ageText(150.0))
    }

    @Test
    fun `a non-finite or negative age cannot render nonsense or throw`() {
        // iOS guards this because `.lost(lastSeen: nil)` hands it a non-finite
        // age and Int() would trap. Kotlin would not trap, but it would render
        // a garbage number, which is worse than saying so.
        assertEquals("unknown", ReadinessHold.ageText(Double.NaN))
        assertEquals("unknown", ReadinessHold.ageText(Double.POSITIVE_INFINITY))
        assertEquals("unknown", ReadinessHold.ageText(-1.0))
        assertEquals(
            "Held — no telemetry received",
            ReadinessHold.label(TelemetryData.FlightReadiness.READY, Double.POSITIVE_INFINITY),
        )
    }
}
