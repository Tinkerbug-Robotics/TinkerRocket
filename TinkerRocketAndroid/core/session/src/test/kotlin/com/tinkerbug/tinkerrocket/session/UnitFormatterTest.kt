package com.tinkerbug.tinkerrocket.session

import kotlin.test.Test
import kotlin.test.assertEquals

/**
 * Pins the display formats to the iOS `UnitFormatter` outputs (#160) — the
 * values here were computed from the iOS format strings, so a change on
 * either side that un-twins the formatting fails this suite.
 */
class UnitFormatterTest {

    private val m = UnitSystem.METRIC
    private val i = UnitSystem.IMPERIAL

    @Test
    fun `altitude promotes to km in metric, never to miles in imperial`() {
        assertEquals("400 m", UnitFormatter.altitude(400.0, m))
        assertEquals("1.50 km", UnitFormatter.altitude(1500.0, m))
        assertEquals("1312 ft", UnitFormatter.altitude(400.0, i))
        // 5-figure feet, still feet — apogees beat a mile routinely.
        assertEquals("16404 ft", UnitFormatter.altitude(5000.0, i))
        assertEquals("m", UnitFormatter.altitudeUnit(m))
        assertEquals("ft", UnitFormatter.altitudeUnit(i))
        // Raw conversion (iOS metersToDisplay) — the pyro trigger text
        // composes "%.0f%s on descent" from these two.
        assertEquals(150.0, UnitFormatter.altitudeValue(150.0, m), 0.0001)
        assertEquals(492.126, UnitFormatter.altitudeValue(150.0, i), 0.001)
    }

    @Test
    fun `distance promotes to km and miles`() {
        assertEquals("730 m", UnitFormatter.distance(730.0, m))
        assertEquals("2.4 km", UnitFormatter.distance(2400.0, m))
        assertEquals("853 ft", UnitFormatter.distance(260.0, i))
        assertEquals("1.49 mi", UnitFormatter.distance(2400.0, i))
    }

    @Test
    fun `speed and acceleration`() {
        assertEquals("95.0 m/s", UnitFormatter.speed(95.0, m))
        assertEquals("311.7 ft/s", UnitFormatter.speed(95.0, i))
        assertEquals("19.61 m/s²", UnitFormatter.acceleration(19.6133, m))
        assertEquals("2.00 g", UnitFormatter.acceleration(19.6133, i))
        assertEquals("g", UnitFormatter.accelerationUnit(i))
        assertEquals(2.0, UnitFormatter.accelerationValue(19.6133, i), 0.0001)
    }

    @Test
    fun `temperature converts to fahrenheit`() {
        assertEquals("20.0 °C", UnitFormatter.temperature(20.0, m))
        assertEquals("68.0 °F", UnitFormatter.temperature(20.0, i))
    }
}
