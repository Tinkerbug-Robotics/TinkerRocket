package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull

/** #1091 item 4: the sidecar Android always wrote and never read. */
class FlightSummaryFromJsonTest {
    @Test
    fun roundTripsTheHeadlineNumbers() {
        val s = FlightSummary(maxAltitudeM = 412.5, maxSpeedMps = 88.25, burnoutTimeS = 1.6, apogeeTimeS = 9.1, settings = null)
        val back = FlightSummary.fromJson(s.toJson())!!
        assertEquals(412.5, back.maxAltitudeM); assertEquals(88.25, back.maxSpeedMps)
        assertEquals(1.6, back.burnoutTimeS); assertEquals(9.1, back.apogeeTimeS)
    }

    @Test
    fun aStaleOrUnknownSettingsBlockCannotBlankTheRow() {
        // The iOS lesson of #1077: headline numbers decode on their own.
        val json = """{"max_altitude_m": 100.0, "max_speed_mps": null, "settings": {"imu": {"nonsense": true}}}"""
        val back = FlightSummary.fromJson(json)!!
        assertEquals(100.0, back.maxAltitudeM); assertNull(back.maxSpeedMps); assertNull(back.settings)
    }

    @Test
    fun garbageIsNullNotAThrow() {
        assertNull(FlightSummary.fromJson("not json"))
        assertNull(FlightSummary.fromJson("[1,2,3]"))
    }
}
