package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull

/**
 * #1099: a non-finite headline number cannot be JSON-encoded. iOS's
 * CSVGenerator threw on encode and discarded the CSV with it; both generators
 * now sanitise where the numbers are made so the two stay twins.
 */
class FlightSummarySanitiseTest {
    @Test
    fun nonFiniteBecomesNull() {
        assertNull(Double.NaN.finiteOrNull())
        assertNull(Double.POSITIVE_INFINITY.finiteOrNull())
        assertNull(Double.NEGATIVE_INFINITY.finiteOrNull())
        assertNull((null as Double?).finiteOrNull())
    }

    @Test
    fun finiteValuesPassThroughUntouched() {
        assertEquals(412.5, 412.5.finiteOrNull())
        assertEquals(0.0, 0.0.finiteOrNull())
        assertEquals(-3.25, (-3.25).finiteOrNull())
    }

    @Test
    fun summaryBuiltFromSanitisedValuesEncodes() {
        // A NaN that reached the summary would fail here; a sanitised one is
        // simply absent from the JSON.
        val s = FlightSummary(
            maxAltitudeM = Double.NaN.finiteOrNull(),
            maxSpeedMps = 88.0.finiteOrNull(),
            burnoutTimeS = null,
            apogeeTimeS = Double.POSITIVE_INFINITY.finiteOrNull(),
            settings = null,
        )
        val json = s.toJson()
        assertEquals(false, json.contains("NaN"))
        assertEquals(false, json.contains("Infinity"))
        assertEquals(true, json.contains("88"))
    }
}
