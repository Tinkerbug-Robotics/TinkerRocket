package com.tinkerbug.tinkerrocket.app

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * #1650: the base station's network-ID drop line (iOS SignalStrengthView).
 * Another pair on its own network ID is normal traffic while the base station
 * tracks its rocket; the same drops while it hears no rocket are the fault the
 * counter exists for, and must read as one.
 */
class NetidDropsLineTest {

    @Test
    fun nothingDropped_noLine() {
        assertNull(netidDropsLine(null, trackingHealthy = false))
        assertNull(netidDropsLine(0, trackingHealthy = false))
    }

    @Test
    fun whileTracking_itIsInformation() {
        val (text, warn) = netidDropsLine(4, trackingHealthy = true)!!
        assertFalse(warn)
        assertEquals("Hearing 4 packets from other networks (another pair nearby is normal)", text)
    }

    @Test
    fun withNoRocket_itIsAWarning() {
        val (text, warn) = netidDropsLine(4, trackingHealthy = false)!!
        assertTrue(warn)
        assertTrue(text.startsWith("NetID mismatch: 4 packets dropped"))
    }
}
