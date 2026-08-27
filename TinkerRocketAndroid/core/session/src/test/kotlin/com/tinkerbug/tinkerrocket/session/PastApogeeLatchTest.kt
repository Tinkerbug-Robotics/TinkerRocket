package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

/**
 * #968: `altApo` / `velApo` are LIVE VOTES from a leaky counter whose baro test
 * requires `alt_est > 15.0f`, so both clear below ~15 m AGL. Measured on the
 * 2026-08-27 sim flight: `alt_apogee` dropped at 130.96 s and stayed down
 * through the end of the log at 138.84 s — the last ~8 seconds of the flight.
 *
 * Consumers that mean "this flight is past apogee" therefore flipped back to
 * "pre-apogee" right before landing. That is what let a stale burnout be
 * announced (#964), and the same clearing caused #235.
 *
 * Mirrors iOS `PastApogeeLatchTests`.
 */
class PastApogeeLatchTest {

    private companion object {
        const val ALT_APO = 0x04
        const val VEL_APO = 0x02
    }

    private fun frame(state: String = "INFLIGHT", fs: Int, rid: Int? = null): TelemetryData {
        val ridPart = if (rid != null) """"rid":$rid,""" else ""
        val t = TelemetryData.decode("""{$ridPart"st":"$state","fs":$fs,"palt":120.5}""")
        assertNotNull(t, "test frame should decode")
        return t
    }

    /**
     * The exact shape of the sim flight: the vote rises at apogee, then clears
     * under ~15 m AGL. The latch must survive that.
     */
    @Test
    fun `latch holds when the live vote clears near landing`() {
        val latch = PastApogeeLatch()
        assertTrue(latch.apply(frame(fs = ALT_APO)).pastApogee, "latch should rise with the vote")

        val late = latch.apply(frame(fs = 0))
        assertFalse(late.altApo, "the live vote really does clear")
        assertTrue(late.pastApogee, "pastApogee must NOT follow the vote down (#968)")
    }

    /** `velApo` leads `altApo` by a few hundred ms, so either vote arms it. */
    @Test
    fun `velocity vote also arms the latch`() {
        val latch = PastApogeeLatch()
        assertTrue(latch.apply(frame(fs = VEL_APO)).pastApogee)
    }

    @Test
    fun `not latched before apogee`() {
        val latch = PastApogeeLatch()
        assertFalse(latch.apply(frame(fs = 0)).pastApogee)
    }

    /** A second flight on the same connection must start clean. */
    @Test
    fun `latch clears on prelaunch for the next flight`() {
        val latch = PastApogeeLatch()
        assertTrue(latch.apply(frame(fs = ALT_APO)).pastApogee)
        assertFalse(
            latch.apply(frame(state = "PRELAUNCH", fs = 0)).pastApogee,
            "PRELAUNCH re-arms for a new flight",
        )
        assertFalse(
            latch.apply(frame(fs = 0)).pastApogee,
            "and it stays clear until the next apogee",
        )
    }

    /**
     * A base station relays two flights at once. One rocket reaching apogee
     * must not mark the other (#390).
     */
    @Test
    fun `relayed rockets latch independently`() {
        val latch = PastApogeeLatch()
        val seven = latch.apply(frame(fs = ALT_APO, rid = 7))
        val nine = latch.apply(frame(fs = 0, rid = 9))
        assertTrue(seven.pastApogee, "rocket 7 passed apogee")
        assertFalse(nine.pastApogee, "rocket 9 has not — latches must not cross (#390)")
    }

    /** The bit position matches what the firmware would send if it ever did. */
    @Test
    fun `latched bit is fs bit 10`() {
        val latch = PastApogeeLatch()
        assertEquals(0x400, TelemetryData.PAST_APOGEE_BIT)
        assertEquals(
            ALT_APO or 0x400,
            latch.apply(frame(fs = ALT_APO)).flightStatusBits,
        )
    }
}
