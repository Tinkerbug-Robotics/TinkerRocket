package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * #1085 — the MTU-trim flag was decoded on both platforms and consumed on
 * neither, so a frame the firmware deliberately labelled "partial" rendered as
 * a rocket whose readings had stopped.
 */
class TrimAdvisoryTest {

    @Test
    fun `a session that has never seen a trimmed frame shows nothing`() {
        // The common case, and the one that must stay silent: on a link that
        // settles at a large MTU the whole frame fits and the flag never sets.
        assertFalse(TrimAdvisory.isShowing(null, 0L))
        assertFalse(TrimAdvisory.isShowing(null, 10_000_000L))
    }

    @Test
    fun `the line holds after the last trimmed frame rather than flickering`() {
        // The flag toggles with payload size frame to frame, so a per-frame
        // render would strobe at telemetry rate.
        val t = 100_000L
        assertTrue(TrimAdvisory.isShowing(t, t))
        assertTrue(TrimAdvisory.isShowing(t, t + TrimAdvisory.HOLD_MS - 1))
        assertTrue(TrimAdvisory.isShowing(t, t + TrimAdvisory.HOLD_MS))
        assertFalse(TrimAdvisory.isShowing(t, t + TrimAdvisory.HOLD_MS + 1))
    }

    @Test
    fun `the hold outlives a single frame interval by a wide margin`() {
        // Telemetry runs at a few Hz; the hold has to cover several frames or
        // it is the same flicker with extra steps.
        assertTrue(TrimAdvisory.HOLD_MS >= 2_000L)
    }

    @Test
    fun `the wording says what it is not, because that is the useful half`() {
        // An operator seeing blanks mid-flight will read them as sensors
        // dropping out. The line exists to say they are not.
        assertTrue(TrimAdvisory.TEXT.contains("not sensor failures"))
        // Advisory, not verdict — nothing that would compete with the state
        // banner this line is forbidden from recolouring.
        //
        // "FAIL" is deliberately NOT in this list: the text says "not sensor
        // failures", and that phrase is the useful half of the sentence. The
        // first version of this test banned the substring and failed on its
        // own wording, which is the wrong lesson to encode.
        for (w in listOf("ERROR", "ABORT", "CRITICAL", "WARNING")) {
            assertFalse(TrimAdvisory.TEXT.uppercase().contains(w), "alarm word: $w")
        }
        // And it must not read as a verdict about the vehicle.
        assertFalse(TrimAdvisory.TEXT.uppercase().startsWith("FAIL"))
    }

    @Test
    fun `a later trimmed frame re-arms the hold`() {
        val first = 1_000L
        val later = first + 60_000L
        assertFalse(TrimAdvisory.isShowing(first, later))
        assertTrue(TrimAdvisory.isShowing(later, later + 100L))
    }
}
