package com.tinkerbug.tinkerrocket.app

import com.tinkerbug.tinkerrocket.protocol.AntennaOffset
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The GNSS antenna offset footer and inline error — the wording is iOS's
 * (`SettingsView.antennaOffsetFooter` / `antennaInputError`), and the status
 * it appends follows the GNSS log footer's rules.
 */
class AntennaOffsetCaptionTest {

    private val measured = AntennaOffset(true, 312, -25, 4)

    private fun caption(
        measuredShown: Boolean = false,
        profile: AntennaOffset = AntennaOffset.UNKNOWN,
        connected: Boolean = true,
        haveConfig: Boolean = true,
        reported: AntennaOffset? = AntennaOffset.UNKNOWN,
        receiver: String? = "mosaic-G5",
        inflight: Boolean = false,
    ): String = antennaOffsetCaption(measuredShown, profile, connected, haveConfig, reported, receiver, inflight)

    @Test
    fun unknown_explains_the_default_and_says_it_is_in_use() {
        val text = caption()
        assertTrue(text.startsWith("Unknown (the default): the flight computer allows for a GNSS antenna up to 30 cm"))
        assertTrue(text.endsWith(" In use by the flight computer."))
    }

    @Test
    fun measured_names_the_board_axes() {
        val text = caption(measuredShown = true, profile = measured, reported = measured)
        assertTrue(text.startsWith("From the flight computer’s IMU chip to the center of the GNSS antenna"))
        assertTrue("Z out of the component side" in text)
        assertTrue(text.endsWith(" In use by the flight computer."))
    }

    @Test
    fun a_different_report_is_said() {
        val text = caption(measuredShown = true, profile = measured, reported = AntennaOffset.UNKNOWN)
        assertTrue(text.endsWith(" The flight computer reports Unknown."))
        val other = caption(profile = AntennaOffset.UNKNOWN, reported = measured)
        assertTrue(other.endsWith(" The flight computer reports X 312, Y -25, Z 4 mm."))
    }

    @Test
    fun a_draft_is_not_sent_yet() {
        val text = caption(measuredShown = true, profile = AntennaOffset.UNKNOWN)
        assertTrue(text.endsWith(" Not sent yet: enter all three, then tap Done."))
    }

    @Test
    fun firmware_without_the_setting_and_flight_are_said() {
        assertTrue(caption(reported = null).endsWith(" This rocket’s firmware has no antenna offset setting."))
        assertTrue(caption(inflight = true).endsWith(" Locked while the rocket is flying."))
        // No readback yet, or not connected: no status at all.
        assertFalse("flight computer reports" in caption(haveConfig = false, reported = null))
        assertFalse("In use" in caption(connected = false))
    }

    @Test
    fun another_receiver_keeps_it_for_later() {
        val text = caption(receiver = "SAM-M10Q")
        assertTrue(text.endsWith(
            " In use by the flight computer. Only the mosaic-G5’s navigation filter uses it; " +
                "the rocket keeps it for when one does.",
        ))
        assertFalse("Only the mosaic-G5" in caption(receiver = null))
    }

    @Test
    fun inline_error_names_the_bad_fields() {
        assertNull(antennaInputError("312", "-25", "4"))
        assertEquals(
            "Board Y: enter whole millimeters from -2000 to 2000. Nothing is sent until all three are.",
            antennaInputError("312", "2001", "4"),
        )
        assertEquals(
            "Board X, Z: enter whole millimeters from -2000 to 2000. Nothing is sent until all three are.",
            antennaInputError("", "0", "1.5"),
        )
    }
}
