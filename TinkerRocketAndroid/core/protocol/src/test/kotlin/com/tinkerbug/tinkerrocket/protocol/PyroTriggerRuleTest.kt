package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * #1775: the app's copy of the rocket's pyro trigger check (firmware
 * PyroConfigGate.h, iOS PyroTriggerRule). An enabled channel with a mode
 * other than 0 or 1, a NaN value, or an altitude of 0 m or less cannot fire
 * when it is needed, and the rocket now refuses such a config.
 */
class PyroTriggerRuleTest {

    @Test
    fun delay_zeroTo120SecondsIsAccepted() {
        for (v in listOf(0f, -0f, 0.5f, 1.5f, 100f, 120f)) assertTrue(PyroTriggerRule.accepts(0, v), "$v")
        for (v in listOf(-0.001f, -1f, 120.001f, 121f, 1500f, Float.POSITIVE_INFINITY, Float.NaN)) {
            assertFalse(PyroTriggerRule.accepts(0, v), "$v")
        }
    }

    @Test
    fun altitude_30MetersOrMoreIsAccepted() {
        for (v in listOf(30f, 30.48f, 150f, 3000f)) assertTrue(PyroTriggerRule.accepts(1, v), "$v")
        for (v in listOf(29.99f, 29.87f, 0f, -0f, -5f, Float.NaN, Float.NEGATIVE_INFINITY)) {
            assertFalse(PyroTriggerRule.accepts(1, v), "$v")
        }
    }

    @Test
    fun otherModesAreRefused() {
        for (mode in listOf(2, 3, 255, -1)) assertFalse(PyroTriggerRule.accepts(mode, 1f), "$mode")
    }

    @Test
    fun anyRefused_checksOnlyEnabledChannels() {
        assertFalse(
            PyroTriggerRule.anyRefused(
                listOf(
                    PyroChannelConfig(true, 0, 0f), PyroChannelConfig(true, 1, 150f),
                    PyroChannelConfig(false, 1, 0f), PyroChannelConfig(false, 7, Float.NaN),
                ),
            ),
        )
        assertTrue(
            PyroTriggerRule.anyRefused(
                listOf(
                    PyroChannelConfig(true, 0, 0f), PyroChannelConfig(true, 1, 0f),
                    PyroChannelConfig(false, 0, 0f), PyroChannelConfig(false, 0, 0f),
                ),
            ),
        )
    }

    @Test
    fun entry_rejectsWhatToFloatOrNullLetsThrough() {
        // toFloatOrNull() takes all of these; none is a number to send.
        assertNull(PyroTriggerRule.entry("NaN"))
        assertNull(PyroTriggerRule.entry("Infinity"))
        assertNull(PyroTriggerRule.entry("-Infinity"))
        assertNull(PyroTriggerRule.entry("1e40"))
        assertNull(PyroTriggerRule.entry(""))
        assertNull(PyroTriggerRule.entry("1,000.5"))
        assertEquals(150f, PyroTriggerRule.entry(" 150 "))
        assertEquals(1.5f, PyroTriggerRule.entry("1.5"))
        // A comma-decimal keyboard types "1,5".
        assertEquals(1.5f, PyroTriggerRule.entry("1,5"))
    }

    @Test
    fun firstRefused_namesTheFirstFailingEnabledChannel() {
        assertNull(
            PyroTriggerRule.firstRefused(
                listOf(
                    PyroChannelConfig(true, 0, 0f), PyroChannelConfig(true, 1, 150f),
                    PyroChannelConfig(false, 1, 0f), PyroChannelConfig(false, 0, 0f),
                ),
            ),
        )
        assertEquals(
            3,
            PyroTriggerRule.firstRefused(
                listOf(
                    PyroChannelConfig(true, 0, 0f), PyroChannelConfig(false, 1, 0f),
                    PyroChannelConfig(true, 1, 10f), PyroChannelConfig(true, 2, 1f),
                ),
            ),
        )
        assertEquals(
            "Pyro settings not sent: channel 3's trigger is outside what the rocket accepts, " +
                "so it kept its own. Everything else was sent.",
            PyroTriggerRule.notSentText(3),
        )
    }

    @Test
    fun rangeText_theAltitudeShownIsItselfAccepted() {
        assertEquals("The rocket accepts 0 to 120 s.", PyroTriggerRule.rangeText(0, imperial = false))
        assertEquals("The rocket accepts 30 m or more.", PyroTriggerRule.rangeText(1, imperial = false))
        // 30 m is 98.4 ft: 98 ft is under the floor, so the text says 99.
        assertEquals("The rocket accepts 99 ft or more.", PyroTriggerRule.rangeText(1, imperial = true))
        assertTrue(PyroTriggerRule.accepts(1, (99 * 0.3048).toFloat()))
        assertFalse(PyroTriggerRule.accepts(1, (98 * 0.3048).toFloat()))
        assertEquals("Choose a trigger.", PyroTriggerRule.rangeText(2, imperial = false))
    }

    @Test
    fun theProfileDefaultsPassEnabled() {
        // RocketProfile defaults: all time mode, at 1.0, 100.0, 0.0 and 0.0 s.
        assertFalse(
            PyroTriggerRule.anyRefused(
                listOf(
                    PyroChannelConfig(true, 0, 1f), PyroChannelConfig(true, 0, 100f),
                    PyroChannelConfig(true, 0, 0f), PyroChannelConfig(true, 0, 0f),
                ),
            ),
        )
    }
}
