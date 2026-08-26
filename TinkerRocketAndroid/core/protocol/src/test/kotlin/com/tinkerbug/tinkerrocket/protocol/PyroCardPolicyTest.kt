package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * #838 item 7 — the pyro card was hidden entirely on base-station links,
 * justified by a comment that misstated what iOS does.
 *
 * The Android dashboard wrapped the whole section in
 * `if (!session.isBaseStation)` with the comment "direct rocket links only —
 * iOS hides pyro on BS links". iOS does the OPPOSITE: it renders
 * `PyroChannelsView(relayMode: true)` on a base-station link, with continuity
 * from the relayed health scorecard.
 *
 * So the comment both misdescribed the reference implementation and disguised
 * a missing safety display as deliberate parity. In the standard pad
 * configuration — rocket relaying over LoRa to a base station the phone is
 * paired to over BLE — the Android operator had NO continuity readout at all,
 * on the only link available at the pad, and the next maintainer reading that
 * comment would have believed the gap was intentional.
 *
 * `pyroContinuityOf` already resolved the relay path correctly (see
 * PyroContinuityTest). Nothing on that screen ever called it with
 * `isBaseStation = true`, because nothing rendered.
 */
class PyroCardPolicyTest {

    /** THE regression. */
    @Test
    fun theCardShowsOnBothLinkTypes() {
        assertTrue(PyroCardPolicy.showCard())
    }

    /**
     * The badge must be visible on a relay without waiting for `armed` — which
     * does not exist there, so gating on it hid the readout permanently.
     */
    @Test
    fun theBadgeIsAlwaysRevealedOnARelay() {
        for (ch in 1..4) {
            assertTrue(
                PyroCardPolicy.revealed(isBaseStation = true, armed = false,
                                        testedChannel = 0, channel = ch),
                "channel $ch would never show a continuity reading over LoRa",
            )
        }
    }

    /** Direct links keep the reveal ladder they had. */
    @Test
    fun aDirectLinkRevealsOnlyWhenArmedOrJustTested() {
        assertFalse(PyroCardPolicy.revealed(false, armed = false, testedChannel = 0, channel = 1))
        assertTrue(PyroCardPolicy.revealed(false, armed = true, testedChannel = 0, channel = 1))
        assertTrue(PyroCardPolicy.revealed(false, armed = false, testedChannel = 1, channel = 1))
        // Single-reveal: another tile's test does not reveal this one.
        assertFalse(PyroCardPolicy.revealed(false, armed = false, testedChannel = 2, channel = 1))
    }

    /**
     * armed/fired do not exist on the relay path — the LoRa downlink carries
     * no pyro_status — so a zeroed field must not render as a measurement.
     */
    @Test
    fun armedAndFiredAreDirectLinkOnly() {
        assertFalse(PyroCardPolicy.armed(isBaseStation = true, telemetryArmed = true))
        assertFalse(PyroCardPolicy.fired(isBaseStation = true, telemetryFired = true))
        assertTrue(PyroCardPolicy.armed(isBaseStation = false, telemetryArmed = true))
        assertTrue(PyroCardPolicy.fired(isBaseStation = false, telemetryFired = true))
    }

    /**
     * No test button on a relay: the stand-back LoRa test (cmds 35/36) is
     * iOS-only, so the button would send a direct-link cmd 35 into a base
     * station — nothing happens, and the silence reads as a fault.
     */
    @Test
    fun noTestButtonOnARelay() {
        assertFalse(PyroCardPolicy.showTestButton(
            isBaseStation = true, armed = false, fired = false,
            inflight = false, powerPinOn = true))
    }

    /** The direct-link gate is unchanged, including the rail check. */
    @Test
    fun theDirectLinkTestButtonGateIsUnchanged() {
        fun gate(armed: Boolean = false, fired: Boolean = false,
                 inflight: Boolean = false, power: Boolean = true) =
            PyroCardPolicy.showTestButton(false, armed, fired, inflight, power)

        assertTrue(gate())
        assertFalse(gate(armed = true))
        assertFalse(gate(fired = true))
        assertFalse(gate(inflight = true))
        // The OC refuses cmd 35 with the rail off — a queued ARM pulse would
        // deliver at the next power-on.
        assertFalse(gate(power = false))
    }

    /**
     * A relay never receives the cmd-20 readback, so the active profile stands
     * in — what the app believes it pushed, not what the rocket echoed.
     */
    @Test
    fun configComesFromTheProfileOnARelayAndTheRocketOnADirectLink() {
        assertEquals(PyroCardPolicy.ConfigSource.ACTIVE_PROFILE,
                     PyroCardPolicy.configSource(isBaseStation = true))
        assertEquals(PyroCardPolicy.ConfigSource.ROCKET_READBACK,
                     PyroCardPolicy.configSource(isBaseStation = false))
    }

    /** The relay title says where the numbers came from. */
    @Test
    fun theTitleDistinguishesTheLink() {
        assertEquals("Pyro Channels (via LoRa)", PyroCardPolicy.title(true, armed = false))
        assertEquals("Pyro Channels", PyroCardPolicy.title(false, armed = false))
        assertEquals("Pyro Channels — ARMED", PyroCardPolicy.title(false, armed = true))
        // ARMED cannot appear on a relay: armed() already forced it false, and
        // claiming ARMED from a link that cannot report it would be worse than
        // showing nothing.
        assertEquals("Pyro Channels (via LoRa)", PyroCardPolicy.title(true, armed = true))
    }
}
