package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull

/**
 * #828 (Android half). Both render sites collapsed continuity to a Bool —
 * `pyroCont(channel) && dataStatus == LIVE`, drawn green or red — even though
 * [TelemetryData.pyroMeasuredContinuity] had been on the wire since #803 and
 * could tell "never tested" from "measured open". It had zero callers.
 *
 * A channel nobody has measured has no reading at all. Rendering that as a
 * confident MEASURED open reads to an operator as a dead igniter or a spent
 * charge, on a channel that is in fact live.
 */
// Distinct from TelemetryDataTest's `SH`: two files in one package cannot
// both declare a private top-level typealias of the same name.
private typealias PSH = TelemetryData.SensorHealth

class PyroContinuityTest {

    private fun decode(json: String): TelemetryData =
        assertNotNull(TelemetryData.decode(json), "frame must decode: $json")

    /** Measured-continuity bits live at shift 24 + (ch-1)*2. */
    private fun measured(vararg channelToState: Pair<Int, PSH>): TelemetryData {
        var h = 0
        for ((ch, st) in channelToState) h = h or (st.raw shl (24 + (ch - 1) * 2))
        return decode("""{"st":"READY","h":$h}""")
    }

    private fun verdict(
        t: TelemetryData,
        channel: Int,
        isConnected: Boolean = true,
        dataStatus: TelemetryData.DataStatus = TelemetryData.DataStatus.LIVE,
        isBaseStation: Boolean = false,
    ) = pyroContinuityOf(t, channel, isConnected, dataStatus, isBaseStation)

    // ── The defect ──────────────────────────────────────────────────────

    @Test
    fun neverMeasuredChannelIsUntestedNotOpen() {
        // ch1 measured OK means the firmware reports measured bits at all, so
        // ch3's NA is a real "not tested this session".
        val t = measured(1 to PSH.OK)
        assertEquals(PyroContinuity.UNTESTED, verdict(t, 3))
        assertNotEquals(PyroContinuity.OPEN, verdict(t, 3))
    }

    @Test
    fun measuredOpenIsOpen() {
        assertEquals(PyroContinuity.OPEN, verdict(measured(3 to PSH.BAD), 3))
    }

    @Test
    fun measuredPresentIsPresent() {
        assertEquals(PyroContinuity.PRESENT, verdict(measured(1 to PSH.OK), 1))
    }

    @Test
    fun degradedMeansConfiguredButUntested() {
        assertEquals(PyroContinuity.UNTESTED, verdict(measured(2 to PSH.DEGRADED), 2))
    }

    // ── #297 fail-safe ──────────────────────────────────────────────────

    @Test
    fun staleFrameIsNoDataNotOpen() {
        val t = measured(1 to PSH.OK)
        val v = verdict(t, 1, dataStatus = TelemetryData.DataStatus.STALE)
        assertEquals(PyroContinuity.NO_DATA, v)
        assertNotEquals(PyroContinuity.OPEN, v)
    }

    @Test
    fun syncingFrameIsNoData() {
        val t = measured(1 to PSH.OK)
        assertEquals(
            PyroContinuity.NO_DATA,
            verdict(t, 1, dataStatus = TelemetryData.DataStatus.SYNCING),
        )
    }

    @Test
    fun disconnectedIsNoData() {
        assertEquals(PyroContinuity.NO_DATA, verdict(measured(1 to PSH.OK), 1, isConnected = false))
    }

    @Test
    fun staleBeatsAMeasuredOpen() {
        // A held-over reading of ANY kind is untrustworthy once the stream is
        // stale — including a red one.
        val t = measured(1 to PSH.BAD)
        assertEquals(
            PyroContinuity.NO_DATA,
            verdict(t, 1, dataStatus = TelemetryData.DataStatus.STALE),
        )
    }

    // ── Legacy firmware, direct link ────────────────────────────────────

    @Test
    fun legacyDirectLinkFallsBackToTheRawContBit() {
        // "ps" layout: b0 armed, then (cont, fired) per channel — b1 = ch1 cont.
        val t = decode("""{"st":"READY","ps":${1 shl 1}}""")
        assertNull(
            t.pyroMeasuredContinuity(1),
            "premise: this frame predates the measured bits",
        )
        assertEquals(PyroContinuity.PRESENT, verdict(t, 1))
        // Genuinely cannot tell open from untested on this path — pinned so
        // it is not "improved" into UNTESTED forever against older rockets.
        assertEquals(PyroContinuity.OPEN, verdict(t, 2))
    }

    // ── Relay path ──────────────────────────────────────────────────────

    @Test
    fun relayPrefersTheMeasuredBits() {
        val t = measured(1 to PSH.OK, 2 to PSH.BAD)
        assertEquals(PyroContinuity.PRESENT, verdict(t, 1, isBaseStation = true))
        assertEquals(PyroContinuity.OPEN, verdict(t, 2, isBaseStation = true))
        assertEquals(PyroContinuity.UNTESTED, verdict(t, 3, isBaseStation = true))
    }

    @Test
    fun relayLegacyUsesConfigGatedHealthAndNeverInventsAnOpen() {
        // Config-gated pyro health lives at shift 12 + (ch-1)*2. An
        // unconfigured channel is NA — unknowable on this path, NOT open.
        val h = (PSH.OK.raw shl 12) or (PSH.BAD.raw shl 14)
        val t = decode("""{"st":"READY","h":$h}""")
        assertNull(t.pyroMeasuredContinuity(1), "premise: no measured bits set")
        assertEquals(PyroContinuity.PRESENT, verdict(t, 1, isBaseStation = true))
        assertEquals(PyroContinuity.OPEN, verdict(t, 2, isBaseStation = true))
        assertEquals(PyroContinuity.UNTESTED, verdict(t, 3, isBaseStation = true))
    }

    @Test
    fun relayIgnoresTheRawContBitItCannotReceive() {
        // The 65-byte LoRa downlink carries no pyro_status, so a "ps" value
        // must never drive the relay verdict.
        val t = decode("""{"st":"READY","ps":${1 shl 1}}""")
        assertEquals(PyroContinuity.UNTESTED, verdict(t, 1, isBaseStation = true))
    }

    // ── Bounds ──────────────────────────────────────────────────────────

    @Test
    fun outOfRangeChannelsDoNotClaimAMeasurement() {
        val t = measured(1 to PSH.OK)
        for (ch in listOf(0, 5, -1)) {
            assertNotEquals(PyroContinuity.PRESENT, verdict(t, ch))
        }
    }
}
