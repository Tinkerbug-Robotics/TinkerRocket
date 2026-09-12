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
        // #1060: fresh by default so every pre-existing case reads as it did.
        telemetryAgeMs: Long? = 0L,
    ) = pyroContinuityOf(t, channel, isConnected, dataStatus, isBaseStation, telemetryAgeMs)

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
        // #1048: a clear bit is UNTESTED, not OPEN. The old assertion pinned
        // the opposite — the reasoning being that reporting UNTESTED forever
        // against an older rocket was worse. It is not: OPEN renders a
        // confident red "NO CONT", which an operator reads as a dead igniter
        // or a fired charge on a channel that may be live, and the FC sets
        // this bit as `cont_known && cont_state` so a clear bit cannot
        // distinguish the two. A set bit still proves presence. iOS twin:
        // BLEDevice.swift `telemetry.pyroCont(channel:) ? .present : .untested`.
        assertEquals(PyroContinuity.UNTESTED, verdict(t, 2))
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

    // ── #1060: connected is not the same as talking ─────────────────────

    @Test
    fun aStalledButConnectedLinkStopsReportingContinuity() {
        // The reported case. The OC holds the BLE connection up and stops
        // notifying — during a flight-log download, whose read loop runs
        // inside loop_oc and blocks the telemetry send. dataStatus stays LIVE
        // because nothing recomputes it, so before #1060 the last frame's
        // green CONT was held for as long as the connection stood. On an ARMED
        // rocket that is a safety readout asserting a measurement it does not
        // have.
        val t = measured(1 to TelemetryData.SensorHealth.OK)
        assertEquals(PyroContinuity.PRESENT, verdict(t, 1, telemetryAgeMs = 0L))
        assertEquals(PyroContinuity.PRESENT, verdict(t, 1, telemetryAgeMs = 2_999L))
        assertEquals(PyroContinuity.NO_DATA, verdict(t, 1, telemetryAgeMs = 3_001L))
        assertEquals(PyroContinuity.NO_DATA, verdict(t, 1, telemetryAgeMs = 60_000L))
    }

    @Test
    fun neverHavingHeardAFrameIsNoDataNotAgeZero() {
        // null means "no frame has ever arrived", which is emphatically not
        // "the last frame arrived just now". Treating it as age zero is how a
        // zero-init struct becomes a green tile.
        val t = measured(1 to TelemetryData.SensorHealth.OK)
        assertEquals(PyroContinuity.NO_DATA, verdict(t, 1, telemetryAgeMs = null))
    }

    @Test
    fun theStaleVerdictOverridesEveryChannelStateIncludingOpen() {
        // Staleness is not "keep the scary half". It withdraws the reading in
        // both directions, exactly as a disconnect does — a stale OPEN is no
        // more a measurement than a stale PRESENT.
        val t = measured(
            1 to TelemetryData.SensorHealth.OK,
            2 to TelemetryData.SensorHealth.BAD,
            3 to TelemetryData.SensorHealth.NA,
        )
        for (ch in 1..3) {
            assertEquals(
                PyroContinuity.NO_DATA,
                verdict(t, ch, telemetryAgeMs = 5_000L),
                "channel $ch",
            )
        }
    }

    @Test
    fun theThresholdMatchesTheOneIosCarries() {
        // f25af04 put 3000 ms on the iOS side. If these drift apart, the two
        // apps disagree about whether a continuity readout is a measurement.
        assertEquals(3000L, TELEMETRY_STALE_THRESHOLD_MS)
    }
}
