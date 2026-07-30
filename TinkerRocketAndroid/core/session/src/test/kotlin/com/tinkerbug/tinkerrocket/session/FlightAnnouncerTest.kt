package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.currentTime
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.math.roundToLong
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * FlightAnnouncer policy + dispatch, Checkpoint A Group 7 / #643.
 *
 * Three layers:
 *  1. Dispatch — the port of iOS `FlightAnnouncerDispatchTests` (#138), over
 *     REAL DeviceSessions fed by FakeFirmware: relayed telemetry must reach
 *     the announcer, not just direct-rocket frames.
 *  2. Wording — the port of iOS `FlightAnnouncerWordingTests` (#235).
 *  3. Flight profile — NEW, no iOS equivalent: the callout state machine
 *     under virtual time. iOS couples policy to AVSpeechSynthesizer so this
 *     layer has never been testable there; the AnnouncerSpeech seam is what
 *     buys it here.
 */
class FlightAnnouncerTest {

    /** Captures speech without any engine. */
    private class FakeSpeech : AnnouncerSpeech {
        val spoken = mutableListOf<String>()
        val interrupts = mutableListOf<Boolean>()
        override var isBusy: Boolean = false
        var stops = 0
        override fun speak(text: String, interrupt: Boolean) {
            spoken += text
            interrupts += interrupt
        }
        override fun stop() {
            stops++
        }
    }

    /** Captures processTelemetry calls (iOS SpyAnnouncer). */
    private class SpyAnnouncer : TelemetryAnnouncer {
        val received = mutableListOf<TelemetryData>()
        var resetCount = 0
        override fun processTelemetry(telemetry: TelemetryData) {
            received += telemetry
        }
        override fun reset() {
            resetCount++
        }
    }

    // ── Frame builder ────────────────────────────────────────────────────

    private fun frame(
        state: String = "INFLIGHT",
        maxSpeed: Float? = null,
        palt: Float? = null,
        rate: Float? = null,
        maxAlt: Float? = null,
        apo: Boolean = false,
        landed: Boolean = false,
        launch: Boolean = false,
        lat: Double? = null,
        lon: Double? = null,
        status: TelemetryData.DataStatus = TelemetryData.DataStatus.LIVE,
    ) = TelemetryData(
        state = state,
        maxSpeedMps = maxSpeed,
        pressureAlt = palt,
        altitudeRate = rate,
        maxAltM = maxAlt,
        flightStatusBits = (if (launch) 0x01 else 0) or
            (if (apo) 0x04 else 0) or (if (landed) 0x08 else 0),
        latitude = lat,
        longitude = lon,
        dataStatus = status,
    )

    private class Rig(initialEnabled: Boolean = true) {
        var now = 0L
        val speech = FakeSpeech()
        val announcer = FlightAnnouncer(
            speech = speech,
            initialEnabled = initialEnabled,
            clock = { now },
        )
    }

    /** Drive to a just-burned-out state: burnout announced, log cleared. */
    private fun Rig.burnOut(maxSpeed: Float = 60f) {
        announcer.processTelemetry(frame(maxSpeed = maxSpeed))
        repeat(3) { announcer.processTelemetry(frame(maxSpeed = maxSpeed)) }
        speech.spoken.clear()
        speech.interrupts.clear()
    }

    // ── 1. Dispatch (port of iOS FlightAnnouncerDispatchTests, #138) ─────

    private class Link(val device: FleetDevice<DeviceSession>, val fw: FakeFirmware)

    private fun TestScope.link(name: String, identityJson: String): Link {
        val fw = FakeFirmware(backgroundScope).apply {
            configJson = null
            configPyroJson = null
            configIdentityJson = identityJson
            pushEchoWithConfig = false
        }
        val session = DeviceSession(
            scope = backgroundScope,
            transport = fw,
            connectedDeviceName = name,
            clock = { currentTime },
        )
        session.start()
        runCurrent()
        advanceTimeBy(1100)
        runCurrent()
        return Link(
            FleetDevice(
                deviceId = "id:$name",
                advertisedName = name,
                generation = 1,
                session = session,
                transport = fw,
                deviceType = session.identity.value.deviceType,
            ),
            fw,
        )
    }

    private fun TestScope.baseStation(): Link = link(
        "TR-B-Test",
        """{"type":"config_identity","uid":"TR-B-Test","un":"TR-B-Test","nid":5,"dt":"B"}""",
    )

    private fun TestScope.rocket(): Link = link(
        "TR-R-Test",
        """{"type":"config_identity","uid":"TR-R-Test","un":"TR-R-Test","nid":5,"rid":3,"dt":"R"}""",
    )

    /** Core #138 regression: relayed telemetry must invoke the announcer. */
    @Test
    fun relayedTelemetry_triggersAnnouncer() = runTest {
        val bs = baseStation()
        val spy = SpyAnnouncer()
        bs.device.session.telemetryAnnouncer = spy

        bs.fw.emitTelemetryJson("""{"rid":7,"run":"Atlas","st":"INFLIGHT","palt":120.5,"mspd":85.0}""")
        runCurrent()

        assertEquals(1, spy.received.size, "Relayed telemetry should invoke the announcer (#138)")
        assertEquals(7, spy.received.first().sourceRocketId)
        assertEquals("INFLIGHT", spy.received.first().state)
        assertEquals(1, bs.device.session.remoteRockets.value.size)
    }

    /** Every frame of a relayed sequence reaches the announcer, in order —
     *  the edge detection inside it needs the transitions. */
    @Test
    fun relayedFlightSequence_allFramesReachAnnouncer() = runTest {
        val bs = baseStation()
        val spy = SpyAnnouncer()
        bs.device.session.telemetryAnnouncer = spy

        for (st in listOf("PRELAUNCH", "INFLIGHT", "LANDED")) {
            bs.fw.emitTelemetryJson("""{"rid":7,"st":"$st","palt":120.5}""")
        }
        runCurrent()

        assertEquals(listOf("PRELAUNCH", "INFLIGHT", "LANDED"), spy.received.map { it.state })
    }

    /** Direct-rocket path keeps working (the case #138 compared against). */
    @Test
    fun directRocketTelemetry_triggersAnnouncer() = runTest {
        val r = rocket()
        val spy = SpyAnnouncer()
        r.device.session.telemetryAnnouncer = spy

        r.fw.emitTelemetryJson("""{"st":"INFLIGHT","palt":120.5,"mspd":85.0}""")
        runCurrent()

        assertEquals(1, spy.received.size)
        assertNull(spy.received.first().sourceRocketId)
    }

    /** A BS frame with no rid (its own heartbeat) routes via the non-relay
     *  branch — announced, and NOT added to the roster. */
    @Test
    fun baseStation_selfTelemetry_triggersAnnouncer() = runTest {
        val bs = baseStation()
        val spy = SpyAnnouncer()
        bs.device.session.telemetryAnnouncer = spy

        bs.fw.emitTelemetryJson("""{"st":"READY","palt":1.0}""")
        runCurrent()

        assertEquals(1, spy.received.size)
        assertEquals(0, bs.device.session.remoteRockets.value.size)
    }

    /** #390: only the FOCUSED rocket's relay frames reach voice — two
     *  interleaved flights' callouts are noise. (No iOS equivalent test.) */
    @Test
    fun relayed_nonFocusedRocket_isSilent() = runTest {
        val bs = baseStation()
        val spy = SpyAnnouncer()
        bs.device.session.telemetryAnnouncer = spy

        bs.fw.emitTelemetryJson("""{"rid":7,"st":"INFLIGHT","palt":10.0}""")   // latches focus
        bs.fw.emitTelemetryJson("""{"rid":8,"st":"INFLIGHT","palt":20.0}""")   // other rocket
        bs.fw.emitTelemetryJson("""{"rid":7,"st":"INFLIGHT","palt":30.0}""")
        runCurrent()

        assertEquals(listOf(7, 7), spy.received.map { it.sourceRocketId })
    }

    /** Disconnect resets the announcer (iOS onDisconnect parity). */
    @Test
    fun disconnect_resetsAnnouncer() = runTest {
        val r = rocket()
        val spy = SpyAnnouncer()
        r.device.session.telemetryAnnouncer = spy

        r.fw.fireDisconnect()
        runCurrent()

        assertEquals(1, spy.resetCount)
    }

    // ── 2. Wording (port of iOS FlightAnnouncerWordingTests, #235) ───────

    @Test
    fun word_matchesRateSign() {
        assertEquals("climbing", FlightAnnouncer.climbDescendWord(80f))
        assertEquals("climbing", FlightAnnouncer.climbDescendWord(1.5f))
        assertEquals("descending", FlightAnnouncer.climbDescendWord(-30f))
        assertEquals("descending", FlightAnnouncer.climbDescendWord(-1.5f))
    }

    @Test
    fun word_omittedWithinDeadband() {
        assertNull(FlightAnnouncer.climbDescendWord(0f))
        assertNull(FlightAnnouncer.climbDescendWord(0.5f))
        assertNull(FlightAnnouncer.climbDescendWord(-0.5f))
        assertNull(FlightAnnouncer.climbDescendWord(1.0f), "boundary is strict (> deadband)")
        assertNull(FlightAnnouncer.climbDescendWord(-1.0f))
    }

    @Test
    fun word_neverContradictsRate_acrossFlightProfile() {
        val rates = listOf(80f, 40f, 12f, 5f, 1.5f, 0.4f, 0f, -0.4f, -1.5f, -8f, -20f, -6f, -3f, -2f)
        for (r in rates) {
            val word = FlightAnnouncer.climbDescendWord(r)
            when {
                r > 1.0f -> assertEquals("climbing", word, "rate $r")
                r < -1.0f -> assertEquals("descending", word, "rate $r")
                else -> assertNull(word, "near-level rate $r")
            }
            if (r < 0) assertTrue(word != "climbing", "never 'climbing' at negative rate ($r)")
        }
    }

    // ── 3. Flight profile (new — the policy state machine) ───────────────

    @Test
    fun burnout_announcedAfterThreeStableFrames_withMaxSpeed() {
        val rig = Rig()
        // Accelerating: each frame raises max_speed → counter stays 0.
        for (v in listOf(20f, 40f, 60f)) rig.announcer.processTelemetry(frame(maxSpeed = v))
        assertEquals(0, rig.speech.spoken.size, "still accelerating — no burnout yet")
        // Stable: three consecutive unchanged frames confirm burnout.
        repeat(2) { rig.announcer.processTelemetry(frame(maxSpeed = 60f)) }
        assertEquals(0, rig.speech.spoken.size, "two stable frames are not enough")
        rig.announcer.processTelemetry(frame(maxSpeed = 60f))
        assertEquals(listOf("Burnout. Max speed 60 meters per second"), rig.speech.spoken)
        assertEquals(listOf(true), rig.interruptsOf())
        // One-shot: further stable frames stay silent.
        rig.announcer.processTelemetry(frame(maxSpeed = 60f))
        assertEquals(1, rig.speech.spoken.size)
    }

    private fun Rig.interruptsOf() = speech.interrupts.toList()

    @Test
    fun burnout_ignoredBelowMinimumSpeed() {
        val rig = Rig()
        // A rocket sitting on the pad jitters below 10 m/s forever.
        repeat(10) { rig.announcer.processTelemetry(frame(maxSpeed = 5f)) }
        assertEquals(0, rig.speech.spoken.size)
    }

    @Test
    fun burnout_jitterWithinHalfMps_countsAsStable() {
        val rig = Rig()
        rig.announcer.processTelemetry(frame(maxSpeed = 60f))
        // +0.4 m/s is telemetry jitter, not acceleration.
        for (v in listOf(60.4f, 60.2f, 60.4f)) rig.announcer.processTelemetry(frame(maxSpeed = v))
        assertEquals(1, rig.speech.spoken.size)
        assertEquals("Burnout. Max speed 60 meters per second", rig.speech.spoken.first())
    }

    @Test
    fun altitude_notCalledOutBeforeBurnout() {
        val rig = Rig()
        // Powered flight: altitude present but burnout not yet confirmed.
        rig.now = 100_000
        rig.announcer.processTelemetry(frame(maxSpeed = 30f, palt = 150f, rate = 80f))
        assertEquals(0, rig.speech.spoken.size, "no altitude callouts during powered flight")
    }

    @Test
    fun altitude_every5s_withClimbWordFromRateSign() {
        val rig = Rig()
        rig.burnOut()
        rig.now = 10_000
        rig.announcer.processTelemetry(frame(palt = 200f, rate = 40f))
        assertEquals(listOf("200 meters, climbing 40 meters per second"), rig.speech.spoken)
        // 3 s later: inside the 5 s gate — silent.
        rig.now = 13_000
        rig.announcer.processTelemetry(frame(palt = 260f, rate = 35f))
        assertEquals(1, rig.speech.spoken.size)
        // 5 s after the first: next callout.
        rig.now = 15_000
        rig.announcer.processTelemetry(frame(palt = 300f, rate = 30f))
        assertEquals(2, rig.speech.spoken.size)
        assertEquals("300 meters, climbing 30 meters per second", rig.speech.spoken[1])
    }

    @Test
    fun altitude_nearLevelRate_omitsDirectionWord() {
        val rig = Rig()
        rig.burnOut()
        rig.now = 10_000
        rig.announcer.processTelemetry(frame(palt = 400f, rate = 0.5f))
        assertEquals(listOf("400 meters"), rig.speech.spoken)
    }

    @Test
    fun apogee_announcedOnRisingEdge_withMaxAltitude() {
        val rig = Rig()
        rig.burnOut()
        rig.announcer.processTelemetry(frame(apo = true, maxAlt = 407f, palt = 400f))
        assertEquals(listOf("Apogee. 407 meters"), rig.speech.spoken)
        // Flag stays set for the rest of the flight — announced exactly once.
        rig.announcer.processTelemetry(frame(apo = true, maxAlt = 407f, palt = 395f))
        assertEquals(1, rig.speech.spoken.size)
    }

    @Test
    fun apogee_withoutMaxAlt_saysAltitudeUnknown() {
        val rig = Rig()
        rig.burnOut()
        rig.announcer.processTelemetry(frame(apo = true))
        assertEquals(listOf("Apogee. altitude unknown"), rig.speech.spoken)
    }

    @Test
    fun descent_firstCallout5sAfterApogee_then10sCadence() {
        val rig = Rig()
        rig.burnOut()
        rig.now = 30_000
        rig.announcer.processTelemetry(frame(apo = true, maxAlt = 400f, palt = 400f))
        rig.speech.spoken.clear()

        // 3 s after apogee: not yet.
        rig.now = 33_000
        rig.announcer.processTelemetry(frame(apo = true, palt = 380f, rate = -8f))
        assertEquals(0, rig.speech.spoken.size)
        // 5 s after apogee: first descent callout.
        rig.now = 35_000
        rig.announcer.processTelemetry(frame(apo = true, palt = 360f, rate = -8f))
        assertEquals(listOf("360 meters, descending 8 meters per second"), rig.speech.spoken)
        // Then every 10 s.
        rig.now = 40_000
        rig.announcer.processTelemetry(frame(apo = true, palt = 320f, rate = -8f))
        assertEquals(1, rig.speech.spoken.size, "inside the 10 s gate")
        rig.now = 45_000
        rig.announcer.processTelemetry(frame(apo = true, palt = 280f, rate = -8f))
        assertEquals(2, rig.speech.spoken.size)
    }

    @Test
    fun landed_flagEdge_speaksHaversineDistanceFromLaunchSite() {
        val rig = Rig()
        // PRELAUNCH captures the launch fix.
        rig.announcer.processTelemetry(frame(state = "PRELAUNCH", lat = 40.0, lon = -74.0))
        rig.burnOut()
        rig.announcer.processTelemetry(frame(apo = true, maxAlt = 400f))
        rig.speech.spoken.clear()

        // Landed 1e-3 deg of longitude east (≈ 85 m at 40°N).
        rig.announcer.processTelemetry(
            frame(apo = true, landed = true, lat = 40.0, lon = -73.999),
        )
        val expected = DriftCast.haversineM(40.0, -74.0, 40.0, -73.999).roundToLong()
        assertEquals(listOf("Landed. $expected meters away"), rig.speech.spoken)
        // Edge is one-shot even with the flag held.
        rig.announcer.processTelemetry(
            frame(apo = true, landed = true, lat = 40.0, lon = -73.999),
        )
        assertEquals(1, rig.speech.spoken.size)
    }

    @Test
    fun landed_stateFallback_whenFlagNeverArrives() {
        val rig = Rig()
        rig.burnOut()
        rig.announcer.processTelemetry(frame(state = "LANDED"))
        assertEquals(listOf("Landed."), rig.speech.spoken)
    }

    @Test
    fun staleFrames_neverAnnounce_evenPastTheTimeGates() {
        val rig = Rig()
        rig.burnOut()
        // Rocket went silent; BS forwards frozen STALE frames forever. Without
        // the LIVE gate these would re-announce every 5 s until battery death.
        for (t in longArrayOf(10_000, 20_000, 30_000, 40_000)) {
            rig.now = t
            rig.announcer.processTelemetry(
                frame(palt = 200f, rate = 40f, status = TelemetryData.DataStatus.STALE),
            )
        }
        assertEquals(0, rig.speech.spoken.size)
    }

    @Test
    fun disabled_tracksFramesSilently() {
        val rig = Rig(initialEnabled = false)
        rig.announcer.processTelemetry(frame(state = "PRELAUNCH"))
        for (v in listOf(60f, 60f, 60f, 60f)) rig.announcer.processTelemetry(frame(maxSpeed = v))
        rig.announcer.processTelemetry(frame(apo = true, maxAlt = 400f))
        assertEquals(0, rig.speech.spoken.size)
    }

    @Test
    fun periodicCallout_skippedWhileSpeaking_immediateInterrupts() {
        val rig = Rig()
        rig.burnOut()
        rig.speech.isBusy = true
        // Periodic altitude: skipped — the next one will have fresher data.
        rig.now = 10_000
        rig.announcer.processTelemetry(frame(palt = 200f, rate = 40f))
        assertEquals(0, rig.speech.spoken.size)
        // Apogee is critical: cancels current speech and goes out anyway.
        rig.announcer.processTelemetry(frame(apo = true, maxAlt = 400f))
        assertEquals(listOf("Apogee. 400 meters"), rig.speech.spoken)
        assertEquals(listOf(true), rig.interruptsOf())
    }

    @Test
    fun prelaunchTransition_resetsOneShots_forTheNextFlight() {
        val rig = Rig()
        rig.burnOut()
        rig.announcer.processTelemetry(frame(apo = true, maxAlt = 400f))
        rig.announcer.processTelemetry(frame(state = "LANDED", apo = true, landed = true))
        rig.speech.spoken.clear()

        // Next flight: PRELAUNCH resets, so all events fire again.
        rig.announcer.processTelemetry(frame(state = "PRELAUNCH"))
        for (v in listOf(50f, 50f, 50f, 50f)) rig.announcer.processTelemetry(frame(maxSpeed = v))
        assertEquals(listOf("Burnout. Max speed 50 meters per second"), rig.speech.spoken)
    }

    @Test
    fun setEnabled_speaksReadyConfirmation_disableStopsSpeech() {
        val rig = Rig(initialEnabled = false)
        var persisted: Boolean? = null
        val announcer = FlightAnnouncer(
            speech = rig.speech,
            initialEnabled = false,
            onEnabledChanged = { persisted = it },
            clock = { rig.now },
        )
        announcer.setEnabled(true)
        assertEquals(listOf("Voice ready"), rig.speech.spoken)
        assertEquals(true, persisted)
        announcer.setEnabled(false)
        assertEquals(1, rig.speech.stops)
        assertEquals(false, persisted)
        // Idempotent: same value → no re-announce, no re-persist.
        persisted = null
        announcer.setEnabled(false)
        assertNull(persisted)
    }

    @Test
    fun imperialUnits_spokenInFeet() {
        val rig = Rig()
        val speech = FakeSpeech()
        val announcer = FlightAnnouncer(
            speech = speech,
            initialEnabled = true,
            clock = { rig.now },
            unitSystem = { UnitSystem.IMPERIAL },
        )
        for (v in listOf(60f, 60f, 60f, 60f)) announcer.processTelemetry(frame(maxSpeed = v))
        assertEquals(listOf("Burnout. Max speed 197 feet per second"), speech.spoken)
        announcer.processTelemetry(frame(apo = true, maxAlt = 400f))
        assertEquals("Apogee. 1312 feet", speech.spoken[1])
    }
}
