package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * The predicted landing call: when the base station loses the rocket at
 * touchdown, before the landed frame gets through, the landing is called at
 * the predicted touchdown instead of minutes later. The flight data is in
 * [TouchdownPredictorTest]. iOS twin: `FlightAnnouncerPredictedLandingTests`.
 */
class FlightAnnouncerPredictedLandingTest {

    private class FakeSpeech : AnnouncerSpeech {
        val spoken = mutableListOf<String>()
        val interrupts = mutableListOf<Boolean>()
        override var isBusy: Boolean = false
        override fun speak(text: String, interrupt: Boolean) {
            spoken += text
            interrupts += interrupt
        }
        override fun stop() = Unit
    }

    /** What the announcer schedules; [fireUpTo] runs whatever is due. */
    private class Scheduler {
        private class Entry(val atMs: Long, val action: () -> Unit) { var cancelled = false }
        private val pending = mutableListOf<Entry>()
        fun schedule(atMs: Long, action: () -> Unit): () -> Unit {
            val e = Entry(atMs, action)
            pending += e
            return { e.cancelled = true }
        }
        fun fireUpTo(nowMs: Long) {
            val due = pending.filter { it.atMs <= nowMs }
            pending.removeAll(due)
            due.filter { !it.cancelled }.forEach { it.action() }
        }
        val liveCount: Int get() = pending.count { !it.cancelled }
    }

    private class Rig(enabled: Boolean = true, recovery: RecoveryProfile? = null) {
        var now = 1_000_000L
        val speech = FakeSpeech()
        val scheduler = Scheduler()
        val announcer = FlightAnnouncer(
            speech = speech,
            initialEnabled = enabled,
            clock = { now },
            recovery = { recovery },
            schedule = scheduler::schedule,
        )
        var touchdownMs = 0L
        val landingCalls get() = speech.spoken.filter { it.startsWith("Predicted landing") || it.startsWith("Landed") }

        /** Advance to [s] seconds after touchdown with no frames (the link is
         *  down), firing whatever comes due on the way. */
        fun waitUntil(s: Double, stepMs: Long = 100) {
            while (now < touchdownMs + (s * 1000).toLong()) {
                now += stepMs
                scheduler.fireUpTo(now)
            }
        }
    }

    private fun frame(
        state: String = "INFLIGHT",
        palt: Float? = null,
        rate: Float? = null,
        apo: Boolean = false,
        apoVoteCleared: Boolean = false,
        landed: Boolean = false,
        lat: Double? = null,
        lon: Double? = null,
        stale: Boolean = false,
    ) = TelemetryData(
        state = state,
        pressureAlt = palt,
        altitudeRate = rate,
        flightStatusBits = (if (apo) (0x04 or 0x400) else 0) or
            (if (apoVoteCleared) 0x400 else 0) or
            (if (landed) 0x08 else 0),
        latitude = lat,
        longitude = lon,
        dataStatus = if (stale) TelemetryData.DataStatus.STALE else TelemetryData.DataStatus.LIVE,
    )

    /** The pad, then the real last 20 s of 03 at the relay's spacing; the
     *  rig's touchdown is the flight computer's. */
    private fun Rig.flyRollCtrl03(lat: Double? = null, lon: Double? = null) {
        announcer.processTelemetry(frame(state = "PRELAUNCH", palt = 0f, lat = 40.0, lon = -75.0))
        touchdownMs = now + 30_000
        for ((t, alt, rate) in TouchdownPredictorTest.ROLL_CTRL_03) {
            now = touchdownMs + (t * 1000).toLong()
            announcer.processTelemetry(
                frame(palt = alt, rate = rate, apo = alt > 15f, apoVoteCleared = alt <= 15f, lat = lat, lon = lon),
            )
        }
    }

    /** The 2026-10-24 flight 03: the link died 0.5 s before touchdown. The
     *  call comes 1.5 s after the last frame, 1.0 s after the real touchdown. */
    @Test
    fun linkDiesAtTouchdown_callsTheLandingFromThePrediction() {
        val rig = Rig()
        rig.flyRollCtrl03(lat = 40.001, lon = -75.0)
        rig.waitUntil(0.9)
        assertTrue(rig.landingCalls.isEmpty(), "not before 1.5 s of silence")
        rig.waitUntil(1.1)
        assertEquals(listOf("Predicted landing. 111 meters away"), rig.landingCalls)
        assertTrue(rig.speech.interrupts.last(), "a landing call interrupts, like the real one")
    }

    /** With no timer (the default), the base station's stale re-pushes still
     *  make the call. */
    @Test
    fun staleRePushAlsoMakesTheCall() {
        val rig = Rig()
        rig.flyRollCtrl03()
        rig.now = rig.touchdownMs + 1_500
        rig.announcer.processTelemetry(frame(palt = 2f, rate = -7f, apoVoteCleared = true, stale = true))
        assertEquals(listOf("Predicted landing."), rig.landingCalls)
    }

    /** The landed frame that arrives minutes later must not call it again. */
    @Test
    fun lateLandedFrame_doesNotCallAgain() {
        val rig = Rig()
        rig.flyRollCtrl03()
        rig.waitUntil(3.0)
        rig.now = rig.touchdownMs + 140_000
        rig.announcer.processTelemetry(frame(state = "LANDED", palt = 0f, rate = 0f, apoVoteCleared = true, landed = true))
        assertEquals(listOf("Predicted landing."), rig.landingCalls)
    }

    /** While the link holds, the real landed frame makes the call. */
    @Test
    fun linkHolds_onlyTheRealLandingCall() {
        val rig = Rig()
        rig.flyRollCtrl03()
        rig.now = rig.touchdownMs
        rig.announcer.processTelemetry(frame(palt = 0f, rate = -2f, apoVoteCleared = true))
        rig.now = rig.touchdownMs + 500
        rig.announcer.processTelemetry(frame(palt = 0f, rate = 0f, apoVoteCleared = true, landed = true))
        rig.waitUntil(10.0)
        assertEquals(listOf("Landed."), rig.landingCalls)
        assertEquals(0, rig.scheduler.liveCount, "the landed frame cancels the pending call")
    }

    /** A predicted call, then the link comes back with the rocket well off
     *  the ground: the call is taken back, and the real landing is made. */
    @Test
    fun linkReturnsWithTheRocketStillUp_realLandingIsStillCalled() {
        val rig = Rig()
        rig.announcer.processTelemetry(frame(state = "PRELAUNCH", palt = 0f))
        rig.now += 30_000
        for (alt in listOf(180f, 170f, 160f, 150f)) {          // lost at 150 m, 20 m/s
            rig.announcer.processTelemetry(frame(palt = alt, rate = -20f, apo = true))
            rig.now += 500
        }
        rig.touchdownMs = rig.now
        rig.waitUntil(8.0)
        assertEquals(listOf("Predicted landing."), rig.landingCalls)

        var alt = 60f                                            // back, still flying
        while (alt >= 0f) {
            rig.announcer.processTelemetry(frame(palt = alt, rate = -5f, apo = alt > 15f, apoVoteCleared = alt <= 15f))
            rig.now += 500
            rig.scheduler.fireUpTo(rig.now)
            alt -= 2.5f
        }
        rig.announcer.processTelemetry(frame(palt = 0f, rate = 0f, apoVoteCleared = true, landed = true))
        assertEquals(listOf("Predicted landing.", "Landed."), rig.landingCalls)
    }

    @Test
    fun voiceOff_noPredictedCall() {
        val rig = Rig(enabled = false)
        rig.flyRollCtrl03()
        rig.waitUntil(5.0)
        assertTrue(rig.speech.spoken.isEmpty())
    }

    @Test
    fun prelaunchResets() {
        val rig = Rig()
        rig.flyRollCtrl03()
        rig.announcer.processTelemetry(frame(state = "PRELAUNCH", palt = 0f))
        rig.waitUntil(5.0)
        assertTrue(rig.landingCalls.isEmpty())
    }

    /** Dual deploy from the profile's pyro channels: lost at 400 m under
     *  drogue, the call waits for the main leg. */
    @Test
    fun dualDeployProfile_callWaitsForTheMainLeg() {
        val profile = RecoveryProfile(drogueMps = 18.0, mainMps = 6.5, mainDeployAglM = 213.0, pyroMainAglM = 243.84)
        val rig = Rig(recovery = profile)
        rig.announcer.processTelemetry(frame(state = "PRELAUNCH", palt = 0f))
        rig.now += 30_000
        for (alt in listOf(424f, 416f, 408f, 400f)) {
            rig.announcer.processTelemetry(frame(palt = alt, rate = -16f, apo = true))
            rig.now += 500
        }
        rig.now -= 500
        rig.touchdownMs = rig.now
        val expected = (400 - 243.84) / 16 + 243.84 / 6.5      // 47.3 s
        rig.waitUntil(expected - 0.5, stepMs = 250)
        assertTrue(rig.landingCalls.isEmpty())
        rig.waitUntil(expected + 0.5, stepMs = 250)
        assertEquals(listOf("Predicted landing."), rig.landingCalls)
    }
}
