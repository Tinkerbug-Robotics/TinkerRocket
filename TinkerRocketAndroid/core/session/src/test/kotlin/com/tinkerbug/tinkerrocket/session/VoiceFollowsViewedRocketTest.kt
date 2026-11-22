package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.currentTime
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * Voice follows the computer the dashboard shows ([VoiceRocketPolicy]), not
 * only the base station's radio focus. Port of iOS
 * `VoiceFollowsViewedRocketTests`.
 *
 * 2026-11-21 (iOS): the 65 mm rocket (rocket 8) flew through the base station
 * first; the pin stayed on it, and Rolly Polly V's two computers (rocket 1 in
 * the nose, rocket 7 in the bay) flew in silence while both were on screen.
 */
class VoiceFollowsViewedRocketTest {

    // ── Policy ────────────────────────────────────────────────────────────

    private fun r(
        rid: Int,
        state: String = "PRELAUNCH",
        silentForMs: Long = 500,
        hidden: Boolean = false,
        landed: Boolean = false,
        simulated: Boolean = false,
    ) = VoiceRocketPolicy.Candidate(rid, hidden, silentForMs, state, landed, simulated)

    private fun choose(
        focus: Int?,
        vararg rockets: VoiceRocketPolicy.Candidate,
        focusHidden: Boolean = false,
        fresh: Boolean = false,
        current: Int? = null,
    ): Int? = VoiceRocketPolicy.choose(focus, focusHidden, fresh, current, rockets.toList())

    @Test
    fun followedRocketOnScreen_keepsVoice() {
        assertEquals(7, choose(7, r(1), r(7)))
    }

    /** THE regression: a pin on a rocket that has no section on screen. */
    @Test
    fun pinOnARocketNotOnScreen_voiceFollowsTheComputerShown() {
        assertEquals(1, choose(8, r(1), r(7)))
    }

    @Test
    fun freshPin_holdsVoiceUntilItsRocketIsHeard() {
        assertEquals(8, choose(8, r(1), fresh = true))
    }

    @Test
    fun freshPin_onAHiddenRocket_doesNotHold() {
        assertEquals(1, choose(8, r(1), focusHidden = true, fresh = true))
    }

    @Test
    fun hiddenFollowedRocket_voiceMovesToTheOneShown() {
        assertEquals(7, choose(1, r(1, hidden = true), r(7), focusHidden = true))
    }

    @Test
    fun quietFollowedRocket_anotherFlying_theFlightSpeaks() {
        assertEquals(1, choose(8, r(1, "INFLIGHT"), r(7, "INFLIGHT"), r(8, "INFLIGHT", silentForMs = 5_400_000)))
    }

    @Test
    fun quietFollowedRocket_nothingFlying_keepsVoice() {
        assertEquals(8, choose(8, r(1), r(8, "LANDED", silentForMs = 60_000, landed = true)))
    }

    @Test
    fun followedRocketOnThePad_anotherFlying_keepsVoice() {
        assertEquals(1, choose(1, r(1), r(2, "INFLIGHT")))
    }

    @Test
    fun twoComputersFlying_voiceStaysWhereItIs() {
        assertEquals(7, choose(8, r(1, "INFLIGHT"), r(7, "INFLIGHT"), r(8, silentForMs = 600_000), current = 7))
    }

    @Test
    fun simulatedFlight_doesNotClaimVoice() {
        assertEquals(8, choose(8, r(1, "INFLIGHT", simulated = true), r(8, "LANDED", landed = true)))
    }

    @Test
    fun rocketHeardLongAgo_losesToOneBeingHeard() {
        assertEquals(7, choose(9, r(1, silentForMs = 600_000), r(7)))
    }

    @Test
    fun everythingHidden_fallsBackToTheFocus() {
        assertEquals(1, choose(1, r(1, hidden = true), focusHidden = true))
    }

    @Test
    fun nothingHeardNothingPinned_noVoiceRocket() {
        assertNull(choose(null))
    }

    // ── Base-station dispatch ─────────────────────────────────────────────

    private class SpyAnnouncer : TelemetryAnnouncer {
        val received = mutableListOf<TelemetryData>()
        val handovers = mutableListOf<TelemetryData?>()
        override fun processTelemetry(telemetry: TelemetryData) {
            received += telemetry
        }
        override fun reset() {}
        override fun followStream(baseline: TelemetryData?) {
            handovers += baseline
        }
    }

    private class Station(val session: DeviceSession, val fw: FakeFirmware, val hidden: MutableSet<RocketKey>)

    private fun TestScope.baseStation(): Station {
        val hidden = mutableSetOf<RocketKey>()
        val fw = FakeFirmware(backgroundScope).apply { configIdentityJson = null }
        val session = DeviceSession(
            scope = backgroundScope,
            transport = fw,
            connectedDeviceName = "TR-B-Test",
            initialDeviceType = BleDeviceType.BASE_STATION,
            clock = { currentTime },
            hiddenRockets = { hidden.toSet() },
        )
        session.start()
        runCurrent()
        return Station(session, fw, hidden)
    }

    private fun FakeFirmware.relayed(rid: Int, state: String, fs: Int = 0, palt: Double = 0.0) =
        emitTelemetryJson("""{"rid":$rid,"run":"R$rid","st":"$state","fs":$fs,"palt":$palt}""")

    @Test
    fun hidingTheFollowedComputer_movesVoiceWithAHandover() = runTest {
        val bs = baseStation()
        val spy = SpyAnnouncer()
        bs.session.telemetryAnnouncer = spy

        bs.fw.relayed(1, "INFLIGHT", fs = 0x01 or 0x04, palt = 500.0)
        bs.fw.relayed(7, "INFLIGHT", fs = 0x01 or 0x04, palt = 490.0)
        runCurrent()
        assertEquals(1, bs.session.focusRocketId.value)
        assertEquals(listOf<Int?>(1), spy.received.map { it.sourceRocketId })

        bs.hidden += RocketKey(0, 1)
        bs.fw.relayed(1, "INFLIGHT", fs = 0x01 or 0x04, palt = 480.0)
        bs.fw.relayed(7, "INFLIGHT", fs = 0x01 or 0x04, palt = 470.0)
        runCurrent()

        assertEquals(1, bs.session.focusRocketId.value, "hiding never touches the radio")
        assertEquals(listOf<Int?>(1, 7), spy.received.map { it.sourceRocketId })
        assertEquals(1, spy.handovers.size)
        assertEquals(490f, spy.handovers.single()?.pressureAlt, "the baseline is rocket 7's own previous frame")
        assertTrue(spy.handovers.single()?.pastApogee == true)
        assertEquals(7, bs.session.voiceRocketId.value)
    }

    @Test
    fun follow_movesVoice() = runTest {
        val bs = baseStation()
        val spy = SpyAnnouncer()
        bs.session.telemetryAnnouncer = spy

        bs.fw.relayed(1, "PRELAUNCH")
        bs.fw.relayed(7, "PRELAUNCH")
        runCurrent()
        bs.session.setFocusRocket(7)
        runCurrent()
        bs.fw.relayed(1, "PRELAUNCH")
        bs.fw.relayed(7, "PRELAUNCH")
        runCurrent()

        assertEquals(listOf<Int?>(1, 7), spy.received.map { it.sourceRocketId })
        assertEquals(1, spy.handovers.size)
    }

    @Test
    fun routingTheSameAnnouncerAgain_isNotAHandover() = runTest {
        val bs = baseStation()
        val spy = SpyAnnouncer()
        bs.session.telemetryAnnouncer = spy
        bs.fw.relayed(1, "PRELAUNCH")
        runCurrent()
        bs.session.telemetryAnnouncer = spy
        assertEquals(1, bs.session.voiceStreamRocketId)
        bs.session.telemetryAnnouncer = SpyAnnouncer()
        assertNull(bs.session.voiceStreamRocketId, "a different announcer starts with no stream")
    }

    // ── Announcer hand-over ───────────────────────────────────────────────

    private class FakeSpeech : AnnouncerSpeech {
        val spoken = mutableListOf<String>()
        override val isBusy: Boolean = false
        override fun speak(text: String, interrupt: Boolean) {
            spoken += text
        }
        override fun stop() {}
    }

    private fun frame(
        state: String,
        fs: Int = 0,
        palt: Float = 0f,
        lat: Double? = null,
        lon: Double? = null,
    ) = TelemetryData(
        state = state,
        flightStatusBits = fs,
        pressureAlt = palt,
        maxAltM = 600f,
        latitude = lat,
        longitude = lon,
    )

    private val launch = 0x01
    private val landed = 0x08
    private val pastApogee = 0x04 or 0x400

    /** A rocket that flew earlier must not leave apogee and landing marked as
     *  said for the next one: its frames start PRELAUNCH like its baseline,
     *  so no PRELAUNCH transition would clear them. */
    @Test
    fun handover_theNextRocketsFlightIsCalledOut() {
        val speech = FakeSpeech()
        val a = FlightAnnouncer(speech = speech, initialEnabled = true, clock = { 0L })
        a.processTelemetry(frame("PRELAUNCH"))
        a.processTelemetry(frame("INFLIGHT", fs = launch or pastApogee))
        a.processTelemetry(frame("LANDED", fs = launch or pastApogee or landed))
        assertEquals(1, speech.spoken.count { it.startsWith("Apogee") })

        speech.spoken.clear()
        val pad = frame("PRELAUNCH", lat = 39.4681, lon = -75.2927)
        a.followStream(pad)
        a.processTelemetry(pad)
        a.processTelemetry(frame("INFLIGHT", fs = launch, palt = 100f))
        a.processTelemetry(frame("INFLIGHT", fs = launch or pastApogee, palt = 600f))
        a.processTelemetry(frame("INFLIGHT", fs = launch or pastApogee or landed, lat = 39.4681, lon = -75.2927))

        assertEquals(1, speech.spoken.count { it.startsWith("Apogee") }, "${speech.spoken}")
        assertEquals(1, speech.spoken.count { it.startsWith("Landed") }, "${speech.spoken}")
    }

    @Test
    fun handoverPastApogee_aTogglingVoteIsNoFreshApogee() {
        val speech = FakeSpeech()
        val a = FlightAnnouncer(speech = speech, initialEnabled = true, clock = { 0L })
        a.followStream(frame("INFLIGHT", fs = launch or pastApogee, palt = 400f))
        a.processTelemetry(frame("INFLIGHT", fs = launch or 0x400, palt = 390f))
        a.processTelemetry(frame("INFLIGHT", fs = launch or pastApogee, palt = 380f))
        assertTrue(speech.spoken.none { it.startsWith("Apogee") }, "${speech.spoken}")
    }

    @Test
    fun handoverLanded_isNoFreshLanding() {
        val speech = FakeSpeech()
        val a = FlightAnnouncer(speech = speech, initialEnabled = true, clock = { 0L })
        a.followStream(frame("LANDED", fs = launch or pastApogee or landed))
        a.processTelemetry(frame("LANDED", fs = launch or pastApogee or landed))
        assertTrue(speech.spoken.isEmpty(), "${speech.spoken}")
    }
}
