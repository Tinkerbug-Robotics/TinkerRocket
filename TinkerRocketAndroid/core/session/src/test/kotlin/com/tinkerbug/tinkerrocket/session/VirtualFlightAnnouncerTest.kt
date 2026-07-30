package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue
import kotlin.test.fail

/**
 * Replays the EXACT Virtual Rocket frames ([VirtualFlightScript]) through the
 * real announcer policy — the regression for the 2026-07-30 voice-callout
 * outage, where the script spoke a dialect of firmware the announcer's gates
 * couldn't hear:
 *  - "DESCENT" isn't a firmware state (descent = INFLIGHT + alt_apo), so
 *    every descent callout was skipped;
 *  - `mspd` was never fed, so burnout never announced — and ascent altitude
 *    callouts are gated behind burnout;
 *  - no PRELAUNCH countdown, so the one-shot flags never reset and a SECOND
 *    demo flight was completely mute.
 *
 * The script is replayed twice (with a READY idle frame between, as the demo
 * loop does) and must produce the identical callout sequence both times.
 * iOS twin: `VirtualRocketTests.testVirtualFlightScript_drivesFullCalloutSequence`.
 */
class VirtualFlightAnnouncerTest {

    private class FakeSpeech : AnnouncerSpeech {
        val spoken = mutableListOf<String>()
        override var isBusy: Boolean = false
        override fun speak(text: String, interrupt: Boolean) {
            spoken += text
        }
        override fun stop() = Unit
    }

    private fun flyOnce(announcer: FlightAnnouncer, clock: (Long) -> Unit) {
        for (tick in 0 until VirtualFlightScript.TICKS) {
            clock(tick * VirtualFlightScript.TICK_MS)
            val frame = TelemetryData.decode(VirtualFlightScript.frameJson(tick))
                ?: fail("script frame $tick did not decode")
            announcer.processTelemetry(frame)
        }
    }

    @Test
    fun `virtual flight drives the full callout sequence, twice`() {
        val speech = FakeSpeech()
        var nowMs = 0L
        val announcer = FlightAnnouncer(
            speech = speech,
            initialEnabled = true,
            clock = { nowMs },
        )

        val idleReady = TelemetryData.decode(
            """{"rid":1,"run":"Booster","st":"READY","fs":16,"palt":0.2}""",
        ) ?: fail("idle frame did not decode")

        var flightBase = 0L
        repeat(2) { flight ->
            val startCount = speech.spoken.size
            flyOnce(announcer) { offset -> nowMs = flightBase + offset }
            val calls = speech.spoken.drop(startCount)

            // One-shots: exactly one each, in flight order.
            assertEquals(1, calls.count { it.startsWith("Burnout") }, "flight $flight: $calls")
            assertEquals(1, calls.count { it.startsWith("Apogee") }, "flight $flight: $calls")
            assertEquals(1, calls.count { it.startsWith("Landed") }, "flight $flight: $calls")
            assertTrue(
                calls.first().startsWith("Burnout"),
                "nothing may speak before burnout (flight $flight): $calls",
            )
            assertTrue(calls.last().startsWith("Landed"), "flight $flight: $calls")

            // Apogee altitude reaches the callout (script max = 400 m).
            val apogee = calls.first { it.startsWith("Apogee") }
            assertTrue("400" in apogee, apogee)

            // Descent cadence: first 5 s after apogee, then every 10 s — the
            // 16 s descent yields exactly two, both saying "descending".
            val descent = calls.filter { "descending" in it }
            assertEquals(2, descent.size, "flight $flight: $calls")

            // Ascent altitude cadence between burnout and apogee, climbing.
            val apogeeIx = calls.indexOfFirst { it.startsWith("Apogee") }
            val ascent = calls.subList(1, apogeeIx)
            assertTrue(ascent.isNotEmpty(), "flight $flight: $calls")
            assertTrue(ascent.all { "climbing" in it }, "flight $flight: $calls")

            // The drift under canopy gives the landed callout a distance.
            assertTrue("away" in calls.last(), calls.last())

            // Back to the READY idle, then the next flight's PRELAUNCH edge
            // must reset the one-shots (the second-flight-mute regression).
            nowMs = flightBase + VirtualFlightScript.TICKS * VirtualFlightScript.TICK_MS
            announcer.processTelemetry(idleReady)
            flightBase = nowMs + 5_000
        }
    }
}
