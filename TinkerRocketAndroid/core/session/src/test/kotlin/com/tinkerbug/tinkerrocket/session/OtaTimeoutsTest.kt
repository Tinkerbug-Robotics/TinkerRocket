package com.tinkerbug.tinkerrocket.session

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long
import java.io.File
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue
import kotlin.test.fail

/**
 * The OTA timing contract, checked two ways.
 *
 * [everyRelayCrossingStageIsStretchedOnTheFcPath] is the one that matters:
 * it states the property the numbers have to satisfy rather than the numbers
 * themselves, so it catches a NEW stage added without an FC window as well as
 * the bug it was written for.  Bench 2026-07-28 found `FINISH` flat at 15 s on
 * both paths — it expired while a live 591.7 kB FC flash was still running, so
 * the app declared failure on a transfer that then succeeded (#627).  A plain
 * Swift↔Kotlin diff would NOT have caught that: both platforms said 15 s and
 * agreed with each other.
 *
 * [matchesTheSharedFixture] is the anti-drift half — iOS asserts the same file,
 * so changing one platform's table without the other fails that build.
 */
class OtaTimeoutsTest {

    private fun fixture(): JsonObject {
        // core/session/src/test/kotlin/... → repo root → tests_cpp/fixtures
        val f = File(System.getProperty("tr.appBehaviorFixtures") ?: "")
            .resolve("ota_timeouts.json")
        if (!f.isFile) fail("missing shared fixture at ${f.absolutePath}")
        return Json.parseToJsonElement(f.readText()) as JsonObject
    }

    @Test
    fun everyRelayCrossingStageIsStretchedOnTheFcPath() {
        for (stage in OtaStage.entries) {
            val local = OtaTimeouts.millis(stage, targetIsFc = false)
            val fc = OtaTimeouts.millis(stage, targetIsFc = true)
            if (stage.crossesRelay) {
                assertTrue(
                    fc > local,
                    "$stage waits on the FC over the relay, so the FC window " +
                        "must exceed the local one (was fc=$fc local=$local)",
                )
            } else {
                assertEquals(
                    local, fc,
                    "$stage is a local link event — the relay plays no part, " +
                        "so both paths must wait the same",
                )
            }
        }
    }

    @Test
    fun matchesTheSharedFixture() {
        val json = fixture()
        val stages = json["stages"]!!.jsonArray.map { it as JsonObject }
        assertEquals(
            OtaStage.entries.size, stages.size,
            "fixture lists ${stages.size} stages but Kotlin has ${OtaStage.entries.size}",
        )
        for (s in stages) {
            val name = s["name"]!!.jsonPrimitive.content
            val stage = OtaStage.entries.firstOrNull { it.wireName == name }
                ?: fail("fixture stage '$name' has no Kotlin OtaStage")
            assertEquals(
                s["crossesRelay"]!!.jsonPrimitive.content.toBoolean(), stage.crossesRelay,
                "$name crossesRelay disagrees with the fixture",
            )
            assertEquals(
                s["localMs"]!!.jsonPrimitive.long, OtaTimeouts.millis(stage, false),
                "$name local window disagrees with the fixture",
            )
            assertEquals(
                s["fcMs"]!!.jsonPrimitive.long, OtaTimeouts.millis(stage, true),
                "$name FC window disagrees with the fixture",
            )
        }
        assertEquals(json["pollMs"]!!.jsonPrimitive.long, OtaTimeouts.POLL_MS)
        assertEquals(
            json["fcRelayMaxBytesPerSec"]!!.jsonPrimitive.long,
            OtaTimeouts.FC_RELAY_MAX_BYTES_PER_SEC,
            "#627 relay cap disagrees with the fixture",
        )
    }

    @Test
    fun fcRelayPacerHoldsTheCapAndCreditsElapsedTime() {
        val rate = OtaTimeouts.FC_RELAY_MAX_BYTES_PER_SEC

        // One second's worth of bytes, no time spent yet → wait the full second.
        assertEquals(1000L, OtaTimeouts.fcRelayPaceDelayMs(rate, elapsedMs = 0))

        // Same bytes, but the writes themselves already took 400 ms — credit
        // that instead of adding to it, or the effective rate drifts under the
        // cap and a 591.7 kB FC image takes far longer than the 60 s finish
        // window assumes.
        assertEquals(600L, OtaTimeouts.fcRelayPaceDelayMs(rate, elapsedMs = 400))

        // Already slower than the cap → never wait, never go backwards.
        assertEquals(0L, OtaTimeouts.fcRelayPaceDelayMs(rate, elapsedMs = 1000))
        assertEquals(0L, OtaTimeouts.fcRelayPaceDelayMs(rate, elapsedMs = 5000))

        // The cap has to stay under what the OC can drain, and the bench
        // measured where that is (#811, 2026-09-10, 647,472 B over the relay):
        //
        //   12 KB/s  53.0 s  qdepth peak 2                 clean
        //   20 KB/s  31.3 s  qdepth peak 3                 clean   <- shipped
        //   30 KB/s  20.7 s  qdepth 13 at startup only     clean
        //   40 KB/s  15.2 s  qdepth 16 sustained, 4 ACL alloc failures
        //
        // The lower bound is the rate iOS was already proven to run at, so
        // this can never regress below something known to work. The upper
        // bound is where the OC's 16-frame feed queue saturates and NimBLE
        // starts failing ACL allocations — the #627 wedge, reproduced
        // deliberately at 40. Anything in between is a judgement about margin.
        assertTrue(
            rate in 11_000..30_000,
            "cap must stay between iOS's proven-good rate and the measured " +
                "point where the OC's feed queue saturates, was $rate B/s",
        )
    }

    @Test
    fun sessionConstantsAreViewsOntoTheTable() {
        // The companion aliases are convenience only; if they ever drift from
        // the table the fixture check above would pass while the flow used
        // different numbers.
        assertEquals(OtaTimeouts.millis(OtaStage.BEGIN, false), OtaSession.BEGIN_TIMEOUT_MS)
        assertEquals(OtaTimeouts.millis(OtaStage.BEGIN, true), OtaSession.BEGIN_TIMEOUT_FC_MS)
        assertEquals(OtaTimeouts.millis(OtaStage.FINISH, false), OtaSession.FINISH_TIMEOUT_MS)
        assertEquals(OtaTimeouts.millis(OtaStage.FINISH, true), OtaSession.FINISH_TIMEOUT_FC_MS)
        assertEquals(OtaTimeouts.millis(OtaStage.DISCONNECT, false), OtaSession.DISCONNECT_TIMEOUT_MS)
        assertEquals(OtaTimeouts.millis(OtaStage.RECONNECT, false), OtaSession.RECONNECT_TIMEOUT_MS)
        assertEquals(OtaTimeouts.millis(OtaStage.FW_PUBLISH, false), OtaSession.FW_TIMEOUT_MS)
        assertEquals(OtaTimeouts.millis(OtaStage.FW_PUBLISH, true), OtaSession.FW_TIMEOUT_FC_MS)
        assertEquals(OtaTimeouts.POLL_MS, OtaSession.POLL_MS)
    }
}
