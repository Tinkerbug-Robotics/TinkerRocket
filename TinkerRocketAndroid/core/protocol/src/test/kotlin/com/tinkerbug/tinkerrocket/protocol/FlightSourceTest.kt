package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * #1658: a downloaded flight records which rocket and computer it came from.
 * The summary gains `source` (filled at download) and `settings.board` (from
 * the log); both read back on their own, like the headline numbers (#1077).
 * Keys match iOS FlightSourceTests.
 */
class FlightSourceTest {

    private val full = FlightSource(
        unitId = "a1cae150", unitName = "Atlas", rocketProfile = "Piercer Mini", rocketId = 2,
        fcBoard = "V5", ocBoard = "V5", fcFw = "41b2ab92", ocFw = "41b2ab92",
        fcGnss = "u-blox", downloadedAt = "2026-10-24T18:03:00Z",
    )

    @Test
    fun `unknown fields are left out, not nulled`() {
        val obj = FlightSource(unitId = "a1cae150", ocBoard = "V5").toJsonObject()
        assertEquals(setOf("unit_id", "oc_board"), obj.keys)
    }

    @Test
    fun `source rides the summary after settings and round-trips`() {
        val bare = FlightSummary(912.0, 173.0, 1.3, 8.9, settings = null)
        assertFalse("source" in bare.toJsonObject().keys, "no device, no source")

        val text = bare.copy(source = full).toJson()
        assertEquals(
            listOf("apogee_time_s", "burnout_time_s", "max_altitude_m", "max_speed_mps", "source"),
            (Json.parseToJsonElement(text) as JsonObject).keys.toList(),
        )
        assertEquals(full, FlightSummary.sourceFromJson(text))
        assertEquals(912.0, FlightSummary.fromJson(text)?.maxAltitudeM, "headline unaffected")
    }

    @Test
    fun `board reads from settings`() {
        val text = """{ "settings" : { "board" : "M1 (asserted)", "fw_git_sha" : "abc" } }"""
        assertEquals("M1 (asserted)", FlightSummary.boardFromJson(text))
    }

    @Test
    fun `an old sidecar has neither, and that is not an error`() {
        val text = """{ "max_altitude_m" : 311.0, "settings" : { "fw_git_sha" : "abc" } }"""
        assertNull(FlightSummary.sourceFromJson(text))
        assertNull(FlightSummary.boardFromJson(text))
        assertEquals(311.0, FlightSummary.fromJson(text)?.maxAltitudeM)
    }

    @Test
    fun `malformed blocks read as absent`() {
        assertNull(FlightSummary.sourceFromJson("""{ "source" : "Atlas" }"""))
        assertNull(FlightSummary.boardFromJson("""{ "settings" : { "board" : 9 } }"""))
        assertNull(FlightSummary.sourceFromJson("not json"))
        // A field of the wrong type is dropped; the rest of the block survives.
        val partial = FlightSummary.sourceFromJson("""{ "source" : { "unit_id" : 7, "unit_name" : "Atlas" } }""")
        assertEquals(FlightSource(unitName = "Atlas"), partial)
    }

    @Test
    fun `the label is the profile, else the unit name`() {
        assertEquals("Piercer Mini", full.rocketLabel)
        assertEquals("Atlas", full.copy(rocketProfile = null).rocketLabel)
        assertNull(FlightSource().rocketLabel)
    }

    @Test
    fun `the generator writes the log's board into settings`() {
        val (_, summary) = CsvGenerator().writeCsv(WireFixtures.bytes("csv/tiny_flight.bin"))
        assertEquals("V9", summary.settings?.board)
        assertNull(summary.source, "the converter never fills source")
    }

    @Test
    fun `firstFrame stops at the first settings frame`() {
        val bin = WireFixtures.bytes("csv/tiny_flight.bin")
        val frame = assertNotNull(MessageParser.firstFrame(bin, MsgType.FLIGHT_SETTINGS))
        val first = MessageParser.parse(bin).first { it.type == MsgType.FLIGHT_SETTINGS }
        assertTrue(frame.payload.contentEquals(first.payload))
        assertEquals("V9", FlightSettingsData.decode(frame.payload)?.boardRevName)
        assertNull(MessageParser.firstFrame(bin, 0xFE), "absent type")
    }
}
