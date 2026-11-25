package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonPrimitive
import java.io.File
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue
import kotlin.test.fail

/**
 * Kotlin base-station bin->CSV against the committed golden.
 *
 * The golden's INPUT (tests_cpp/fixtures/wire/csv/bs_tiny.bin) is packed by
 * wire_fixture_gen using the REAL firmware LoRa packers, so this is not three
 * ports agreeing with each other — it is three ports agreeing with the
 * firmware.  That distinction is the whole reason the fixture exists: a
 * round-trip test would happily pass while every implementation shared the same
 * wrong idea of the layout.
 *
 * bs_tiny.bin is a LoRa protocol v6 log (23-byte SLOW frames, status 1) and
 * bs_tiny_v5.bin the same log as a v5 rocket sent it (22-byte SLOW frames, no
 * status byte).  Both render the ONE golden: the status byte adds no column.
 */
class BsLogCsvGoldenTest {
    private val goldenDir = File(WireFixtures.root.parentFile, "csv_golden")

    private fun golden() = File(goldenDir, "bs_tiny.expected.csv").readText()

    private fun assertMatchesGolden(rel: String) {
        val csv = BsLogCsvGenerator().writeCsv(WireFixtures.bytes(rel))
        val g = golden()
        if (csv != g) {
            val a = csv.split("\n")
            val b = g.split("\n")
            for (i in 0 until maxOf(a.size, b.size)) {
                val al = a.getOrNull(i) ?: "<missing>"
                val bl = b.getOrNull(i) ?: "<missing>"
                if (al != bl) fail("$rel: CSV differs at line ${i + 1}:\n generated: $al\n golden:    $bl")
            }
        }
        assertEquals(g, csv, rel)
    }

    /**
     * The LoRa frame of every BS_LORA_RX_MSG record, in order, as (offset in
     * [b], length).  Record layout: preamble(4) type(1) len(1) payload(len)
     * trailer(2); the frame starts 12 bytes into the payload.
     */
    private fun loraFrameOffsets(b: ByteArray): List<Pair<Int, Int>> {
        val out = ArrayList<Pair<Int, Int>>()
        var i = 0
        while (i + 6 <= b.size) {
            if (b[i] != 0xAA.toByte() || b[i + 1] != 0x55.toByte() ||
                b[i + 2] != 0xAA.toByte() || b[i + 3] != 0x55.toByte()
            ) {
                i++
                continue
            }
            val type = b[i + 4].toInt() and 0xFF
            val len = b[i + 5].toInt() and 0xFF
            if (type == 0xFC && len > 12) out.add(Pair(i + 6 + 12, len - 12))
            i += 6 + len + 2
        }
        return out
    }

    @Test
    fun `base station CSV matches the committed golden byte-for-byte`() {
        assertMatchesGolden("csv/bs_tiny.bin")
    }

    @Test
    fun `a v5 rocket's log renders the same golden`() {
        assertMatchesGolden("csv/bs_tiny_v5.bin")
    }

    @Test
    fun `the two fixtures are the protocol versions their sidecars claim`() {
        for (rel in listOf("csv/bs_tiny.bin", "csv/bs_tiny_v5.bin")) {
            val side = WireFixtures.sidecar(rel)
            val proto = side["lora_proto"]!!.jsonPrimitive.int
            val slowLen = side["slow_frame_len"]!!.jsonPrimitive.int
            val bin = WireFixtures.bytes(rel)
            val frames = loraFrameOffsets(bin)
            assertEquals(side["rx_records"]!!.jsonPrimitive.int, frames.size, rel)
            for ((off, len) in frames) {
                val verType = bin[off + 5].toInt() and 0xFF
                assertEquals(proto, verType shr 4, "$rel: version nibble")
                val want = if ((verType and 0x0F) == BsLogCsvGenerator.LORA_FRAME_SLOW) slowLen else 55
                assertEquals(want, len, "$rel: frame length")
            }
        }
    }

    @Test
    fun `v6 status byte decodes into the row and a v5 slow frame reads as no board`() {
        for (rel in listOf("csv/bs_tiny.bin", "csv/bs_tiny_v5.bin")) {
            val side = WireFixtures.sidecar(rel)
            val (rows, _) = BsLogCsvGenerator().parse(WireFixtures.bytes(rel))
            assertEquals(side["fast_frames"]!!.jsonPrimitive.int, rows.count { it.frame == "fast" }, rel)
            assertEquals(side["slow_frames"]!!.jsonPrimitive.int, rows.count { it.frame == "slow" }, rel)
            val firstSlow = rows.indexOfFirst { it.frame == "slow" }
            assertTrue(firstSlow > 0, "$rel: a fast frame must lead the slow one")
            assertTrue(rows.take(firstSlow).all { it.gnssImu == 0 && it.mount == 0 }, "$rel: nothing before the first slow frame")
            // Set by the slow frame and forward-filled onto the fast rows after it.
            val want = side["gnss_imu"]!!.jsonPrimitive.int
            assertTrue(rows.drop(firstSlow).all { it.gnssImu == want }, "$rel: gnss_imu $want from the slow frame on")
            // #1755: bits 2-3 of the same byte, each read apart from the other.
            val wantMount = side["mount"]!!.jsonPrimitive.int
            assertTrue(rows.drop(firstSlow).all { it.mount == wantMount }, "$rel: mount $wantMount from the slow frame on")
        }
    }

    @Test
    fun `a slow frame at the other version's length is skipped`() {
        // v6 header on a 22-byte slow frame, and v5 header on a 23-byte one:
        // rewrite the version nibble of each slow frame in the other fixture.
        for ((rel, wrongVersion) in listOf("csv/bs_tiny_v5.bin" to 6, "csv/bs_tiny.bin" to 5)) {
            val bin = WireFixtures.bytes(rel)
            var rewritten = 0
            for ((off, _) in loraFrameOffsets(bin)) {
                val verType = bin[off + 5].toInt() and 0xFF
                if ((verType and 0x0F) != BsLogCsvGenerator.LORA_FRAME_SLOW) continue
                bin[off + 5] = ((wrongVersion shl 4) or (verType and 0x0F)).toByte()
                rewritten++
            }
            assertEquals(2, rewritten, "$rel: the fixture's shape changed; recheck the rewrite")

            val (rows, events) = BsLogCsvGenerator().parse(bin)
            assertEquals(10, rows.size, "$rel: only the ten fast frames survive")
            assertTrue(rows.all { it.frame == "fast" }, rel)
            // Nothing from a skipped frame leaks into the forward-fill.
            assertTrue(rows.all { it.camA == 0.0 && it.maxAlt == 0.0 && it.gnssImu == 0 && it.mount == 0 }, rel)
            assertEquals(1, events.size, rel)
            // The CSV still renders, header plus 10 rows plus the event.
            assertEquals(12, BsLogCsvGenerator().writeCsv(bin).trim().split("\n").size, rel)
        }
    }

    @Test
    fun `header is the firmware columns with frame appended`() {
        val cols = BsLogCsvGenerator.COLUMNS
        assertEquals(40, cols.size, "39 firmware columns + frame")
        assertEquals("time_ms", cols.first())
        assertEquals("frame", cols.last(), "frame is APPENDED so name-keyed consumers are unaffected")
        assertEquals("rocket_id", cols[cols.size - 2], "the firmware's last column stays last-but-one")
    }

    @Test
    fun `forward fill carries slow fields onto fast rows and back`() {
        val bin = WireFixtures.bytes("csv/bs_tiny.bin")
        val csv = BsLogCsvGenerator().writeCsv(bin)
        val lines = csv.trim().split("\n")
        val cols = lines[0].split(",")
        val rows = lines.drop(1).map { it.split(",") }.filter { it[1] != "EVENT" }

        val iCam = cols.indexOf("cam_a")
        val iPalt = cols.indexOf("pressure_alt")
        val iFrame = cols.indexOf("frame")

        val firstSlow = rows.indexOfFirst { it[iFrame] == "slow" }
        assertTrue(firstSlow > 0, "the fixture must contain a slow frame after a fast one")

        // The slow row keeps the position it never carried...
        assertTrue(rows[firstSlow][iPalt].toDouble() > 0.0,
            "a slow frame must not blank the position")
        // ...and the next fast row keeps the battery it never carried.
        val nextFast = rows.drop(firstSlow + 1).first { it[iFrame] == "fast" }
        assertEquals("1.480", nextFast[iCam],
            "a fast frame must not blank the rail current from the preceding slow frame")
    }

    @Test
    fun `unknown rssi renders as nan not as zero dBm`() {
        val bin = WireFixtures.bytes("csv/bs_tiny.bin")
        val csv = BsLogCsvGenerator().writeCsv(bin)
        val lines = csv.trim().split("\n")
        val iRssi = lines[0].split(",").indexOf("rssi")
        assertTrue(lines.drop(1).any { it.split(",")[iRssi] == "nan" },
            "the fixture carries the INT16_MIN sentinel; 0 dBm is a legal reading and " +
                "must not be how 'no reading' renders")
    }

    @Test
    fun `events are interleaved by time`() {
        val bin = WireFixtures.bytes("csv/bs_tiny.bin")
        val csv = BsLogCsvGenerator().writeCsv(bin)
        val lines = csv.trim().split("\n").drop(1)
        val times = lines.map { it.split(",")[0].toLong() }
        assertEquals(times.sorted(), times, "rows and events must be in arrival order")
        assertTrue(lines.any { it.split(",")[1] == "EVENT" })
    }

    @Test
    fun `a rocket log is refused rather than parsed as garbage`() {
        val rocket = WireFixtures.bytes("csv/tiny_flight.bin")
        assertFailsWith<BsLogCsvGenerator.NotABaseStationLogException> {
            BsLogCsvGenerator().writeCsv(rocket)
        }
    }

    @Test
    fun `detection is by magic`() {
        assertTrue(BsLogCsvGenerator.isBaseStationLog(WireFixtures.bytes("csv/bs_tiny.bin")))
        assertTrue(!BsLogCsvGenerator.isBaseStationLog(WireFixtures.bytes("csv/tiny_flight.bin")))
    }
}
