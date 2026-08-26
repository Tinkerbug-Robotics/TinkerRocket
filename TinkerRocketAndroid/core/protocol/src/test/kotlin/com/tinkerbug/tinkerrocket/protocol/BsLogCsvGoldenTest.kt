package com.tinkerbug.tinkerrocket.protocol

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
 */
class BsLogCsvGoldenTest {
    private val goldenDir = File(WireFixtures.root.parentFile, "csv_golden")

    private fun golden() = File(goldenDir, "bs_tiny.expected.csv").readText()

    @Test
    fun `base station CSV matches the committed golden byte-for-byte`() {
        val bin = WireFixtures.bytes("csv/bs_tiny.bin")
        val csv = BsLogCsvGenerator().writeCsv(bin)
        val g = golden()
        if (csv != g) {
            val a = csv.split("\n")
            val b = g.split("\n")
            for (i in 0 until maxOf(a.size, b.size)) {
                val al = a.getOrNull(i) ?: "<missing>"
                val bl = b.getOrNull(i) ?: "<missing>"
                if (al != bl) fail("CSV differs at line ${i + 1}:\n generated: $al\n golden:    $bl")
            }
        }
        assertEquals(g, csv)
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
