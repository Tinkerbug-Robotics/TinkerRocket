package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertSame
import kotlin.test.assertTrue

/**
 * Port of the iOS CSVParserTests (#236 over-long-row clamp, #514 split-header
 * repair) with identical inputs and expectations, plus Kotlin-only pins for
 * the strict field parsing that mirrors Swift `Double(Substring)` semantics.
 *
 * #236: `CSVParser.parse` trapped with "Range requires lowerBound <=
 * upperBound" on LoRa logs whose corrupted-telemetry rows have MORE comma
 * fields than the header (e.g. a garbled value like "166.65.5" splitting into
 * extra fields).  The padding loop `fields.count..<columnCount` inverted when
 * `fields.count > columnCount`.  Parsing must be crash-proof and column-aligned.
 */
class CsvParserTest {

    private fun parse(csv: String): FlightCsvData = CsvParser.parse(csv)

    // MARK: iOS CSVParserTests ports

    /** The exact #236 crash: a row with MORE fields than the header. */
    @Test
    fun `row with extra fields does not trap`() {
        val data = parse("a,b,c\n1,2,3\n4,5,6,7,8\n9,10,11\n")
        assertEquals(3, data.rowCount)
        // Every column stays aligned (one value per row); extra fields ignored.
        assertEquals(listOf(1.0, 4.0, 9.0), data.columns["a"])
        assertEquals(listOf(2.0, 5.0, 10.0), data.columns["b"])
        assertEquals(listOf(3.0, 6.0, 11.0), data.columns["c"])
    }

    /** Rows with FEWER fields than the header still pad with NaN. */
    @Test
    fun `row with missing fields pads NaN`() {
        val data = parse("a,b,c\n1,2,3\n4,5\n")
        assertEquals(2, data.rowCount)
        assertEquals(listOf(1.0, 4.0), data.columns["a"])
        assertEquals(listOf(2.0, 5.0), data.columns["b"])
        val c = data.columns["c"]!!
        assertEquals(3.0, c[0])
        assertEquals(2, c.size)
        assertTrue(c[1].isNaN())   // padded
    }

    /** Mirrors the real LoRa shape: 36-col header with a 38-field garbled row. */
    @Test
    fun `LoRa shaped garbled row does not trap`() {
        val header = (0 until 36).joinToString(",") { "c$it" }
        val good = (0 until 36).joinToString(",") { "$it" }
        val garbled = "$good,166.65.5,extra"   // 38 fields
        val data = parse("$header\n$good\n$garbled\n")
        assertEquals(2, data.rowCount)
        assertEquals(listOf(0.0, 0.0), data.columns["c0"])
        assertEquals(2, data.columns["c35"]?.size)           // all columns aligned
    }

    /**
     * Exports from 2026-07-14…07-16 app builds have unquoted commas in the
     * three #514 attitude names, so the header row carries three more tokens
     * than every data row and all later columns load shifted.  The parser must
     * re-join those tokens into the current semicolon names so the values
     * land under the right columns.
     */
    @Test
    fun `split attitude header names are repaired and aligned`() {
        val header = "Quat q3,Roll (deg, body-Z azimuth),Pitch (deg, ZYX Euler)," +
            "Yaw (deg, ZYX Euler),Roll Command (deg),Position Up (m)"
        val data = parse("$header\n0.5,10,20,30,0.04,407.2\n")
        assertEquals(
            listOf(
                "Quat q3",
                "Roll (deg; body-Z azimuth)", "Pitch (deg; ZYX Euler)", "Yaw (deg; ZYX Euler)",
                "Roll Command (deg)", "Position Up (m)",
            ),
            data.headers,
        )
        assertEquals(listOf(0.5), data.columns["Quat q3"])
        assertEquals(listOf(10.0), data.columns["Roll (deg; body-Z azimuth)"])
        assertEquals(listOf(0.04), data.columns["Roll Command (deg)"])
        assertEquals(listOf(407.2), data.columns["Position Up (m)"])   // not shifted
    }

    /**
     * Headers without the broken comma names pass through untouched — a
     * lone "Roll (deg)" (pre-#514) or the current semicolon names must not
     * trigger the repair.
     */
    @Test
    fun `current and legacy headers are not rewritten`() {
        val data = parse("Roll (deg; body-Z azimuth),Roll (deg),Yaw (deg)\n1,2,3\n")
        assertEquals(
            listOf("Roll (deg; body-Z azimuth)", "Roll (deg)", "Yaw (deg)"),
            data.headers,
        )
        assertEquals(listOf(2.0), data.columns["Roll (deg)"])
    }

    // MARK: Kotlin-port pins (no iOS twin — behavior inherited from Swift stdlib)

    @Test
    fun `empty content throws EmptyFile and header-only parses zero rows`() {
        assertFailsWith<CsvParserException.EmptyFile> { parse("") }
        assertFailsWith<CsvParserException.EmptyFile> { parse("\n\n") }
        val data = parse("a,b\n")
        assertEquals(0, data.rowCount)
        assertEquals(listOf<Double>(), data.columns["a"])
    }

    /** A header line with only commas yields zero column names → NoHeader. */
    @Test
    fun `comma-only header throws NoHeader`() {
        assertFailsWith<CsvParserException.NoHeader> { parse(",,,\n1,2,3\n") }
    }

    /** Header split drops empty tokens and trims blanks (Swift split default + .whitespaces). */
    @Test
    fun `header empty tokens dropped and names trimmed`() {
        val data = parse("a, b ,,c\n1,2,3\n")
        assertEquals(listOf("a", "b", "c"), data.headers)
        // Data split KEEPS empties, so 3 fields map to the 3 headers directly.
        assertEquals(listOf(1.0), data.columns["a"])
        assertEquals(listOf(3.0), data.columns["c"])
    }

    /** Empty data fields become NaN (data rows keep empty subsequences). */
    @Test
    fun `empty data field becomes NaN`() {
        val data = parse("a,b,c\n1,,3\n")
        assertTrue(data.columns["b"]!![0].isNaN())
        assertEquals(listOf(1.0), data.columns["a"])
        assertEquals(listOf(3.0), data.columns["c"])
    }

    /**
     * Swift `Double(Substring)` rejects whitespace-padded fields and Java-only
     * spellings; garbage-row forms Kotlin's toDoubleOrNull would otherwise
     * accept ("NaN", "Infinity", hex floats, "1.0d", " 1") must all land on
     * NaN, matching the iOS column outcome.
     */
    @Test
    fun `non plain-decimal fields map to NaN`() {
        val garbage = listOf("NaN", "Infinity", "-Infinity", "0x1p3", "0x1.8p2", "1.0d", "1.0f", " 1", "1 ", "abc", "166.65.5", "--1", "1e", "e5")
        val header = (garbage.indices).joinToString(",") { "c$it" }
        val data = parse("$header\n${garbage.joinToString(",")}\n")
        for (i in garbage.indices) {
            assertTrue(data.columns["c$i"]!![0].isNaN(), "field '${garbage[i]}' must be NaN")
        }
        // Plain decimal and scientific forms still parse.
        val ok = parse("a,b,c,d\n-1.5,2e3,+4,.5\n")
        assertEquals(listOf(-1.5), ok.columns["a"])
        assertEquals(listOf(2000.0), ok.columns["b"])
        assertEquals(listOf(4.0), ok.columns["c"])
        assertEquals(listOf(0.5), ok.columns["d"])
    }

    /** Dialect detection: LoRa short names select the LoRa groups. */
    @Test
    fun `column groups selected by header dialect`() {
        assertSame(
            FlightCsvData.loraColumnGroups,
            FlightCsvData.columnGroupsFor(listOf("time", "pressure_alt", "speed")),
        )
        assertSame(
            FlightCsvData.loraColumnGroups,
            FlightCsvData.columnGroupsFor(listOf("rssi", "snr")),
        )
        assertSame(
            FlightCsvData.columnGroups,
            FlightCsvData.columnGroupsFor(listOf("Pressure Altitude (m)", "Quat q0")),
        )
        // Rotation group must list BOTH current semicolon and pre-#514 names.
        val rotation = FlightCsvData.columnGroups.first { it.name == "Rotation" }.columns
        assertTrue(rotation.contains("Roll (deg; body-Z azimuth)"))
        assertTrue(rotation.contains("Roll (deg)"))
    }

    /** repairSplitHeaderNames is exact: an unpaired first token stays split. */
    @Test
    fun `repair requires the exact second token`() {
        assertEquals(
            listOf("Roll (deg", "wrong)"),
            CsvParser.repairSplitHeaderNames(listOf("Roll (deg", "wrong)")),
        )
        assertEquals(
            listOf("Roll (deg"),
            CsvParser.repairSplitHeaderNames(listOf("Roll (deg")),
        )
    }

    // ── #636: large-input handling ───────────────────────────────────────

    @Test
    fun streamingOverloadMatchesTheStringOverload() {
        val csv = buildString {
            append("a,b,c\n")
            repeat(500) { r -> append("$r,${r * 2},${r * 3}\n") }
        }
        val fromString = CsvParser.parse(csv)
        val fromStream = CsvParser.parse(csv.splitToSequence("\n"))

        assertEquals(fromString.headers, fromStream.headers)
        assertEquals(fromString.rowCount, fromStream.rowCount)
        for (h in fromString.headers) {
            assertEquals(fromString.columns[h], fromStream.columns[h], "column $h")
        }
    }

    @Test
    fun streamingParseHandlesAFlightSizedCsvWithoutHoldingItAllInMemory() {
        // The shape that crashed the app: a real 63-column flight CSV.  Parsed
        // as one String this is the 134 MB allocation from #636; streamed, peak
        // memory is the columns alone.  Sized to stay quick in CI while still
        // being far larger than any fixture the golden corpus carries.
        val cols = 63
        val rows = 20_000
        val header = (0 until cols).joinToString(",") { "col$it" }
        val lines = sequence {
            yield(header)
            repeat(rows) { r ->
                yield((0 until cols).joinToString(",") { c -> "${r * cols + c}.5" })
            }
        }

        val data = CsvParser.parse(lines)

        assertEquals(cols, data.headers.size)
        assertEquals(rows, data.rowCount)
        assertEquals(rows, data.columns["col0"]?.size)
        // Spot-check that values land in the right column, not merely that it parsed.
        assertEquals(0.5, data.columns["col0"]!![0])
        assertEquals(62.5, data.columns["col62"]!![0])
        assertEquals((cols + 0) + 0.5, data.columns["col0"]!![1])
    }
}
