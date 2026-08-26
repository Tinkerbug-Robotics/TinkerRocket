package com.tinkerbug.tinkerrocket.protocol

// Port of TinkerRocketApp/Models/CSVParser.swift (parse + header repair +
// column groups).  The iOS source is the behavioral reference — including the
// #236 over-long-row clamp and the #514 split-header repair — so any change
// here must stay lockstep with the Swift original.

/** One named group of column headers for the chart column picker UI. */
public data class ColumnGroup(
    public val name: String,
    public val columns: List<String>,
)

/**
 * Parsed CSV flight data in columnar format for efficient charting.
 * Mirrors iOS `FlightCSVData`.
 */
public class FlightCsvData(
    public val headers: List<String>,
    public val columns: Map<String, List<Double>>,
    public val rowCount: Int,
) {
    public companion object {

        /**
         * Column names grouped by sensor category for the picker UI (rocket
         * on-board CSV).  Mirrors iOS `FlightCSVData.columnGroups`.
         */
        public val columnGroups: List<ColumnGroup> = listOf(
            ColumnGroup("Altitude & Position", listOf(
                "Pressure Altitude (m)", "GNSS Altitude (m)",
                "Position East (m)", "Position North (m)", "Position Up (m)",
            )),
            ColumnGroup("Velocity", listOf(
                "Altitude Rate (m/s)",
                "Velocity East (m/s)", "Velocity North (m/s)", "Velocity Up (m/s)",
                "GNSS East Velocity (m/s)", "GNSS North Velocity (m/s)", "GNSS Up Velocity (m/s)",
            )),
            ColumnGroup("Acceleration", listOf(
                "Low-G Acceleration X (m/s2)", "Low-G Acceleration Y (m/s2)", "Low-G Acceleration Z (m/s2)",
                "High-G Acceleration X (m/s2)", "High-G Acceleration Y (m/s2)", "High-G Acceleration Z (m/s2)",
            )),
            // #514: the quaternion is the only reconstructable attitude — Roll
            // is the body-Z azimuth while Pitch/Yaw are ZYX-Euler, so those
            // three are NOT a valid Euler triple.  Two name generations appear
            // in the wild: the current semicolon names (the #514 comma names
            // broke naive readers and were re-released with semicolons;
            // repairSplitHeaderNames folds the 7/14–7/16 comma exports into
            // this form) and the pre-#514 plain names.
            ColumnGroup("Rotation", listOf(
                "Gyro X (deg/s)", "Gyro Y (deg/s)", "Gyro Z (deg/s)",
                "Quat q0", "Quat q1", "Quat q2", "Quat q3",
                "Roll (deg; body-Z azimuth)", "Pitch (deg; ZYX Euler)", "Yaw (deg; ZYX Euler)",
                "Roll (deg)", "Pitch (deg)", "Yaw (deg)",           // pre-#514 exports
                "Roll Command (deg)",
            )),
            ColumnGroup("Environment", listOf(
                "Pressure (Pa)", "Barometer Temperature (C)",
                "Magnetic Field X (uT)", "Magnetic Field Y (uT)", "Magnetic Field Z (uT)",
            )),
            ColumnGroup("Power", listOf(
                "Voltage (V)", "Current (mA)", "State of Charge (%)",
            )),
            ColumnGroup("GPS", listOf(
                "Latitude (deg)", "Longitude (deg)",
                "Number of Satellites", "PDOP",
                "GNSS Horizontal Accuracy (m)", "GNSS Vertical Accuracy (m)",
            )),
            // #142/#143 renamed the per-detector apogee columns in the WRITER
            // and added a voted master, but this list kept the old names —
            // "Altitude Apogee Flag" / "Velocity Apogee Flag" — for a year.
            // Both pickers intersect these groups with the file's actual
            // headers, so the dead names were silently dropped rather than
            // shown as ghosts, and the 16 columns that do exist were
            // unreachable: an operator investigating a deployment-timing
            // anomaly could not plot when the master apogee vote latched or
            // when a pyro fired (#838 item 5).
            ColumnGroup("Flags", listOf(
                "Launch Flag", "Landed Flag", "Deployed Flag",
                "Reboot Recovery", "FC Guidance Enabled",
            )),
            // Its own group because apogee is not one thing: four independent
            // detectors vote and the master latches on that vote, and telling
            // a deployment story means plotting them against each other.
            ColumnGroup("Apogee", listOf(
                "Apogee Detector: Baro", "Apogee Detector: Velocity",
                "Apogee Detector: GPS", "Apogee Detector: Pitch",
                "Apogee Flag (Master)",
            )),
            ColumnGroup("Pyro", listOf(
                "Pyro 1 Continuity", "Pyro 2 Continuity",
                "Pyro 3 Continuity", "Pyro 4 Continuity",
                "Pyro 1 Fired", "Pyro 2 Fired", "Pyro 3 Fired", "Pyro 4 Fired",
            )),
            ColumnGroup("Diagnostics", listOf(
                "EKF Ticks",
            )),
        )

        /**
         * Column names kept in [columnGroups] that the CURRENT writer no
         * longer emits, because files written by older builds still carry
         * them and must stay plottable.
         *
         * The parity test that pins [columnGroups] against
         * `CsvGenerator.buildCsvHeader()` allows exactly these — anything else
         * absent from the header is a rename that left this list behind, which
         * is what #838 item 5 was.
         */
        internal val legacyColumnAliases: Set<String> = setOf(
            // Pre-#514 attitude names.  #514 renamed these to spell out that
            // Roll is a body-Z azimuth while Pitch/Yaw are ZYX-Euler, so the
            // three are not a valid Euler triple.
            "Roll (deg)", "Pitch (deg)", "Yaw (deg)",
        )

        /**
         * Header columns deliberately absent from [columnGroups] — not
         * plottable as a series.
         */
        internal val nonPlottableColumns: Set<String> = setOf(
            "Time (ms)",     // the X axis
        )

        /**
         * Column names grouped by category for LoRa base-station CSV files.
         * Mirrors iOS `FlightCSVData.loraColumnGroups`.
         */
        public val loraColumnGroups: List<ColumnGroup> = listOf(
            ColumnGroup("Altitude & Speed", listOf(
                "pressure_alt", "alt_m", "alt_rate", "speed",
                "max_alt", "max_speed",
            )),
            ColumnGroup("Acceleration", listOf(
                "acc_x", "acc_y", "acc_z",
            )),
            ColumnGroup("Rotation", listOf(
                "gyro_x", "gyro_y", "gyro_z",
                "roll", "pitch", "yaw",
            )),
            ColumnGroup("Power", listOf(
                "voltage", "current", "soc",
            )),
            ColumnGroup("GPS", listOf(
                "lat", "lon", "num_sats", "pdop", "h_acc",
            )),
            ColumnGroup("Radio", listOf(
                "rssi", "snr",
            )),
            ColumnGroup("State & Flags", listOf(
                "state", "launch", "vel_apo", "alt_apo", "landed",
            )),
        )

        /**
         * Pick the right column groups based on which headers are present.
         * LoRa CSVs use short names (e.g. "pressure_alt"), rocket CSVs use
         * descriptive names (e.g. "Pressure Altitude (m)").
         */
        public fun columnGroupsFor(headers: List<String>): List<ColumnGroup> {
            if (headers.contains("pressure_alt") || headers.contains("rssi")) {
                return loraColumnGroups
            }
            return columnGroups
        }
    }
}

/** Parse failures, mirroring iOS `CSVParserError`. */
public sealed class CsvParserException(message: String) : Exception(message) {
    public class EmptyFile : CsvParserException("CSV file is empty")
    public class NoHeader : CsvParserException("CSV file has no header row")
}

public object CsvParser {

    /**
     * Apps built 2026-07-14…07-16 (#514, pre-fix) emitted three column names
     * containing literal commas without quoting, so the header row carried
     * three more comma-separated tokens than every data row and all columns
     * after "Quat q3" loaded shifted.  Re-join those known token pairs into
     * the current comma-free names so existing exports parse correctly.
     * (The writer no longer puts commas in column names — see CSVGenerator.)
     *
     * Table-driven and exact: pre-#514 plain names ("Roll (deg)") and the
     * current semicolon names never match a (first, second) pair, so they
     * pass through untouched.
     */
    private val splitHeaderRepairs: Map<String, Pair<String, String>> = mapOf(
        // first token → (second token, joined replacement)
        "Roll (deg" to ("body-Z azimuth)" to "Roll (deg; body-Z azimuth)"),
        "Pitch (deg" to ("ZYX Euler)" to "Pitch (deg; ZYX Euler)"),
        "Yaw (deg" to ("ZYX Euler)" to "Yaw (deg; ZYX Euler)"),
    )

    public fun repairSplitHeaderNames(headers: List<String>): List<String> {
        val repaired = ArrayList<String>(headers.size)
        var i = 0
        while (i < headers.size) {
            val fix = splitHeaderRepairs[headers[i]]
            if (i + 1 < headers.size && fix != null && headers[i + 1] == fix.first) {
                repaired.add(fix.second)
                i += 2
            } else {
                repaired.add(headers[i])
                i += 1
            }
        }
        return repaired
    }

    /**
     * Parse CSV text into columnar [FlightCsvData].
     *
     * Deviation from iOS (by design): the Swift `parse(url:)` reads the file
     * itself; this pure-JVM port takes the decoded UTF-8 content and leaves
     * I/O to the caller.  Everything after the read is a line-for-line port:
     *  - lines split on "\n" dropping EMPTY lines only (a lone "\r" survives,
     *    exactly as in Swift `split(omittingEmptySubsequences: true)`);
     *  - the header row splits on "," dropping empty tokens (Swift default),
     *    then trims spaces/tabs (Swift `.whitespaces` excludes CR/LF, so a
     *    trailing "\r" on a CRLF header is preserved — mirrored here);
     *  - data rows split on "," KEEPING empty fields; unparseable or empty
     *    fields become NaN; short rows pad NaN; over-long rows clamp with
     *    min() — the #236 trap: the naive `fields.count..<columnCount` pad
     *    range inverts (and trapped on iOS) when a garbled row like
     *    "166.65.5,extra" carries more fields than the header;
     *  - [FlightCsvData.rowCount] counts data lines, not columns.
     */
    public fun parse(content: String): FlightCsvData =
        parse(content.splitToSequence("\n"), maxSampleHz = null)

    /**
     * Streaming overload — the one the app should use for real flight CSVs.
     *
     * #636: the `String` overload cannot handle a real log.  A 96-second sim
     * flight produces a 72 MB CSV, which `readText()` turns into a ~134 MB
     * UTF-16 `String`, and `split("\n")` then copies every line again.  That
     * allocation threw `OutOfMemoryError` against a 268 MB heap and killed the
     * app.  Feeding this from `File.useLines` keeps peak memory to the parsed
     * columns alone, independent of file size.
     *
     * Semantics are identical to the `String` overload — empty lines are
     * dropped ANYWHERE, not just at the end, matching the original
     * `split("\n").filter { it.isNotEmpty() }`.
     */
    public fun parse(
        lines: Sequence<String>,
        maxSampleHz: Double? = null,
    ): FlightCsvData {
        val it = lines.filter { line -> line.isNotEmpty() }.iterator()

        if (!it.hasNext()) {
            throw CsvParserException.EmptyFile()
        }

        val rawHeaders = it.next().split(",")
            .filter { h -> h.isNotEmpty() }      // Swift split omits empty subsequences
            .map { h -> h.trim(' ', '\t') }      // Swift .whitespaces: blanks, NOT CR/LF
        val headers = repairSplitHeaderNames(rawHeaders)
        val columnCount = headers.size

        if (columnCount == 0) {
            throw CsvParserException.NoHeader()
        }

        // Columnar primitive buffers.  ArrayList<Double> boxes every value at
        // ~24 B; a 72 MB CSV holds millions, so boxing alone can exhaust the
        // heap even once the text is streamed (#636).
        val columns = Array(columnCount) { DoubleBuf() }
        var rowCount = 0

        // #636 rate limiting.  The FC logs the IMU at 1920-3840 Hz, so a
        // 96-second flight is a 72 MB / ~500k-row CSV — and the chart shows at
        // most ~2000 LTTB points, so essentially all of that is parsed and then
        // discarded.  When a ceiling is given, keep at most one row per
        // interval and SKIP THE SPLIT for the rest: a dropped row costs one
        // indexOf plus a short substring instead of 63 substrings, which is
        // where the time actually goes.
        //
        // Safe because of how this CSV is shaped, both verified on real logs:
        //   - every column is forward-filled (~100% populated), so dropping
        //     rows never loses a slow signal like GNSS — those values repeat
        //   - every event column LATCHES rather than spiking (Launch, Apogee,
        //     Landed, Pyro Fired all hold their value once set), so a stride
        //     cannot step over an event.  It only quantises the transition
        //     time to the interval — 10 ms at 100 Hz
        // A sensor that pulsed for a single row would NOT survive this; if one
        // is ever added, exclude it here or keep its column at full rate.
        val minIntervalMs = maxSampleHz?.let { hz -> if (hz > 0) 1000.0 / hz else null }
        var lastKeptMs = Double.NEGATIVE_INFINITY

        // Parse all rows in a single pass
        while (it.hasNext()) {
            val line = it.next()

            if (minIntervalMs != null) {
                // Read ONLY the leading time field; skipping is the whole point,
                // so a skipped row must not pay for a full split.
                val comma = line.indexOf(',')
                val t = (if (comma >= 0) line.substring(0, comma) else line)
                    .trim().toDoubleOrNull()
                if (t != null) {
                    if (t - lastKeptMs < minIntervalMs) continue
                    lastKeptMs = t
                }
                // Unparseable time: keep the row rather than silently drop it.
            }

            rowCount++
            val fields = line.split(",")         // Kotlin split keeps empty fields
            val consumed = minOf(fields.size, columnCount)
            for (col in 0 until consumed) {
                columns[col].add(parseFieldStrict(fields[col]))
            }
            // Pad short rows.  min() guards over-long rows (more fields than
            // the header — e.g. garbled LoRa telemetry like "166.65.5"
            // splitting into extra fields): the bare Swift range
            // `fields.count..<columnCount` trapped when fields.count >
            // columnCount (#236).  Extra fields were already consumed above.
            for (col in consumed until columnCount) {
                columns[col].add(Double.NaN)
            }
        }

        // Build map keyed by column name (duplicate headers: last one wins,
        // matching the Swift dictionary assignment).
        val dict = LinkedHashMap<String, List<Double>>(columnCount)
        for ((i, header) in headers.withIndex()) {
            dict[header] = columns[i].toList()
        }

        return FlightCsvData(headers = headers, columns = dict, rowCount = rowCount)
    }

    /**
     * Growable primitive double buffer, exposed as an unboxed `List<Double>`.
     *
     * Keeps [FlightCsvData.columns] typed `Map<String, List<Double>>` — so the
     * public API, the golden tests and every consumer are unchanged — while
     * storing 8 B per value instead of a boxed ~24 B (#636).  Boxing then
     * happens only on individual `get()` calls, which the chart makes a couple
     * of thousand times after LTTB decimation rather than millions of times up
     * front.
     */
    private class DoubleBuf {
        private var a = DoubleArray(1024)
        private var n = 0

        fun add(v: Double) {
            if (n == a.size) a = a.copyOf(a.size * 2)
            a[n++] = v
        }

        /** Element-wise `equals`/`hashCode` come from AbstractList. */
        fun toList(): List<Double> {
            val exact = a.copyOf(n)
            return object : AbstractList<Double>() {
                override val size: Int get() = exact.size
                override fun get(index: Int): Double = exact[index]
            }
        }
    }

    /**
     * Strict field-to-Double conversion mirroring Swift `Double(Substring)`.
     *
     * Kotlin's `toDoubleOrNull` (Java `parseDouble`) is laxer than Swift: it
     * accepts surrounding whitespace, trailing 'd'/'D'/'f'/'F' suffixes, and
     * differs on "NaN"/"Infinity"/hex-float spellings.  Those forms only ever
     * appear in garbage rows, where iOS lands on NaN — so anything outside a
     * plain decimal/scientific literal maps to NaN here, matching the iOS
     * column outcome.  (Documented deviation: Swift itself parses a literal
     * "nan"/"inf" field to nan/inf; per the port decision those are folded to
     * NaN since no writer ever emits them.)
     */
    private fun parseFieldStrict(field: String): Double {
        if (field.isEmpty()) return Double.NaN
        for (c in field) {
            when (c) {
                in '0'..'9', '+', '-', '.', 'e', 'E' -> Unit
                else -> return Double.NaN            // whitespace, x/X, n/N, i/I, d/D, f/F, …
            }
        }
        return field.toDoubleOrNull() ?: Double.NaN
    }
}
