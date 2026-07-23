package com.tinkerbug.tinkerrocket.protocol

import java.time.Instant
import java.time.LocalDateTime
import java.time.ZoneOffset
import java.time.format.DateTimeFormatter
import java.time.format.ResolverStyle
import java.util.Locale

// Pure-logic port of the naming/classification rules in
// TinkerRocketApp/Models/FileCache.swift (CachedFlight + FileCache).  Only
// the string/date/size logic lives here — all FileManager I/O stays in the
// Android storage layer, which calls these functions over plain names and
// sizes it read from disk.

/** Cache entry classification. Mirrors iOS `CachedFlightType`. */
public enum class CachedFlightKind {
    /** On-board rocket flight ("flight_*.csv" with a paired .bin/.json). */
    ROCKET,

    /** Base-station LoRa log ("lora_*.csv", CSV-only). */
    LORA_LOG,
}

public object FlightFiles {

    // iOS uses DateFormatter "yyyyMMddHHmmss" in UTC with the default
    // lenient=false, which rejects impossible dates (month 13, Feb 31).
    // java.time's SMART resolver would silently adjust some of those, so the
    // port pins STRICT + "uuuu" to match the iOS reject behavior.
    private val utcTimestampFormat: DateTimeFormatter =
        DateTimeFormatter.ofPattern("uuuuMMddHHmmss", Locale.ROOT)
            .withResolverStyle(ResolverStyle.STRICT)

    // MARK: bin ↔ csv ↔ json sidecar name pairing

    /**
     * Convert a binary filename to its CSV counterpart.
     * e.g. "flight_20260224_021306.bin" → "flight_20260224_021306.csv"
     * Mirrors iOS `FileCache.csvName(for:)`.
     */
    public fun csvNameFor(binaryFilename: String): String =
        if (binaryFilename.endsWith(".bin")) {
            binaryFilename.dropLast(4) + ".csv"
        } else {
            "$binaryFilename.csv"
        }

    /**
     * Legacy cache name: early app builds appended ".csv" to the FULL binary
     * name, producing the double extension "flight_x.bin.csv".  Readers must
     * still accept it (iOS `getCachedCSV` falls back to this name).
     */
    public fun legacyCsvNameFor(binaryFilename: String): String = "$binaryFilename.csv"

    /**
     * CSV cache-lookup candidates for a binary, in iOS probe order:
     * current name first ("x.csv"), then the legacy double-extension form
     * ("x.bin.csv").  For a non-".bin" input both candidates coincide.
     */
    public fun candidateCsvNames(binaryFilename: String): List<String> =
        listOf(csvNameFor(binaryFilename), legacyCsvNameFor(binaryFilename)).distinct()

    /**
     * Convert a binary filename to its flight-summary JSON counterpart.
     * e.g. "flight_20260224_021306.bin" → "flight_20260224_021306.json"
     * Mirrors iOS `FileCache.summaryName(for:)`.
     */
    public fun summaryNameFor(binaryFilename: String): String =
        if (binaryFilename.endsWith(".bin")) {
            binaryFilename.dropLast(4) + ".json"
        } else {
            "$binaryFilename.json"
        }

    /**
     * Paired binary name for a cached CSV filename (rocket flights), handling
     * both generations: "flight_x.bin.csv" → "flight_x.bin" (legacy) and
     * "flight_x.csv" → "flight_x.bin".  Mirrors the derivation inside iOS
     * `FileCache.listCachedFlights()`.  Null for non-CSV names.
     */
    public fun binaryNameForCsv(csvFilename: String): String? = when {
        csvFilename.endsWith(".bin.csv") -> csvFilename.dropLast(4)
        csvFilename.endsWith(".csv") -> csvFilename.dropLast(4) + ".bin"
        else -> null
    }

    /**
     * Extension-free base name for a cached CSV filename:
     * "flight_x.bin.csv" → "flight_x", "flight_x.csv" → "flight_x".
     * Null for non-CSV names.
     */
    public fun baseNameForCsv(csvFilename: String): String? = when {
        csvFilename.endsWith(".bin.csv") -> csvFilename.dropLast(8)
        csvFilename.endsWith(".csv") -> csvFilename.dropLast(4)
        else -> null
    }

    /**
     * Summary-JSON sidecar name for a cached CSV filename ("flight_x.csv" or
     * legacy "flight_x.bin.csv" → "flight_x.json").  Null for non-CSV names.
     */
    public fun summaryNameForCsv(csvFilename: String): String? =
        baseNameForCsv(csvFilename)?.plus(".json")

    // MARK: classification

    /**
     * Classify a cached CSV filename for the Flight Logs page, mirroring the
     * filename-pattern branch of iOS `FileCache.listCachedFlights()`:
     *  - ".json" files are sidecars, never standalone entries → null;
     *  - "flight_" prefix with ".csv" (or legacy ".bin.csv") → [CachedFlightKind.ROCKET];
     *  - "lora_" prefix with ".csv" → [CachedFlightKind.LORA_LOG];
     *  - anything else is an unknown pattern → null (skipped).
     */
    public fun classify(filename: String): CachedFlightKind? {
        if (filename.endsWith(".json")) return null
        // The redundant-looking .bin.csv and !lora_ terms mirror iOS exactly.
        if (filename.startsWith("flight_") &&
            (filename.endsWith(".csv") || filename.endsWith(".bin.csv")) &&
            !filename.startsWith("lora_")
        ) {
            return CachedFlightKind.ROCKET
        }
        if (filename.startsWith("lora_") && filename.endsWith(".csv")) {
            return CachedFlightKind.LORA_LOG
        }
        return null
    }

    // MARK: flight_YYYYMMDD_HHMMSS timestamp naming

    /**
     * Parse the UTC timestamp from a flight filename.
     * Supports "flight_YYYYMMDD_HHMMSS.csv" (or legacy ".bin.csv"),
     * "lora_YYYYMMDD_HHMMSS.csv", and extension-free base names — a
     * line-for-line port of iOS `CachedFlight.flightDate`, including its
     * quirks: the separator character between date and time is skipped
     * WITHOUT being checked (any single char passes), and trailing extra
     * characters after the 6 time digits are ignored.
     *
     * Returns null for unknown prefixes, short names, non-digit fields, or
     * impossible dates (strict resolution, matching iOS's non-lenient
     * DateFormatter).
     */
    public fun flightTimestampUtc(name: String): Instant? {
        var base = name
        if (base.endsWith(".bin.csv")) {
            base = base.dropLast(8) // legacy format
        } else if (base.endsWith(".csv")) {
            base = base.dropLast(4)
        }

        // Determine prefix length: "flight_" (7) or "lora_" (5)
        val prefixLen = when {
            base.startsWith("flight_") -> 7
            base.startsWith("lora_") -> 5
            else -> return null
        }

        // Need at least prefix + 8 (date) + 1 (separator) + 6 (time) = prefix + 15
        if (base.length < prefixLen + 15) return null

        val dateStr = base.substring(prefixLen, prefixLen + 8)
        val timeStr = base.substring(prefixLen + 9, prefixLen + 15) // skip separator, unchecked

        if (!dateStr.all { it.isDigit() } || !timeStr.all { it.isDigit() }) return null

        return try {
            LocalDateTime.parse(dateStr + timeStr, utcTimestampFormat)
                .toInstant(ZoneOffset.UTC)
        } catch (_: Exception) {
            null
        }
    }

    /**
     * Format the canonical extension-free base name for a flight timestamp:
     * "flight_YYYYMMDD_HHMMSS" (or "lora_…"), UTC.
     *
     * NOTE: convenience for the Android side and round-trip tests — iOS has
     * no formatter (firmware names the files); keep firmware authoritative.
     */
    public fun formatFlightBaseName(timestamp: Instant, lora: Boolean = false): String {
        val t = LocalDateTime.ofInstant(timestamp, ZoneOffset.UTC)
        val prefix = if (lora) "lora_" else "flight_"
        return prefix +
            DateTimeFormatter.ofPattern("uuuuMMdd", Locale.ROOT).format(t) +
            "_" +
            DateTimeFormatter.ofPattern("HHmmss", Locale.ROOT).format(t)
    }

    /**
     * Fallback display title when a filename carries no parseable timestamp:
     * strip a trailing ".csv" for cleaner display, otherwise the raw name.
     * Mirrors the fallback arm of iOS `CachedFlight.displayTitle` (the
     * timestamp arm is a localized date string, formatted by the UI layer).
     */
    public fun displayNameFallback(name: String): String =
        if (name.endsWith(".csv")) name.dropLast(4) else name

    // MARK: size-equality cache-hit rule

    /**
     * Whether a device-listed flight is already fully cached.  Legacy
     * firmware reuses sequential names (rocket_data_000.bin), so a filename
     * match alone is NOT enough — the cached binary's byte size must equal
     * the device-reported size to distinguish a new flight from a previously
     * downloaded one.  Mirrors iOS `FileCache.isFlightCached(_:expectedSize:)`:
     * both the binary and its CSV must exist, and an unreadable cached size
     * counts as 0 (never a hit for a non-empty file).
     *
     * @param binaryCached whether the cached binary file exists
     * @param csvCached whether the converted CSV exists in the cache
     * @param cachedBinarySizeBytes size of the cached binary, null if unreadable
     * @param expectedSizeBytes device-reported size (wire UInt32, widened)
     */
    public fun isFlightCacheHit(
        binaryCached: Boolean,
        csvCached: Boolean,
        cachedBinarySizeBytes: Long?,
        expectedSizeBytes: Long,
    ): Boolean {
        if (!binaryCached || !csvCached) return false
        return (cachedBinarySizeBytes ?: 0L) == expectedSizeBytes
    }
}
