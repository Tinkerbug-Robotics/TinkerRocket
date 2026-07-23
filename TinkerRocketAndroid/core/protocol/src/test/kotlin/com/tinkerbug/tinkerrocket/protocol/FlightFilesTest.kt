package com.tinkerbug.tinkerrocket.protocol

import java.time.Instant
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * Pins for the pure FileCache logic port (iOS FileCache.swift has no test
 * twin — expectations here are hand-derived from the Swift source):
 * flight_YYYYMMDD_HHMMSS naming, legacy ".bin.csv" double extension, lora_
 * classification, sidecar name pairing, and the size-equality cache-hit rule.
 */
class FlightFilesTest {

    private val ts = Instant.parse("2026-02-24T02:13:06Z")

    // MARK: timestamp parsing (CachedFlight.flightDate)

    @Test
    fun `parses UTC timestamp from all supported name forms`() {
        assertEquals(ts, FlightFiles.flightTimestampUtc("flight_20260224_021306.csv"))
        // Legacy double extension strips ".bin.csv" (8 chars), not ".csv".
        assertEquals(ts, FlightFiles.flightTimestampUtc("flight_20260224_021306.bin.csv"))
        assertEquals(ts, FlightFiles.flightTimestampUtc("lora_20260224_021306.csv"))
        // Extension-free base names parse too (iOS leaves base unchanged).
        assertEquals(ts, FlightFiles.flightTimestampUtc("flight_20260224_021306"))
    }

    @Test
    fun `rejects unknown prefixes short names and non-digits`() {
        assertNull(FlightFiles.flightTimestampUtc("rocket_data_000.bin"))
        assertNull(FlightFiles.flightTimestampUtc("rocket_data_000.csv"))
        assertNull(FlightFiles.flightTimestampUtc("flight_2026_0213.csv"))       // too short
        assertNull(FlightFiles.flightTimestampUtc("flight_2026ab24_021306.csv")) // letters in date
        assertNull(FlightFiles.flightTimestampUtc("flight_20260224_0213ab.csv")) // letters in time
        assertNull(FlightFiles.flightTimestampUtc(""))
    }

    /** Non-lenient date resolution: impossible dates return null (iOS DateFormatter). */
    @Test
    fun `rejects impossible dates strictly`() {
        assertNull(FlightFiles.flightTimestampUtc("flight_20261332_021306.csv")) // month 13
        assertNull(FlightFiles.flightTimestampUtc("flight_20260231_021306.csv")) // Feb 31
        assertNull(FlightFiles.flightTimestampUtc("flight_20260224_251306.csv")) // hour 25
    }

    /**
     * iOS quirks, pinned deliberately: the single separator char between the
     * date and time digits is skipped WITHOUT being validated, and characters
     * after the 6 time digits are ignored.
     */
    @Test
    fun `separator is unchecked and trailing chars ignored`() {
        assertEquals(ts, FlightFiles.flightTimestampUtc("flight_20260224X021306.csv"))
        assertEquals(ts, FlightFiles.flightTimestampUtc("flight_20260224_02130699.csv"))
    }

    @Test
    fun `format and parse round trip`() {
        assertEquals("flight_20260224_021306", FlightFiles.formatFlightBaseName(ts))
        assertEquals("lora_20260224_021306", FlightFiles.formatFlightBaseName(ts, lora = true))
        assertEquals(ts, FlightFiles.flightTimestampUtc(FlightFiles.formatFlightBaseName(ts)))
        assertEquals(
            ts,
            FlightFiles.flightTimestampUtc(FlightFiles.formatFlightBaseName(ts, lora = true) + ".csv"),
        )
    }

    // MARK: classification (FileCache.listCachedFlights filename branch)

    @Test
    fun `classifies rocket lora and unknown names`() {
        assertEquals(CachedFlightKind.ROCKET, FlightFiles.classify("flight_20260224_021306.csv"))
        assertEquals(CachedFlightKind.ROCKET, FlightFiles.classify("flight_20260224_021306.bin.csv"))
        assertEquals(CachedFlightKind.LORA_LOG, FlightFiles.classify("lora_20260224_021306.csv"))
        // Summary JSONs are sidecars, never standalone entries.
        assertNull(FlightFiles.classify("flight_20260224_021306.json"))
        // Unknown patterns are skipped.
        assertNull(FlightFiles.classify("rocket_data_000.csv"))
        assertNull(FlightFiles.classify("flight_20260224_021306.bin"))
        assertNull(FlightFiles.classify("lora_20260224_021306.txt"))
        assertNull(FlightFiles.classify("notes.txt"))
    }

    // MARK: sidecar name pairing (bin ↔ csv ↔ json)

    @Test
    fun `binary to csv and summary names`() {
        assertEquals("flight_20260224_021306.csv", FlightFiles.csvNameFor("flight_20260224_021306.bin"))
        assertEquals("flight_20260224_021306.json", FlightFiles.summaryNameFor("flight_20260224_021306.bin"))
        // Non-".bin" names get the extension appended (iOS fallback arm).
        assertEquals("rocket_data_000.csv", FlightFiles.csvNameFor("rocket_data_000"))
        assertEquals("rocket_data_000.json", FlightFiles.summaryNameFor("rocket_data_000"))
    }

    @Test
    fun `csv lookup candidates include legacy double extension`() {
        assertEquals(
            listOf("flight_x.csv", "flight_x.bin.csv"),
            FlightFiles.candidateCsvNames("flight_x.bin"),
        )
        // Without ".bin" the two candidates coincide.
        assertEquals(listOf("flight_x.csv"), FlightFiles.candidateCsvNames("flight_x"))
    }

    @Test
    fun `csv back to binary and base names for both generations`() {
        // Current generation
        assertEquals("flight_x.bin", FlightFiles.binaryNameForCsv("flight_x.csv"))
        assertEquals("flight_x", FlightFiles.baseNameForCsv("flight_x.csv"))
        assertEquals("flight_x.json", FlightFiles.summaryNameForCsv("flight_x.csv"))
        // Legacy generation: "flight_x.bin.csv" → drop only ".csv" for the bin.
        assertEquals("flight_x.bin", FlightFiles.binaryNameForCsv("flight_x.bin.csv"))
        assertEquals("flight_x", FlightFiles.baseNameForCsv("flight_x.bin.csv"))
        assertEquals("flight_x.json", FlightFiles.summaryNameForCsv("flight_x.bin.csv"))
        // Non-CSV names have no pairing.
        assertNull(FlightFiles.binaryNameForCsv("flight_x.bin"))
        assertNull(FlightFiles.baseNameForCsv("flight_x.json"))
        assertNull(FlightFiles.summaryNameForCsv("flight_x"))
    }

    @Test
    fun `display fallback strips only a csv extension`() {
        assertEquals("flight_x", FlightFiles.displayNameFallback("flight_x.csv"))
        assertEquals("flight_x.bin", FlightFiles.displayNameFallback("flight_x.bin.csv"))
        assertEquals("notes.txt", FlightFiles.displayNameFallback("notes.txt"))
    }

    // MARK: size-equality cache-hit rule (FileCache.isFlightCached(_:expectedSize:))

    /**
     * Legacy firmware reuses sequential names, so a name match alone must not
     * count as cached — the binary size has to match the device-reported size.
     */
    @Test
    fun `cache hit requires both files and exact size match`() {
        assertTrue(FlightFiles.isFlightCacheHit(true, true, 1234L, 1234L))
        // Same name, different size → NOT a hit (new flight under a reused name).
        assertFalse(FlightFiles.isFlightCacheHit(true, true, 1234L, 999L))
        // Unreadable size counts as 0 (never matches a non-empty file)…
        assertFalse(FlightFiles.isFlightCacheHit(true, true, null, 1234L))
        // …which does match a zero-length expectation, exactly like iOS.
        assertTrue(FlightFiles.isFlightCacheHit(true, true, null, 0L))
        // Either file missing → miss regardless of size.
        assertFalse(FlightFiles.isFlightCacheHit(false, true, 1234L, 1234L))
        assertFalse(FlightFiles.isFlightCacheHit(true, false, 1234L, 1234L))
    }
}
