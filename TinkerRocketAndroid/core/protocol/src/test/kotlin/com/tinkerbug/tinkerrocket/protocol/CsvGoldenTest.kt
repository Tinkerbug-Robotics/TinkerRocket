package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonElement
import kotlinx.serialization.json.JsonNull
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import java.io.File
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.fail

/**
 * The Phase-1 exit gate: Kotlin bin→CSV over the emitter's synthetic flight
 * must reproduce the COMMITTED iOS output (tests_cpp/fixtures/csv_golden/,
 * produced and byte-pinned by the iOS CsvGoldenTest).  CSV comparison is
 * byte-for-byte; the summary JSON is compared as parsed structures (iOS
 * JSONEncoder whitespace/number formatting differs from kotlinx — semantic
 * parity is the contract, plan §correction).
 */
class CsvGoldenTest {

    private val goldenDir = File(WireFixtures.root.parentFile, "csv_golden")

    @Test
    fun `tiny flight CSV matches the committed iOS golden byte-for-byte`() {
        val bin = WireFixtures.bytes("csv/tiny_flight.bin")
        val (csv, _) = CsvGenerator().writeCsv(bin)
        val golden = File(goldenDir, "tiny_flight.expected.csv").readText()

        if (csv != golden) {
            val g = csv.split("\n")
            val e = golden.split("\n")
            for (i in 0 until maxOf(g.size, e.size)) {
                val gl = g.getOrNull(i) ?: "<missing>"
                val el = e.getOrNull(i) ?: "<missing>"
                if (gl != el) {
                    fail("CSV differs at line ${i + 1}:\n generated: $gl\n golden:    $el")
                }
            }
        }
        val lines = csv.split("\n").filter { it.isNotEmpty() }
        assertEquals(162, lines.size, "161 data rows + header")
        assertEquals(62, lines[0].split(",").size, "62 columns")
    }

    @Test
    fun `tiny flight summary JSON matches the committed iOS golden semantically`() {
        val bin = WireFixtures.bytes("csv/tiny_flight.bin")
        val (_, summary) = CsvGenerator().writeCsv(bin)
        val generated = Json.parseToJsonElement(summary.toJson())
        val golden = Json.parseToJsonElement(
            File(goldenDir, "tiny_flight.expected.summary.json").readText())
        assertSemanticallyEqual(golden, generated, "$")
    }

    /** Structural compare with NUMERIC primitive equality — Swift encodes
     * Double 6.0 as "6" where kotlinx writes "6.0"; both mean the same. */
    private fun assertSemanticallyEqual(expected: JsonElement, actual: JsonElement, path: String) {
        when {
            expected is JsonObject && actual is JsonObject -> {
                assertEquals(expected.keys, actual.keys, "keys differ at $path")
                for (k in expected.keys) {
                    assertSemanticallyEqual(expected[k]!!, actual[k]!!, "$path.$k")
                }
            }
            expected is JsonArray && actual is JsonArray -> {
                assertEquals(expected.size, actual.size, "array size at $path")
                expected.indices.forEach {
                    assertSemanticallyEqual(expected[it], actual[it], "$path[$it]")
                }
            }
            expected is JsonPrimitive && actual is JsonPrimitive -> {
                if (expected is JsonNull || actual is JsonNull || expected.isString || actual.isString) {
                    assertEquals(expected, actual, "value at $path")
                } else {
                    val e = expected.content.toDoubleOrNull()
                    val a = actual.content.toDoubleOrNull()
                    if (e != null && a != null) {
                        assertEquals(e, a, "numeric value at $path")
                    } else {
                        assertEquals(expected.content, actual.content, "value at $path")
                    }
                }
            }
            else -> fail("shape mismatch at $path: $expected vs $actual")
        }
    }
}
