package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import java.io.File
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * The corpus walk: every fixture the manifest lists exists with the exact
 * byte size the emitter recorded.  This is the Phase-0 exit gate — Kotlin
 * consuming the same committed bytes C++ generated and Swift will walk.
 */
class GoldenManifestTest {

    @Test
    fun `manifest lists a non-trivial corpus`() {
        val fixtures = WireFixtures.manifest()["fixtures"]!!.jsonArray
        assertTrue(fixtures.size >= 40, "expected >= 40 fixtures, got ${fixtures.size}")
    }

    @Test
    fun `every manifest entry exists with the recorded size`() {
        for (entry in WireFixtures.manifest()["fixtures"]!!.jsonArray) {
            val o = entry.jsonObject
            val rel = o["file"]!!.jsonPrimitive.content
            val f = File(WireFixtures.root, rel)
            assertTrue(f.isFile, "missing fixture: $rel")
            assertEquals(o["size"]!!.jsonPrimitive.int, f.length().toInt(), "size drift: $rel")
        }
    }

    @Test
    fun `command fixtures lead with their command byte`() {
        for (entry in WireFixtures.manifest()["fixtures"]!!.jsonArray) {
            val rel = entry.jsonObject["file"]!!.jsonPrimitive.content
            if (!rel.startsWith("commands/")) continue
            val cmd = WireFixtures.sidecar(rel)["cmd"]!!.jsonPrimitive.int
            val first = WireFixtures.bytes(rel)[0].toInt() and 0xFF
            assertEquals(cmd, first, "$rel first byte != sidecar cmd")
        }
    }
}
