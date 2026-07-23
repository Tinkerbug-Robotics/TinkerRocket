package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long
import java.io.File
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertIs
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * FileOpsDispatch: every fileops/ golden fixture routed through the
 * first-byte demux (asserting the variant + spot fields against the
 * sidecar), plus the JSON branches and the per-branch failure behaviors the
 * iOS switch pins.
 */
class FileOpsDispatchGoldenTest {

    // ── golden walk over the whole corpus ─────────────────────────────────

    @Test
    fun `every fileops fixture dispatches to the right variant`() {
        val dir = File(WireFixtures.root, "fileops")
        val fixtures = dir.listFiles { f -> f.name.endsWith(".bin") }!!.sortedBy { it.name }
        assertTrue(fixtures.size >= 8, "corpus shrank? found ${fixtures.size} fixtures")

        for (f in fixtures) {
            val rel = "fileops/${f.name}"
            val msg = FileOpsDispatch.parse(WireFixtures.bytes(rel))
            val side = WireFixtures.sidecar(rel)
            when {
                f.name.startsWith("scan_") -> {
                    val m = assertIs<FileOpsMessage.Scan>(msg, rel)
                    assertEquals(side["n"]!!.jsonPrimitive.int, m.result.samples.size, rel)
                    assertEquals(
                        side["start_mhz"]!!.jsonPrimitive.int.toFloat(), m.result.startMhz, rel,
                    )
                }
                f.name.startsWith("magcal_") -> {
                    val m = assertIs<FileOpsMessage.MagCal>(msg, rel)
                    assertEquals(side["sub_type"]!!.jsonPrimitive.int, m.status.subType.raw, rel)
                    assertEquals(
                        side["sample_count"]!!.jsonPrimitive.int, m.status.sampleCount, rel,
                    )
                    assertEquals(side["offset_x"]!!.jsonPrimitive.int, m.status.offsetX, rel)
                    // Length-generation trailers: present in the sidecar only
                    // when present on the wire, else decode defaults to 0.
                    val mask = side["coverage_mask"]?.jsonPrimitive?.long ?: 0L
                    assertEquals(mask, m.status.coverageMask, rel)
                    val partial = side["partial_mask"]?.jsonPrimitive?.long ?: 0L
                    assertEquals(partial, m.status.partialMask, rel)
                }
                f.name.startsWith("sensorcal_") -> {
                    val m = assertIs<FileOpsMessage.SensorCal>(msg, rel)
                    assertEquals(side["valid"]!!.jsonPrimitive.int != 0, m.status.valid, rel)
                    assertEquals(side["gyro_x"]!!.jsonPrimitive.int, m.status.gyroX, rel)
                }
                f.name.startsWith("storage_rocket_") -> {
                    val m = assertIs<FileOpsMessage.RocketStorage>(msg, rel)
                    assertEquals(
                        side["flight_region_blocks"]!!.jsonPrimitive.int,
                        m.stats.flightRegionBlocks, rel,
                    )
                    assertEquals(
                        side["flight_count"]!!.jsonPrimitive.int, m.stats.flightCount, rel,
                    )
                }
                f.name.startsWith("storage_bs_") -> {
                    val m = assertIs<FileOpsMessage.BsStorage>(msg, rel)
                    assertEquals(side["total_bytes"]!!.jsonPrimitive.long, m.stats.totalBytes, rel)
                    assertEquals(side["backend"]!!.jsonPrimitive.int, m.stats.backend, rel)
                }
                else -> throw AssertionError("unmapped fixture $rel — extend this walk")
            }
        }
    }

    // ── JSON branches ─────────────────────────────────────────────────────

    @Test
    fun `open-brace routes to ota_status when the type matches`() {
        val msg = FileOpsDispatch.parse(
            """{"type":"ota_status","state":"writing","bytes":40960}""".encodeToByteArray(),
        )
        val m = assertIs<FileOpsMessage.Ota>(msg)
        assertEquals(OtaStatusUpdate.State.WRITING, m.status.state)
        assertEquals(40960L, m.status.bytes)
    }

    @Test
    fun `open-brace non-ota json falls through to file-list and drops`() {
        // iOS: OTAStatusUpdate.parse fails → parseFileList → array decode of
        // an object fails → logged and dropped.
        assertNull(FileOpsDispatch.parse("""{"type":"config"}""".encodeToByteArray()))
        assertNull(FileOpsDispatch.parse("""{"name":"x.bin","size":1}""".encodeToByteArray()))
    }

    @Test
    fun `array routes to file list`() {
        val msg = FileOpsDispatch.parse(
            """[{"name":"flight_20260722_101010.bin","size":1024}]""".encodeToByteArray(),
        )
        val m = assertIs<FileOpsMessage.FileList>(msg)
        assertEquals(1, m.page.files.size)
        assertEquals("flight_20260722_101010.bin", m.page.files[0].name)
        assertEquals(1024L, m.page.files[0].size)
        assertEquals(false, m.page.hasMore)
    }

    @Test
    fun `empty and garbage frames drop silently`() {
        assertNull(FileOpsDispatch.parse(ByteArray(0)))
        assertNull(FileOpsDispatch.parse("not json".encodeToByteArray()))
    }

    // ── per-branch failure behavior ───────────────────────────────────────

    @Test
    fun `malformed scan frame is ScanFailed - session must clear the spinner`() {
        val good = WireFixtures.bytes("fileops/scan_0xAA_5samples.bin")
        // Truncated below the 10-byte header.
        assertEquals(FileOpsMessage.ScanFailed, FileOpsDispatch.parse(good.copyOf(9)))
        // Header claims more samples than the frame carries.
        assertEquals(FileOpsMessage.ScanFailed, FileOpsDispatch.parse(good.copyOf(good.size - 1)))
    }

    @Test
    fun `binary status frames below the outer length gate drop silently`() {
        // Outer gates (frame length INCLUDING discriminator): 23 / 20 / 15 / 27.
        assertNull(FileOpsDispatch.parse(ByteArray(22) { if (it == 0) 0xCA.toByte() else 0 }))
        assertNull(FileOpsDispatch.parse(ByteArray(19) { if (it == 0) 0xCB.toByte() else 0 }))
        assertNull(FileOpsDispatch.parse(ByteArray(14) { if (it == 0) 0xCC.toByte() else 0 }))
        assertNull(FileOpsDispatch.parse(ByteArray(26) { if (it == 0) 0xCD.toByte() else 0 }))
        // 0xCC at exactly the (weaker) outer gate of 15 still drops inside
        // decode (payload 14 < 15) — same net behavior as iOS.
        assertNull(FileOpsDispatch.parse(ByteArray(15) { if (it == 0) 0xCC.toByte() else 0 }))
        // At the decoder's real minimum they parse.
        assertIs<FileOpsMessage.RocketStorage>(
            FileOpsDispatch.parse(ByteArray(16) { if (it == 0) 0xCC.toByte() else 0 }),
        )
        assertIs<FileOpsMessage.BsStorage>(
            FileOpsDispatch.parse(ByteArray(27) { if (it == 0) 0xCD.toByte() else 0 }),
        )
    }

    @Test
    fun `magcal golden frame also dispatches through the demux`() {
        // The MagCalStatusTest suite covers decode() in depth; this pins the
        // demux path (discriminator stripped, ≥23 gate) end-to-end.
        val msg = FileOpsDispatch.parse(WireFixtures.bytes("fileops/magcal_0xCA_len36.bin"))
        val m = assertIs<FileOpsMessage.MagCal>(msg)
        assertEquals(MagCalSubType.APPLIED, m.status.subType)
    }
}
