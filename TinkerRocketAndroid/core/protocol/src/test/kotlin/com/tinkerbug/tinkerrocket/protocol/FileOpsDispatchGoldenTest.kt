package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.float
import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long
import java.io.File
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
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
                f.name.startsWith("padcal_") -> {
                    val m = assertIs<FileOpsMessage.PadCal>(msg, rel)
                    assertEquals(side["cal_state"]!!.jsonPrimitive.int, m.status.calState.raw, rel)
                    assertEquals(side["flags"]!!.jsonPrimitive.int, m.status.flags, rel)
                    assertEquals(side["pad_lat_e7"]!!.jsonPrimitive.int, m.status.padLatE7, rel)
                    assertEquals(side["pos_sigma_cm"]!!.jsonPrimitive.int, m.status.posSigmaCm, rel)
                }
                f.name.startsWith("accelcal_") -> {
                    // Every field: the six-face solve reads the means and the
                    // save's read-back compares the offsets to 1e-5 m/s², so a
                    // byte out of place anywhere in the 40 matters.
                    val s = assertIs<FileOpsMessage.AccelCal>(msg, rel).status
                    fun f(key: String): Float = side[key]!!.jsonPrimitive.float
                    val flags = side["flags"]!!.jsonPrimitive.int
                    assertEquals(flags, s.flags, rel)
                    assertEquals(flags and 0x01 != 0, s.stored, rel)
                    assertEquals(flags and 0x02 != 0, s.applied, rel)
                    assertEquals(side["face_state"]!!.jsonPrimitive.int, s.faceState.raw, rel)
                    assertEquals(listOf(f("offset_x"), f("offset_y"), f("offset_z")), s.offset, rel)
                    assertEquals(
                        listOf(f("face_mean_x"), f("face_mean_y"), f("face_mean_z")), s.faceMean, rel,
                    )
                    assertEquals(listOf(f("face_sd_x"), f("face_sd_y"), f("face_sd_z")), s.faceSd, rel)
                    assertEquals(side["face_n"]!!.jsonPrimitive.int, s.faceN, rel)
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
    fun `pad cal status needs the whole 41-byte frame`() {
        // [0xD0][PadCalStatusData 40 B].  One byte short is dropped like the
        // other malformed binaries — and never reaches the file-list parser,
        // where an unknown first byte used to go.
        assertNull(FileOpsDispatch.parse(ByteArray(40) { if (it == 0) 0xD0.toByte() else 0 }))
        val m = assertIs<FileOpsMessage.PadCal>(
            FileOpsDispatch.parse(ByteArray(41) { if (it == 0) 0xD0.toByte() else 0 }),
        )
        assertEquals(PadCalState.IDLE, m.status.calState)
    }

    @Test
    fun `accel cal status needs the whole 41-byte frame`() {
        // [0xD1][AccelCalStatusData 40 B] (#1629).  One byte short is dropped,
        // and never reaches the file-list parser.
        assertNull(FileOpsDispatch.parse(ByteArray(40) { if (it == 0) 0xD1.toByte() else 0 }))
        val m = assertIs<FileOpsMessage.AccelCal>(
            FileOpsDispatch.parse(ByteArray(41) { if (it == 0) 0xD1.toByte() else 0 }),
        )
        assertEquals(AccelFaceState.IDLE, m.status.faceState)
        assertFalse(m.status.stored)
        assertFalse(m.status.applied)
        // A face_state this app does not know reads as IDLE, which the face
        // sequencing ignores, rather than as a result.
        val odd = ByteArray(41).also { it[0] = 0xD1.toByte(); it[2] = 9 }
        assertEquals(AccelFaceState.IDLE, assertIs<FileOpsMessage.AccelCal>(FileOpsDispatch.parse(odd)).status.faceState)
    }

    @Test
    fun `pyro refusal frame parses and truncated ones drop`() {
        // [0xCE][refused cmd][channel][reason] — OC sendPyroTestRefusal.
        val msg = FileOpsDispatch.parse(byteArrayOf(0xCE.toByte(), 36, 2, 1))
        val m = assertIs<FileOpsMessage.PyroRefusal>(msg)
        assertEquals(36, m.cmd)
        assertEquals(2, m.channel)
        assertEquals(1, m.reason)
        // Below the 4-byte gate → dropped like the other malformed binaries.
        assertNull(FileOpsDispatch.parse(byteArrayOf(0xCE.toByte(), 35, 1)))
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
