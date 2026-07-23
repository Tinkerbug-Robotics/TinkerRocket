package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.float
import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull

/**
 * file_ops binary frames: 0xAA freq scan, 0xCB sensor cal, 0xCC/0xCD storage
 * stats — golden walks plus the malformed-frame rejections the iOS parse
 * sites bounds-check.
 */
class FileOpsGoldenTest {

    // --- 0xAA frequency scan ---

    @Test
    fun `scan frame decodes header and per-sample rssi`() {
        val rel = "fileops/scan_0xAA_5samples.bin"
        val side = WireFixtures.sidecar(rel)
        val r = FreqScanResult.decode(WireFixtures.bytes(rel))
        assertNotNull(r)
        assertEquals(side["start_mhz"]!!.jsonPrimitive.float, r.startMhz)
        assertEquals(side["step_khz"]!!.jsonPrimitive.float, r.stepKhz)
        assertEquals(side["n"]!!.jsonPrimitive.int, r.samples.size)
        assertEquals(side["rssi_dbm"]!!.jsonArray.map { it.jsonPrimitive.int },
            r.samples.map { it.rssiDbm })
        // Per-sample frequency mirrors the iOS float math:
        // freq_i = start + (step_khz * i) / 1000.
        r.samples.forEachIndexed { i, sample ->
            assertEquals(r.startMhz + (r.stepKhz * i.toFloat()) / 1000.0f, sample.freqMhz)
        }
    }

    @Test
    fun `scan frame malformed variants return null`() {
        val good = WireFixtures.bytes("fileops/scan_0xAA_5samples.bin")
        // Shorter than the 10-byte header.
        assertNull(FreqScanResult.decode(good.copyOf(9)))
        // Wrong discriminator.
        val wrongDisc = good.copyOf()
        wrongDisc[0] = 0xAB.toByte()
        assertNull(FreqScanResult.decode(wrongDisc))
        // Truncated sample list: header claims n samples but bytes are short.
        assertNull(FreqScanResult.decode(good.copyOf(good.size - 1)))
        // Header-only with n = 0 is valid (empty scan).
        val emptyScan = good.copyOf(10)
        emptyScan[9] = 0
        assertEquals(0, FreqScanResult.decode(emptyScan)!!.samples.size)
    }

    // --- 0xCB sensor cal ---

    @Test
    fun `sensor cal decodes field-for-field`() {
        val rel = "fileops/sensorcal_0xCB_19.bin"
        val side = WireFixtures.sidecar(rel)
        val frame = WireFixtures.bytes(rel)
        assertEquals(0xCB.toByte(), frame[0])
        val s = SensorCalStatus.decode(frame.copyOfRange(1, frame.size))
        assertNotNull(s)
        assertEquals(side["valid"]!!.jsonPrimitive.int != 0, s.valid)
        assertEquals(side["gyro_x"]!!.jsonPrimitive.int, s.gyroX)
        assertEquals(side["gyro_y"]!!.jsonPrimitive.int, s.gyroY)
        assertEquals(side["gyro_z"]!!.jsonPrimitive.int, s.gyroZ)
        assertEquals(side["hg_x"]!!.jsonPrimitive.float, s.hgX)
        assertEquals(side["hg_y"]!!.jsonPrimitive.float, s.hgY)
        assertEquals(side["hg_z"]!!.jsonPrimitive.float, s.hgZ)
    }

    @Test
    fun `sensor cal accepts exactly 19 bytes and rejects 18`() {
        assertNotNull(SensorCalStatus.decode(ByteArray(19)))
        assertNull(SensorCalStatus.decode(ByteArray(18)))
    }

    // --- 0xCC rocket storage ---

    @Test
    fun `rocket storage decodes field-for-field`() {
        val rel = "fileops/storage_rocket_0xCC.bin"
        val side = WireFixtures.sidecar(rel)
        val frame = WireFixtures.bytes(rel)
        assertEquals(0xCC.toByte(), frame[0])
        val s = RocketStorageStats.decode(frame.copyOfRange(1, frame.size))
        assertNotNull(s)
        assertEquals(side["flight_region_blocks"]!!.jsonPrimitive.int, s.flightRegionBlocks)
        assertEquals(side["used_blocks"]!!.jsonPrimitive.int, s.usedBlocks)
        assertEquals(side["free_blocks"]!!.jsonPrimitive.int, s.freeBlocks)
        assertEquals(side["bad_blocks"]!!.jsonPrimitive.int, s.badBlocks)
        assertEquals(side["system_blocks"]!!.jsonPrimitive.int, s.systemBlocks)
        assertEquals(side["flight_count"]!!.jsonPrimitive.int, s.flightCount)
        assertEquals(side["block_size_kb"]!!.jsonPrimitive.int, s.blockSizeKb)
        val flags = side["flags"]!!.jsonPrimitive.int
        assertEquals((flags and 0x01) != 0, s.initialized)
        assertEquals((flags and 0x02) != 0, s.autoEvicted)
        // Derived byte accounting (iOS computed properties).
        val blockBytes = s.blockSizeKb.toLong() * 1024
        assertEquals((s.flightRegionBlocks + s.systemBlocks) * blockBytes, s.totalBytes)
        assertEquals(s.usedBlocks * blockBytes, s.usedBytes)
        assertEquals((s.badBlocks + s.systemBlocks) * blockBytes, s.reservedBytes)
        assertEquals(s.freeBlocks * blockBytes, s.freeBytes)
    }

    @Test
    fun `rocket storage accepts exactly 15 bytes and rejects 14`() {
        assertNotNull(RocketStorageStats.decode(ByteArray(15)))
        assertNull(RocketStorageStats.decode(ByteArray(14)))
    }

    // --- 0xCD base-station storage ---

    @Test
    fun `bs storage decodes u64 byte counts field-for-field`() {
        val rel = "fileops/storage_bs_0xCD.bin"
        val side = WireFixtures.sidecar(rel)
        val frame = WireFixtures.bytes(rel)
        assertEquals(0xCD.toByte(), frame[0])
        val s = BaseStationStorageStats.decode(frame.copyOfRange(1, frame.size))
        assertNotNull(s)
        assertEquals(side["total_bytes"]!!.jsonPrimitive.long, s.totalBytes)   // 512 GB > INT32_MAX
        assertEquals(side["used_bytes"]!!.jsonPrimitive.long, s.usedBytes)
        assertEquals(side["free_bytes"]!!.jsonPrimitive.long, s.freeBytes)
        assertEquals(side["backend"]!!.jsonPrimitive.int, s.backend)
        assertEquals((side["flags"]!!.jsonPrimitive.int and 0x01) != 0, s.mounted)
        assertEquals(maxOf(0L, s.totalBytes - s.usedBytes - s.freeBytes), s.reservedBytes)
    }

    @Test
    fun `bs storage accepts exactly 26 bytes and rejects 25`() {
        assertNotNull(BaseStationStorageStats.decode(ByteArray(26)))
        assertNull(BaseStationStorageStats.decode(ByteArray(25)))
    }
}
