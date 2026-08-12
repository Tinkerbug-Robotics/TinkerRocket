package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long
import kotlin.math.roundToInt
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * MagCalStatus 0xCA ladder (22/26/32/36) — golden walk mirroring the iOS
 * GoldenVectorTests.testMagCalLadder — plus the #207 center-magnitude /
 * warning-bucket pins ported from the iOS MagCalStatusTests.
 */
class MagCalStatusTest {

    private val ladder = listOf(
        "fileops/magcal_0xCA_len22.bin", "fileops/magcal_0xCA_len26.bin",
        "fileops/magcal_0xCA_len32.bin", "fileops/magcal_0xCA_len36.bin",
    )

    @Test
    fun `ladder decodes with trailer defaults at every rung`() {
        for (rel in ladder) {
            val side = WireFixtures.sidecar(rel)
            // Fixtures are full frames; decode() takes bytes AFTER the 0xCA.
            val frame = WireFixtures.bytes(rel)
            assertEquals(0xCA.toByte(), frame[0], rel)
            val s = MagCalStatus.decode(frame.copyOfRange(1, frame.size))
            assertNotNull(s, "decode returned null for $rel")

            val present = side["present_bytes"]!!.jsonPrimitive.int
            assertEquals(side["sub_type"]!!.jsonPrimitive.int, s.subType.raw, rel)
            assertEquals(side["reject_code"]!!.jsonPrimitive.int, s.rejectCode.raw, rel)
            assertEquals(side["coverage_bins"]!!.jsonPrimitive.int, s.coverageBins, rel)
            assertEquals(side["sample_count"]!!.jsonPrimitive.int, s.sampleCount, rel)
            assertEquals(
                listOf(side["offset_x"]!!.jsonPrimitive.int, side["offset_y"]!!.jsonPrimitive.int,
                    side["offset_z"]!!.jsonPrimitive.int),
                listOf(s.offsetX, s.offsetY, s.offsetZ), rel)
            assertEquals(side["inst_field_uT_x10"]!!.jsonPrimitive.int / 10.0f,
                s.instantaneousFieldUt, rel)
            assertEquals(side["field_R_uT_x10"]!!.jsonPrimitive.int / 10.0f, s.fieldRUt, rel)
            assertEquals(side["residual_uT_x10"]!!.jsonPrimitive.int / 10.0f, s.residualUt, rel)
            // Trailer defaults: absent rungs decode as 0, by contract.
            assertEquals(
                if (present >= 26) side["coverage_mask"]!!.jsonPrimitive.long else 0L,
                s.coverageMask, rel)
            assertEquals(
                if (present >= 36) side["partial_mask"]!!.jsonPrimitive.long else 0L,
                s.partialMask, rel)
            if (present >= 32) {
                assertEquals(side["inst_x_lsb"]!!.jsonPrimitive.int * 0.15f, s.liveXUt, rel)
                assertEquals(side["inst_y_lsb"]!!.jsonPrimitive.int * 0.15f, s.liveYUt, rel)
                assertEquals(side["inst_z_lsb"]!!.jsonPrimitive.int * 0.15f, s.liveZUt, rel)
            } else {
                assertEquals(listOf(0f, 0f, 0f), listOf(s.liveXUt, s.liveYUt, s.liveZUt), rel)
            }
        }
    }

    @Test
    fun `payload shorter than 22 bytes returns null`() {
        assertNull(MagCalStatus.decode(ByteArray(21)))
        assertNotNull(MagCalStatus.decode(ByteArray(22)))
    }

    // --- #207 center-magnitude pins ported from iOS MagCalStatusTests ---

    /** Synthetic REVIEW-state status with hand-set offset / R fields. */
    private fun makeStatus(offsetX: Int, offsetY: Int, offsetZ: Int, rUt: Float): MagCalStatus =
        MagCalStatus(
            subType = MagCalSubType.REVIEW,
            coverageBins = 26,
            sampleCount = 2000,
            instantaneousFieldUt = rUt,
            offsetX = offsetX,
            offsetY = offsetY,
            offsetZ = offsetZ,
            fieldRUt = rUt,
            residualUt = 2.0f,
            rejectCode = MagCalRejectCode.OK,
            coverageMask = 0L,
            liveXUt = 0f, liveYUt = 0f, liveZUt = 0f,
            partialMask = 0L,
        )

    @Test
    fun `center magnitude basic - sqrt of squares times LSB`() {
        // 0.15 µT/LSB on the IIS2MDC; sqrt(100² + 100² + 100²) * 0.15 ≈ 25.98 µT.
        val s = makeStatus(100, 100, 100, 50.0f)
        assertEquals(25.98f, s.centerMagnitudeUt, 0.01f)
        assertEquals(25.98f / 50.0f, s.centerToRRatio, 0.001f)
    }

    @Test
    fun `center magnitude zero`() {
        val s = makeStatus(0, 0, 0, 50.0f)
        assertEquals(0.0f, s.centerMagnitudeUt)
        assertEquals(0.0f, s.centerToRRatio)
        assertEquals(MagCalStatus.CenterWarning.OK, s.centerWarning)
    }

    @Test
    fun `R of zero must not divide by zero - ratio defined as 0`() {
        val s = makeStatus(100, 0, 0, 0.0f)
        assertTrue(s.centerMagnitudeUt > 0)
        assertEquals(0.0f, s.centerToRRatio)
        assertEquals(MagCalStatus.CenterWarning.OK, s.centerWarning)
    }

    @Test
    fun `Eagle Claw historical residual lands ok`() {
        // 2026-05-17 Eagle Claw: ~41 µT — well under the chip-limit threshold.
        fun toLsb(ut: Float): Int = (ut / 0.15f).roundToInt()
        val s = makeStatus(toLsb(-17.2f), toLsb(30.8f), toLsb(20.4f), 48.7f)
        assertEquals(41.0f, s.centerMagnitudeUt, 0.5f)
        assertEquals(MagCalStatus.CenterWarning.OK, s.centerWarning)
    }

    @Test
    fun `fresh new-PCB board hard-iron stays ok`() {
        // 1700 µT / 0.15 = 11333 LSB; pure-X for simplicity — typical fresh
        // IIS2MDC, nowhere near the chip's ±4915 µT OFFSET register limit.
        val s = makeStatus(11333, 0, 0, 48.0f)
        assertEquals(1700.0f, s.centerMagnitudeUt, 1.0f)
        assertEquals(MagCalStatus.CenterWarning.OK, s.centerWarning)
    }

    @Test
    fun `warning threshold transitions at 4500 uT`() {
        // 29999 lsb × 0.15 = 4499.85 µT → OK (just under).
        assertEquals(MagCalStatus.CenterWarning.OK,
            makeStatus(29999, 0, 0, 50.0f).centerWarning)
        // 30000 lsb × 0.15 = 4500.00 µT → HIGH (== threshold).
        assertEquals(MagCalStatus.CenterWarning.HIGH,
            makeStatus(30000, 0, 0, 50.0f).centerWarning)
    }

    @Test
    fun `unknown enum raws fall back like iOS nil-coalescing`() {
        assertEquals(MagCalSubType.IDLE, MagCalSubType.fromRaw(99))
        assertEquals(MagCalRejectCode.OK, MagCalRejectCode.fromRaw(99))
    }

    @Test
    fun `needsAbortOnLeave covers the states a calibration can be stranded in`() {
        // Leaving during any of these strands the FC in MAG_CALIBRATION with the
        // magnetometer offsets zeroed, and that state has no launch-detect
        // failsafe -- kinematicChecks is skipped in MAG_CALIBRATION, so nothing
        // clears it on its own.
        assertTrue(MagCalSubType.SAMPLING.needsAbortOnLeave)
        assertTrue(MagCalSubType.REVIEW.needsAbortOnLeave)
        // The sharpest one: a 60 s firmware timeout runs evaluateVerify()
        // whether or not the app is watching, so walking away here can commit a
        // permanent NVS calibration nobody reviewed.
        assertTrue(MagCalSubType.VERIFYING.needsAbortOnLeave)
    }

    @Test
    fun `needsAbortOnLeave excludes APPLIED so a saved calibration survives the exit`() {
        // The FC RESTS in APPLIED whenever a calibration exists, and
        // MagCalibrator::abort() forces ABORTED from ANY state including this
        // one. An abort on the way out of a successful save would throw the
        // calibration away -- this is the case the state gate exists for.
        assertFalse(MagCalSubType.APPLIED.needsAbortOnLeave)

        // Nothing running: an abort would be pointless noise on the link.
        assertFalse(MagCalSubType.IDLE.needsAbortOnLeave)
        assertFalse(MagCalSubType.ABORTED.needsAbortOnLeave)
    }
}
