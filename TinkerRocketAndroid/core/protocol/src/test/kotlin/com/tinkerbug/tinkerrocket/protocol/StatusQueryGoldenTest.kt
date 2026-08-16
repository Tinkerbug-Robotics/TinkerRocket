package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonPrimitive
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull

/**
 * OutStatusQueryData version ladder (v3/v4/v5/v6) — golden walk mirroring the
 * iOS GoldenVectorTests.testStatusQueryLadder, plus size and #204 dual-gate
 * edge pins.  The v6 rung (+mag_type tail byte, undecoded here) pins that the
 * subset decoder tolerates newer, longer frames.
 */
class StatusQueryGoldenTest {

    @Test
    fun `ladder decodes the iOS-visible subset field-for-field`() {
        for (rel in listOf("logframes/statusquery_v3_26.bin", "logframes/statusquery_v4_28.bin",
                "logframes/statusquery_v5_41.bin", "logframes/statusquery_v6_42.bin")) {
            val side = WireFixtures.sidecar(rel)
            val q = OutStatusQueryData.decode(WireFixtures.bytes(rel))
            assertNotNull(q, rel)
            assertEquals(side["ism6_low_g_fs_g"]!!.jsonPrimitive.int, q.ism6LowGFsG, rel)
            assertEquals(side["ism6_high_g_fs_g"]!!.jsonPrimitive.int, q.ism6HighGFsG, rel)
            assertEquals(side["ism6_gyro_fs_dps"]!!.jsonPrimitive.int, q.ism6GyroFsDps, rel)
            assertEquals(side["ism6_rot_z_cdeg"]!!.jsonPrimitive.int, q.ism6RotZCdeg, rel)
            assertEquals(side["mmc_rot_z_cdeg"]!!.jsonPrimitive.int, q.mmcRotZCdeg, rel)
            assertEquals(side["format_version"]!!.jsonPrimitive.int, q.formatVersion, rel)
            // #204 dual gate: version >= 4 AND length >= 28.
            if (side["format_version"]!!.jsonPrimitive.int >= 4) {
                assertEquals(side["iis2mdc_rot_z_cdeg"]!!.jsonPrimitive.int,
                    q.iis2mdcRotZCdeg, rel)
            } else {
                assertNull(q.iis2mdcRotZCdeg, "iis rotation must be null pre-v4 @ $rel")
            }
            // v6 mag_type: present in the sidecar only on the 42-byte rung.
            val sideMagType = side["mag_type"]
            if (sideMagType != null) {
                assertEquals(sideMagType.jsonPrimitive.int, q.magType, rel)
            } else {
                assertNull(q.magType, "mag_type must be null pre-v6 @ $rel")
            }
        }
    }

    @Test
    fun `version 6 with a 41-byte payload leaves mag_type null - length side of the dual gate`() {
        // 41-byte payload claiming v6: the version gate passes but the
        // length gate (>= 42) fails, so mag_type must stay null.
        val bytes = WireFixtures.bytes("logframes/statusquery_v6_42.bin").copyOf(41)
        val q = OutStatusQueryData.decode(bytes)!!
        assertEquals(6, q.formatVersion)
        assertNull(q.magType)
        assertEquals(OutStatusQueryData.IIS2MDC_UT_PER_LSB, q.magUtPerLsb)
    }

    @Test
    fun `mag_type keys the count scale with IIS2MDC fallback for unknown values`() {
        // The golden v6 fixture carries MAG_TYPE_QMC5883P (the mini's #797 mag).
        val q = OutStatusQueryData.decode(WireFixtures.bytes("logframes/statusquery_v6_42.bin"))!!
        assertEquals(OutStatusQueryData.MAG_TYPE_QMC5883P, q.magType)
        assertEquals(OutStatusQueryData.QMC5883P_UT_PER_LSB, q.magUtPerLsb)
        // An unknown future mag_type must fall back to the IIS2MDC scale,
        // same as a pre-v6 log.
        val unknown = q.copy(magType = 7)
        assertEquals(OutStatusQueryData.IIS2MDC_UT_PER_LSB, unknown.magUtPerLsb)
    }

    @Test
    fun `accepts exactly 10 bytes and rejects 9`() {
        assertNotNull(OutStatusQueryData.decode(ByteArray(10)))
        assertNull(OutStatusQueryData.decode(ByteArray(9)))
    }

    @Test
    fun `version 4 with a short payload leaves iis rotation null - length side of the dual gate`() {
        // 26-byte payload claiming v4: the version gate passes but the
        // length gate (>= 28) fails, so the tail must stay null.
        val bytes = WireFixtures.bytes("logframes/statusquery_v4_28.bin").copyOf(26)
        val q = OutStatusQueryData.decode(bytes)
        assertNotNull(q)
        assertEquals(4, q.formatVersion)
        assertNull(q.iis2mdcRotZCdeg)
    }

    @Test
    fun `rotation degree accessors scale centidegrees by 100`() {
        val q = OutStatusQueryData.decode(WireFixtures.bytes("logframes/statusquery_v4_28.bin"))!!
        assertEquals(90.0, q.imuRotationDeg)     // 9000 cdeg
        assertEquals(-45.0, q.magRotationDeg)    // -4500 cdeg
        assertEquals(180.0, q.iisRotationDeg)    // 18000 cdeg
    }
}
