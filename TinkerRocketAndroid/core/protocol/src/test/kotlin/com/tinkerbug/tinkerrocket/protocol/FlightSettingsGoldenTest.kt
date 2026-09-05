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
 * FlightSettingsData version ladder (v1/v2/v3/v5/v6) — golden walk mirroring
 * the iOS GoldenVectorTests.testFlightSettingsLadder, plus dual-gate edge
 * pins the fixtures can't express (version passes but length fails, and
 * vice versa).
 */
class FlightSettingsGoldenTest {

    private val ladder = listOf(
        "logframes/flightsettings_v1_188.bin", "logframes/flightsettings_v2_200.bin",
        "logframes/flightsettings_v3_208.bin", "logframes/flightsettings_v5_210.bin",
        "logframes/flightsettings_v6_219.bin", "logframes/flightsettings_v7_220.bin",
        "logframes/flightsettings_v8_222.bin",
    )

    @Test
    fun `ladder decodes field-for-field with version tails dual-gated`() {
        for (rel in ladder) {
            val side = WireFixtures.sidecar(rel)
            val s = FlightSettingsData.decode(WireFixtures.bytes(rel))
            assertNotNull(s, rel)

            // Base (v1) fields — present at every rung.
            assertEquals(side["time_us"]!!.jsonPrimitive.long, s.timeUs, rel)
            assertEquals(side["version"]!!.jsonPrimitive.int, s.version, rel)
            assertEquals(side["flags"]!!.jsonPrimitive.int, s.flags, rel)
            assertEquals(side["roll_delay_ms"]!!.jsonPrimitive.int, s.rollDelayMs, rel)
            assertEquals(side["kp"]!!.jsonPrimitive.float, s.kp, rel)
            assertEquals(side["ki"]!!.jsonPrimitive.float, s.ki, rel)
            assertEquals(side["kd"]!!.jsonPrimitive.float, s.kd, rel)
            assertEquals(side["d_lpf_hz"]!!.jsonPrimitive.float, s.dLpfHz, rel)
            assertEquals(side["min_cmd_deg"]!!.jsonPrimitive.float, s.minCmdDeg, rel)
            assertEquals(side["max_cmd_deg"]!!.jsonPrimitive.float, s.maxCmdDeg, rel)
            assertEquals(side["kp_angle"]!!.jsonPrimitive.float, s.kpAngle, rel)
            assertEquals(side["kp_angle_rate_cap_dps"]!!.jsonPrimitive.float, s.kpAngleRateCapDps, rel)
            assertEquals(side["gs_v_ref"]!!.jsonPrimitive.float, s.gsVRef, rel)
            assertEquals(side["gs_v_min"]!!.jsonPrimitive.float, s.gsVMin, rel)
            assertEquals(side["gs_scale_cap"]!!.jsonPrimitive.float, s.gsScaleCap, rel)
            assertEquals(side["roll_rate_set_point"]!!.jsonPrimitive.float, s.rollRateSetPoint, rel)
            assertEquals(side["ism6_low_g_fs_g"]!!.jsonPrimitive.int, s.ism6LowGFsG, rel)
            assertEquals(side["ism6_high_g_fs_g"]!!.jsonPrimitive.int, s.ism6HighGFsG, rel)
            assertEquals(side["ism6_gyro_fs_dps"]!!.jsonPrimitive.int, s.ism6GyroFsDps, rel)
            assertEquals(side["servo_bias_us"]!!.jsonArray.map { it.jsonPrimitive.int },
                s.servoBiasUs, rel)
            assertEquals(side["servo_hz"]!!.jsonPrimitive.int, s.servoHz, rel)
            assertEquals(side["servo_min_us"]!!.jsonPrimitive.int, s.servoMinUs, rel)
            assertEquals(side["servo_max_us"]!!.jsonPrimitive.int, s.servoMaxUs, rel)
            assertEquals(side["camera_type"]!!.jsonPrimitive.int, s.cameraType, rel)
            assertEquals(side["fw_git_sha"]!!.jsonPrimitive.content, s.fwGitSha, rel)
            assertEquals(side["roll_profile_num_waypoints"]!!.jsonPrimitive.int, s.numWaypoints, rel)
            assertEquals(s.numWaypoints, s.waypoints.size, rel)

            // Version tails — the dual (version, length) gates.
            val present = side["present_bytes"]!!.jsonPrimitive.int
            if (present >= 200) {
                assertEquals(side["b2r_code"]!!.jsonPrimitive.int, s.b2rCode, rel)
                assertEquals(side["b2r_mode"]!!.jsonPrimitive.int, s.b2rMode, rel)
                assertEquals(side["b2r_residual_cdeg"]!!.jsonPrimitive.int / 100.0f,
                    s.b2rResidualDeg, rel)
            } else {
                assertNull(s.b2rCode, rel)
            }
            if (present >= 208) {
                assertEquals(side["fin_min_deg"]!!.jsonPrimitive.float, s.finMinDeg, rel)
                assertEquals(side["fin_max_deg"]!!.jsonPrimitive.float, s.finMaxDeg, rel)
            } else {
                assertNull(s.finMinDeg, rel)
            }
            if (present >= 210) {
                assertEquals(side["ism6_update_rate_hz"]!!.jsonPrimitive.int,
                    s.ism6UpdateRateHz, rel)
            } else {
                assertNull(s.ism6UpdateRateHz, rel)
            }
            if (present >= 219) {
                assertEquals(side["guid_tgt_e_m"]!!.jsonPrimitive.float, s.guidTgtEM, rel)
                assertEquals(side["guid_tgt_n_m"]!!.jsonPrimitive.float, s.guidTgtNM, rel)
                assertEquals(side["guid_tgt_src"]!!.jsonPrimitive.int, s.guidTgtSrc, rel)
            } else {
                assertNull(s.guidTgtEM, rel)
            }
            if (present >= 220) {
                assertEquals(side["gnss_otp_state"]!!.jsonPrimitive.int, s.gnssOtpState, rel)
            } else {
                assertNull(s.gnssOtpState, rel)
            }
            // v8 roll-control speed gate: deci-m/s on the wire, m/s in the app.
            if (present >= 222) {
                assertEquals(side["roll_min_speed_dmps"]!!.jsonPrimitive.int / 10.0f,
                    s.rollMinSpeedMps, rel)
            } else {
                assertNull(s.rollMinSpeedMps, rel)
            }
        }
    }

    @Test
    fun `accepts exactly 188 bytes and rejects 187`() {
        // 188 zero bytes: version 0, num_waypoints 0 — decodes with no tails.
        assertNotNull(FlightSettingsData.decode(ByteArray(188)))
        assertNull(FlightSettingsData.decode(ByteArray(187)))
    }

    @Test
    fun `v2 frame truncated to 188 leaves b2r null - length side of the dual gate`() {
        val bytes = WireFixtures.bytes("logframes/flightsettings_v2_200.bin").copyOf(188)
        val s = FlightSettingsData.decode(bytes)
        assertNotNull(s)
        assertEquals(2, s.version)
        assertNull(s.b2rCode)
        assertNull(s.b2rMode)
        assertNull(s.b2rResidualDeg)
        assertNull(s.b2rQuat)
    }

    @Test
    fun `200-byte frame claiming v1 leaves b2r null - version side of the dual gate`() {
        val bytes = WireFixtures.bytes("logframes/flightsettings_v2_200.bin").copyOf()
        bytes[4] = 1  // version byte @ offset 4
        val s = FlightSettingsData.decode(bytes)
        assertNotNull(s)
        assertEquals(1, s.version)
        assertNull(s.b2rCode)
    }

    @Test
    fun `flags accessors decode the golden flags byte`() {
        // Golden flags = 79 = 0b1001111: bits 0..3 + 6 set, bit 4/5 clear.
        val s = FlightSettingsData.decode(
            WireFixtures.bytes("logframes/flightsettings_v6_219.bin"))!!
        assertEquals(79, s.flags)
        assertEquals(true, s.useAngleControl)      // bit 0
        assertEquals(true, s.gainScheduleEnabled)  // bit 1
        assertEquals(true, s.guidanceEnabled)      // bit 2
        assertEquals(true, s.servoEnabled)         // bit 3
        assertEquals(false, s.fwDirty)             // bit 4
        assertEquals(false, s.soundsEnabled)       // bit 5
    }

    @Test
    fun `b2r name mapping mirrors firmware orientCodeName`() {
        assertEquals("+X", FlightSettingsData.b2rName(0))
        assertEquals("+X r90", FlightSettingsData.b2rName(1))
        assertEquals("+Y", FlightSettingsData.b2rName(8))
        assertEquals("-Z r270", FlightSettingsData.b2rName(23))
        assertEquals("?", FlightSettingsData.b2rName(24))
    }
}
