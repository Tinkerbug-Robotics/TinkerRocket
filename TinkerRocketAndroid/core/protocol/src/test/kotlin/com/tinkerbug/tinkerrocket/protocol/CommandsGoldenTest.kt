package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.double
import kotlinx.serialization.json.float
import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * Encoder-side golden parity — the check iOS never had.  For EVERY fixture in
 * tests_cpp/fixtures/wire/commands/ the frame rebuilt from the sidecar's
 * recorded parameters must be byte-for-byte the emitter's .bin (which the C++
 * compiler packed through the real firmware structs).  A completeness test
 * fails the suite when a new fixture lands without a matching case here.
 *
 * Commands with no fixture (single-byte toggles, name clamp edge cases) are
 * pinned by hand below, porting the iOS XCTest values where they exist
 * (KnownDeviceStoreTests.testNameClampedTo20Bytes).
 */
class CommandsGoldenTest {

    // ------------------------------------------------------------- utilities

    private fun ByteArray.toHex(): String = joinToString("") { "%02x".format(it) }

    /** Byte-for-byte frame parity against a commands/ fixture. */
    private fun assertMatchesFixture(rel: String, built: ByteArray) {
        val expected = WireFixtures.bytes("commands/$rel")
        assertEquals(expected.toHex(), built.toHex(), "frame mismatch for $rel")
    }

    private fun side(rel: String): JsonObject = WireFixtures.sidecar("commands/$rel")

    private fun JsonObject.f(key: String): Float = this[key]!!.jsonPrimitive.float
    private fun JsonObject.d(key: String): Double = this[key]!!.jsonPrimitive.double
    private fun JsonObject.i(key: String): Int = this[key]!!.jsonPrimitive.int
    private fun JsonObject.s(key: String): String = this[key]!!.jsonPrimitive.content

    private fun hexToBytes(hex: String): ByteArray =
        ByteArray(hex.length / 2) { i ->
            hex.substring(2 * i, 2 * i + 2).toInt(16).toByte()
        }

    // Every fixture this suite covers; the completeness test diffs this
    // against the directory so an uncovered fixture is a FAILURE, not a gap.
    private val coveredFixtures = setOf(
        "cmd05_simconfig_16.bin",
        "cmd09_timesync.bin",
        "cmd10_lora_cfg.bin",
        "cmd12_servo_22.bin",
        "cmd13_pid_20.bin",
        "cmd24_servotest_8.bin",
        "cmd26_rollprofile_76.bin",
        "cmd28_guidpoint_20.bin",
        "cmd31_rollctl_16.bin",
        "cmd33_camera_1.bin",
        "cmd34_pyro_24.bin",
        "cmd40_setname.bin",
        "cmd41_setnid.bin",
        "cmd42_setrid.bin",
        "cmd45_setfocus.bin",
        "cmd50_relay_wrapped.bin",
        "cmd55_magcal_apply_14.bin",
        "cmd60_scan_12.bin",
        "cmd62_sensorcal_18.bin",
        "cmd64_imuorient_auto.bin",
        "cmd65_guidance_45.bin",
        "cmd66_fin_18.bin",
        "cmd67_imurate_2.bin",
        "cmd70_otabegin_37.bin",
    )

    @Test
    fun `every command fixture in the corpus has a builder case`() {
        val present = java.io.File(WireFixtures.root, "commands")
            .listFiles { f -> f.name.endsWith(".bin") }!!
            .map { it.name }
            .toSet()
        assertEquals(coveredFixtures, present,
            "commands/ corpus and CommandsGoldenTest coverage diverged")
    }

    // ----------------------------------------------------- golden, per cmd

    @Test
    fun `cmd05 sim config`() {
        val s = side("cmd05_simconfig_16.bin")
        assertMatchesFixture(
            "cmd05_simconfig_16.bin",
            Commands.simConfig(
                mass = s.f("mass_kg"),
                thrustN = s.f("thrust_n"),
                burnTimeS = s.f("burn_time_s"),
                descentRateMps = s.f("descent_rate_mps"),
            ),
        )
    }

    @Test
    fun `cmd09 time sync`() {
        val s = side("cmd09_timesync.bin")
        assertMatchesFixture(
            "cmd09_timesync.bin",
            Commands.timeSync(
                year = s.i("year"), month = s.i("month"), day = s.i("day"),
                hour = s.i("hour"), minute = s.i("minute"), second = s.i("second"),
            ),
        )
    }

    @Test
    fun `cmd10 lora config with negative i8 tx power`() {
        val s = side("cmd10_lora_cfg.bin")
        assertMatchesFixture(
            "cmd10_lora_cfg.bin",
            Commands.loraConfig(
                freqMHz = s.f("freq_mhz"), bwKHz = s.f("bw_khz"),
                sf = s.i("sf"), cr = s.i("cr"), txPowerDbm = s.i("tx_pwr_dbm"),
            ),
        )
    }

    @Test
    fun `cmd12 servo config`() {
        val s = side("cmd12_servo_22.bin")
        assertMatchesFixture(
            "cmd12_servo_22.bin",
            Commands.servoConfig(
                biasesUs = s["bias_us"]!!.jsonArray.map { it.jsonPrimitive.int },
                hz = s.i("hz"), minUs = s.i("min_us"), maxUs = s.i("max_us"),
                finMinDeg = s.f("fin_min_deg"), finMaxDeg = s.f("fin_max_deg"),
            ),
        )
    }

    @Test
    fun `cmd13 pid config`() {
        val s = side("cmd13_pid_20.bin")
        assertMatchesFixture(
            "cmd13_pid_20.bin",
            Commands.pidConfig(
                kp = s.f("kp"), ki = s.f("ki"), kd = s.f("kd"),
                minCmd = s.f("min_cmd"), maxCmd = s.f("max_cmd"),
            ),
        )
    }

    @Test
    fun `cmd24 servo test angles`() {
        val s = side("cmd24_servotest_8.bin")
        // Sidecar records the on-wire centi-degrees; the builder takes degrees
        // (iOS UI units) and does the *100-truncate-clamp itself.
        val anglesDeg = s["angle_cdeg"]!!.jsonArray.map { it.jsonPrimitive.int / 100.0 }
        assertMatchesFixture("cmd24_servotest_8.bin", Commands.servoTestAngles(anglesDeg))
    }

    @Test
    fun `cmd26 roll profile`() {
        val s = side("cmd26_rollprofile_76.bin")
        val waypoints = s["waypoints"]!!.jsonArray.map {
            val w = it.jsonObject
            RollWaypoint(
                timeS = w.f("time_s"),
                angleDeg = w.f("angle_deg"),
                mode = w.i("mode"),
            )
        }
        assertEquals(s.i("num_waypoints"), waypoints.size)
        assertMatchesFixture("cmd26_rollprofile_76.bin", Commands.rollProfile(waypoints))
    }

    @Test
    fun `cmd28 guidance point`() {
        val s = side("cmd28_guidpoint_20.bin")
        assertMatchesFixture(
            "cmd28_guidpoint_20.bin",
            Commands.guidancePoint(
                latDeg = s.d("lat_deg"), lonDeg = s.d("lon_deg"), altitudeM = s.f("alt_m"),
            ),
        )
    }

    @Test
    fun `cmd31 roll control config`() {
        val s = side("cmd31_rollctl_16.bin")
        assertMatchesFixture(
            "cmd31_rollctl_16.bin",
            Commands.rollControlConfig(
                useAngleControl = s.i("use_angle_control") != 0,
                rollDelayMs = s.i("roll_delay_ms"),
                rateCapDps = s.f("kp_angle_rate_cap_dps"),
                kpAngle = s.f("kp_angle"),
                integralSepThreshold = s.f("integral_sep_threshold_dps"),
            ),
        )
    }

    @Test
    fun `cmd33 camera config`() {
        val s = side("cmd33_camera_1.bin")
        assertMatchesFixture("cmd33_camera_1.bin", Commands.cameraConfig(s.i("camera_type")))
    }

    @Test
    fun `cmd34 pyro config`() {
        val s = side("cmd34_pyro_24.bin")
        val channels = s["channels"]!!.jsonArray.map {
            val c = it.jsonObject
            PyroChannelConfig(
                enabled = c.i("enabled") != 0,
                mode = c.i("mode"),
                value = c.f("value"),
            )
        }
        assertMatchesFixture("cmd34_pyro_24.bin", Commands.pyroConfig(channels))
    }

    @Test
    fun `cmd40 set name`() {
        val s = side("cmd40_setname.bin")
        val built = Commands.setUnitName(s.s("name"))
        assertEquals(1 + s.i("payload_len"), built.size)
        assertMatchesFixture("cmd40_setname.bin", built)
    }

    @Test
    fun `cmd41 set network id`() {
        val s = side("cmd41_setnid.bin")
        assertMatchesFixture("cmd41_setnid.bin", Commands.setNetworkId(s.i("nid")))
    }

    @Test
    fun `cmd42 set rocket id`() {
        val s = side("cmd42_setrid.bin")
        assertMatchesFixture("cmd42_setrid.bin", Commands.setRocketId(s.i("rid")))
    }

    @Test
    fun `cmd45 set focus rocket`() {
        val s = side("cmd45_setfocus.bin")
        assertMatchesFixture("cmd45_setfocus.bin", Commands.setFocusRocket(s.i("rid")))
    }

    @Test
    fun `cmd50 relay envelope wraps a complete inner frame`() {
        val s = side("cmd50_relay_wrapped.bin")
        // The fixture's inner frame IS the cmd24 fixture — compose the two
        // builders the way the app does (relay around another builder's frame).
        val inner = WireFixtures.bytes("commands/cmd24_servotest_8.bin")
        assertEquals(s.i("inner_cmd"), inner[0].toInt() and 0xFF)
        assertEquals(s.i("inner_payload_len"), inner.size - 1)
        assertMatchesFixture(
            "cmd50_relay_wrapped.bin",
            Commands.relayToRocket(targetRid = s.i("target_rid"), innerFrame = inner),
        )
    }

    @Test
    fun `cmd55 mag cal apply`() {
        val s = side("cmd55_magcal_apply_14.bin")
        assertMatchesFixture(
            "cmd55_magcal_apply_14.bin",
            Commands.magCalApply(
                cx = s.i("cx"), cy = s.i("cy"), cz = s.i("cz"),
                fieldRuT = s.f("R_uT"), residualUT = s.f("res_uT"),
            ),
        )
    }

    @Test
    fun `cmd60 frequency scan`() {
        val s = side("cmd60_scan_12.bin")
        assertMatchesFixture(
            "cmd60_scan_12.bin",
            Commands.frequencyScan(
                startMHz = s.f("start_mhz"), stopMHz = s.f("stop_mhz"),
                stepKHz = s.i("step_khz"), dwellMs = s.i("dwell_ms"),
            ),
        )
    }

    @Test
    fun `cmd62 sensor cal apply`() {
        val s = side("cmd62_sensorcal_18.bin")
        assertMatchesFixture(
            "cmd62_sensorcal_18.bin",
            Commands.sensorCalApply(
                gyroX = s.i("gyro_x"), gyroY = s.i("gyro_y"), gyroZ = s.i("gyro_z"),
                hgX = s.f("hg_x"), hgY = s.f("hg_y"), hgZ = s.f("hg_z"),
            ),
        )
    }

    @Test
    fun `cmd64 imu orientation auto`() {
        val s = side("cmd64_imuorient_auto.bin")
        assertMatchesFixture("cmd64_imuorient_auto.bin", Commands.imuOrient(s.i("setting")))
    }

    @Test
    fun `cmd65 guidance config 45 bytes`() {
        val s = side("cmd65_guidance_45.bin")
        val built = Commands.guidanceConfig(
            enabled = s.i("enable") != 0,
            navGain = s.f("nav_gain"),
            maxAccel = s.f("max_accel_mps2"),
            accelToFin = s.f("accel_to_fin_deg"),
            maxFinDeg = s.f("max_fin_deg"),
            minSpeed = s.f("min_speed_mps"),
            coastDelayMs = s.i("coast_delay_ms"),
            targetMode = s.i("target_mode"),
            targetE = s.f("target_e_m"),
            targetN = s.f("target_n_m"),
            targetAlt = s.f("target_alt_m"),
            kpPos = s.f("kp_pos_per_s2"),
            kdVel = s.f("kd_vel_per_s"),
            guidanceLaw = s.i("guidance_law"),
        )
        assertEquals(46, built.size, "GuidanceConfigData frame must be 1+45 bytes")
        assertMatchesFixture("cmd65_guidance_45.bin", built)
    }

    @Test
    fun `cmd66 fin config raw form`() {
        val s = side("cmd66_fin_18.bin")
        assertMatchesFixture(
            "cmd66_fin_18.bin",
            Commands.finConfig(
                azimuthDeg = s["azimuth_deg"]!!.jsonArray.map { it.jsonPrimitive.float },
                reverseMask = s.i("reverse_mask"),
                rollReverseMask = s.i("roll_reverse_mask"),
            ),
        )
    }

    @Test
    fun `cmd66 ring-derived form matches the fixture too`() {
        // Fixture: "+" ring, identity slot mapping, reverse_mask 5 = servos 1+3,
        // roll_reverse_mask 2 = servo 2.
        val built = Commands.finConfig(
            ringMode = 0,
            servoAtSlot = listOf(1, 2, 3, 4),
            reverse = listOf(true, false, true, false),
            rollReverse = listOf(false, true, false, false),
        )!!
        assertMatchesFixture("cmd66_fin_18.bin", built)
    }

    @Test
    fun `cmd67 imu rate`() {
        val s = side("cmd67_imurate_2.bin")
        assertMatchesFixture("cmd67_imurate_2.bin", Commands.imuRate(s.i("rate_hz")))
    }

    @Test
    fun `cmd70 ota begin`() {
        val s = side("cmd70_otabegin_37.bin")
        assertMatchesFixture(
            "cmd70_otabegin_37.bin",
            Commands.otaBegin(
                targetIsFC = s.i("target") != 0,
                totalSize = s["size"]!!.jsonPrimitive.long,
                sha256 = hexToBytes(s.s("sha256_hex")),
            ),
        )
    }

    // ----------------------------------- hand pins (no fixture / iOS XCTest)

    @Test
    fun `name clamps to 20 utf8 bytes not characters`() {
        // Ported from iOS KnownDeviceStoreTests.testNameClampedTo20Bytes.
        val xs = Commands.setUnitName("x".repeat(30))
        assertEquals("x".repeat(20), String(xs.copyOfRange(1, xs.size), Charsets.UTF_8))

        // 🚀 is 4 UTF-8 bytes: 10 rockets clamp to 5 (20 bytes), never split.
        val rockets = Commands.setUnitName("🚀".repeat(10))
        assertEquals("🚀".repeat(5),
            String(rockets.copyOfRange(1, rockets.size), Charsets.UTF_8))
        assertEquals(21, rockets.size)   // [cmd] + exactly 20 payload bytes

        // Already-short names pass through untouched.
        val short = Commands.setUnitName("Atlas")
        assertEquals("Atlas", String(short.copyOfRange(1, short.size), Charsets.UTF_8))
    }

    @Test
    fun `single byte payload commands`() {
        assertEquals(listOf(11, 1), Commands.soundsEnable(true).map { it.toInt() })
        assertEquals(listOf(11, 0), Commands.soundsEnable(false).map { it.toInt() })
        assertEquals(listOf(14, 1), Commands.servoEnable(true).map { it.toInt() })
        assertEquals(listOf(17, 1), Commands.loraHopDisabled(true).map { it.toInt() })
        assertEquals(listOf(22, 0), Commands.gainScheduleEnable(false).map { it.toInt() })
        assertEquals(listOf(32, 1), Commands.guidanceEnable(true).map { it.toInt() })
        assertEquals(listOf(35, 2), Commands.pyroContTest(2).map { it.toInt() })
        assertEquals(listOf(36, 3), Commands.pyroFire(3).map { it.toInt() })
        assertEquals(listOf(46, 1), Commands.bsLogging(true).map { it.toInt() })
        assertEquals(listOf(1, 1), Commands.cameraToggleWithState(true).map { it.toInt() })
        assertEquals(listOf(23, 0), Commands.toggleLoggingWithState(false).map { it.toInt() })
        assertEquals(listOf(2, 3), Commands.fileList(3).map { it.toInt() })
        assertEquals(listOf(8), Commands.bare(BleCommandId.POWER_TOGGLE).map { it.toInt() })
        assertEquals(listOf(64, 7), Commands.imuOrient(7).map { it.toInt() })
    }

    @Test
    fun `file name commands carry raw utf8`() {
        val del = Commands.fileDelete("log_001.bin")
        assertEquals(3, del[0].toInt())
        assertEquals("log_001.bin", String(del.copyOfRange(1, del.size), Charsets.UTF_8))
        val dl = Commands.fileDownload("log_001.bin")
        assertEquals(4, dl[0].toInt())
        assertEquals("log_001.bin", String(dl.copyOfRange(1, dl.size), Charsets.UTF_8))
    }

    @Test
    fun `relay caps inner payload at 33 bytes like iOS prefix`() {
        val innerPayload = ByteArray(40) { (it + 1).toByte() }
        val inner = byteArrayOf(24) + innerPayload
        val relayed = Commands.relayToRocket(targetRid = 3, innerFrame = inner)
        // [50][rid][inner_cmd] + first 33 payload bytes.
        assertEquals(3 + 33, relayed.size)
        assertEquals(50, relayed[0].toInt())
        assertEquals(3, relayed[1].toInt())
        assertEquals(24, relayed[2].toInt())
        assertTrue(innerPayload.copyOf(33).contentEquals(relayed.copyOfRange(3, relayed.size)))
    }

    @Test
    fun `roll profile drops waypoints past 8 and caps count byte`() {
        val many = (0 until 10).map { RollWaypoint(it.toFloat(), 0.0f, 0) }
        val built = Commands.rollProfile(many)
        assertEquals(77, built.size)          // [cmd] + 76-byte RollProfileData
        assertEquals(8, built[1].toInt())     // num_waypoints caps at 8
    }

    @Test
    fun `servo test angles truncate then saturate to i16 like Int16 clamping`() {
        val built = Commands.servoTestAngles(listOf(400.0, -400.0, 0.019, -0.019))
        fun i16At(off: Int): Int =
            ((built[off].toInt() and 0xFF) or (built[off + 1].toInt() shl 8))
        assertEquals(32767, i16At(1))    // 40000 saturates high
        assertEquals(-32768, i16At(3))   // -40000 saturates low
        assertEquals(1, i16At(5))        // 1.9 truncates toward zero
        assertEquals(-1, i16At(7))       // -1.9 truncates toward zero
    }

    @Test
    fun `guard rails mirror iOS preconditions`() {
        assertFailsWith<IllegalArgumentException> {
            Commands.pyroConfig(listOf(PyroChannelConfig(true, 0, 1.0f)))
        }
        assertFailsWith<IllegalArgumentException> {
            Commands.otaBegin(targetIsFC = false, totalSize = 10L, sha256 = ByteArray(31))
        }
        assertFailsWith<IllegalArgumentException> {
            Commands.relayToRocket(1, ByteArray(0))
        }
        // iOS sendFinConfig guards-and-returns on bad counts → builder null.
        assertNull(Commands.finConfig(0, listOf(1, 2, 3), listOf(true), listOf(false)))
    }

    @Test
    fun `cross ring mode derives azimuths from occupied slots`() {
        // "×" ring, servos shuffled: slot azimuths {45,135,225,315} land on
        // the servo occupying each slot (per-servo order on the wire).
        val derived = Commands.finConfig(
            ringMode = 1,
            servoAtSlot = listOf(2, 1, 4, 3),
            reverse = listOf(false, false, false, false),
            rollReverse = listOf(false, false, false, false),
        )!!
        val expected = Commands.finConfig(
            azimuthDeg = listOf(135f, 45f, 315f, 225f),
            reverseMask = 0,
            rollReverseMask = 0,
        )
        assertEquals(expected.toHex(), derived.toHex())
    }
}
