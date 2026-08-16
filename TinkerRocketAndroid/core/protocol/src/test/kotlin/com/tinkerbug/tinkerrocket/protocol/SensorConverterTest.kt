package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

/**
 * Port of iOS SensorConverterTests.swift — its numeric pins are the spec
 * (notably the 0.14 dps/LSB gyro pin from #369/#564).  The raw structs
 * expose only binary decoders, so each test builds a byte blob with the
 * fields it needs and lets the struct's decode() parse it, exactly like the
 * iOS tests drive init(from:).  Golden-bridge tests at the bottom run the
 * converter over the shared C++-emitted wire corpus.
 */

/** Little-endian byte packer — test-side inverse of LeBuffer. */
private class TestLeWriter {
    private val out = ArrayList<Byte>()
    fun u8(v: Int) { out.add((v and 0xFF).toByte()) }
    fun i16(v: Int) { u8(v); u8(v shr 8) }
    fun i32(v: Int) { u8(v); u8(v shr 8); u8(v shr 16); u8(v shr 24) }
    fun u16(v: Int) = i16(v)
    fun u32(v: Long) = i32(v.toInt())
    fun toByteArray(): ByteArray = out.toByteArray()
}

private fun makeIsm6(
    timeUs: Long = 0,
    accLow: Triple<Int, Int, Int> = Triple(0, 0, 0),
    accHigh: Triple<Int, Int, Int> = Triple(0, 0, 0),
    gyro: Triple<Int, Int, Int> = Triple(0, 0, 0),
): Ism6hg256Data {
    val w = TestLeWriter()
    w.u32(timeUs)
    w.i16(accLow.first); w.i16(accLow.second); w.i16(accLow.third)
    w.i16(accHigh.first); w.i16(accHigh.second); w.i16(accHigh.third)
    w.i16(gyro.first); w.i16(gyro.second); w.i16(gyro.third)
    return assertNotNull(Ism6hg256Data.decode(w.toByteArray()))
}

private fun makeBmp585(timeUs: Long = 0, tempC: Double, pressurePa: Double): Bmp585Data {
    val w = TestLeWriter()
    w.u32(timeUs)
    w.i32((tempC * 65536.0).toInt())      // temp_q16
    w.u32((pressurePa * 64.0).toLong())   // press_q6
    return assertNotNull(Bmp585Data.decode(w.toByteArray()))
}

private fun makeMag(timeUs: Long = 0, x: Long, y: Long, z: Long): Mmc5983Data {
    val w = TestLeWriter()
    w.u32(timeUs)
    w.u32(x); w.u32(y); w.u32(z)
    return assertNotNull(Mmc5983Data.decode(w.toByteArray()))
}

private fun makeIis2mdc(timeUs: Long = 0, x: Int, y: Int, z: Int): Iis2mdcData {
    val w = TestLeWriter()
    w.u32(timeUs)
    w.i16(x); w.i16(y); w.i16(z)
    return assertNotNull(Iis2mdcData.decode(w.toByteArray()))
}

private fun makeGnss(
    timeUs: Long = 0,
    latE7: Int, lonE7: Int, altMm: Int,
    numSats: Int = 0, pdopX10: Int = 0,
): GnssData {
    val w = TestLeWriter()
    w.u32(timeUs)
    w.u16(0)                  // year
    w.u8(0); w.u8(0)          // month, day
    w.u8(0); w.u8(0); w.u8(0) // hour, minute, second
    w.u16(0)                  // milli_second
    w.u8(0)                   // fix_mode
    w.u8(numSats); w.u8(pdopX10)
    w.i32(latE7); w.i32(lonE7); w.i32(altMm)
    w.i32(0); w.i32(0); w.i32(0)  // velocities
    w.u8(0); w.u8(0)          // h_acc, v_acc
    return assertNotNull(GnssData.decode(w.toByteArray()))
}

private fun makeNonSensor(
    timeUs: Long = 0,
    flags: Int = 0,
    rocketState: Int = 0,
    pyroStatus: Int = 0,
    apogeeFlags: Int = 0,
): NonSensorData {
    val w = TestLeWriter()
    w.u32(timeUs)
    // q0..q3 + roll_cmd (5 × Int16 = 10 bytes)
    repeat(5) { w.i16(0) }
    // e/n/u_pos + e/n/u_vel (6 × Int32 = 24 bytes)
    repeat(6) { w.i32(0) }
    w.u8(flags); w.u8(rocketState)
    w.i16(0)                  // baro_alt_rate_dmps
    w.u8(pyroStatus); w.u8(apogeeFlags)
    return assertNotNull(NonSensorData.decode(w.toByteArray()))
}

class SensorConverterTest {

    private val converter = SensorConverter()

    // MARK: - IMU

    @Test
    fun `IMU zero raw gives zero SI`() {
        val raw = makeIsm6(timeUs = 1000)
        val si = converter.convertISM6HG256(raw)

        assertEquals(1000L, si.timeUs)
        assertEquals(0.0, si.lowGAccX, 1e-6)
        assertEquals(0.0, si.lowGAccY, 1e-6)
        assertEquals(0.0, si.lowGAccZ, 1e-6)
        assertEquals(0.0, si.gyroX, 1e-6)
        assertEquals(0.0, si.gyroY, 1e-6)
        assertEquals(0.0, si.gyroZ, 1e-6)
    }

    @Test
    fun `IMU full scale converts correctly`() {
        // 16g full-scale, 16-bit two's complement: raw 32767 → (16*g) * 32767/32768.
        val raw = makeIsm6(accLow = Triple(32767, 0, 0))
        val si = converter.convertISM6HG256(raw)

        val expected = 16.0 * 9.80665 * (32767.0 / 32768.0)
        assertEquals(expected, si.lowGAccX, 0.1)
    }

    @Test
    fun `IMU gyro sensitivity matches firmware 369 fix`() {
        // #564: ST gyros use a FIXED per-FS sensitivity, NOT FS/32768. The
        // ISM6HG256 datasheet gives 8.75 mdps/LSB at ±250 dps, doubling each
        // FS step → 140 mdps/LSB at ±4000 (mdps/LSB = FS * 0.035). The
        // firmware converter was fixed in #369 (TR_Sensor_Data_Converter.cpp);
        // this pins the app converter to the same 0.14 dps/LSB so the two
        // can't drift apart again. The stale FS/32768 form (122.07 mdps/LSB)
        // understated every post-flight CSV gyro value by ~12.8%.
        val raw = makeIsm6(gyro = Triple(1000, -2000, 500))
        val si = converter.convertISM6HG256(raw)

        // Default zero rotation: raw maps straight through at 0.14 dps/LSB.
        assertEquals(140.0, si.gyroX, 1e-9)
        assertEquals(-280.0, si.gyroY, 1e-9)
        assertEquals(70.0, si.gyroZ, 1e-9)

        // Guard the exact regression: FS/32768 would give 122.07 dps here.
        val staleDps = 1000.0 * (4000.0 / 32768.0)
        assertTrue(
            abs(si.gyroX - staleDps) > 0.01,
            "gyro scale regressed to the pre-#369 FS/32768 form",
        )
    }

    // MARK: - GNSS

    @Test
    fun `GNSS lat lon alt`() {
        val raw = makeGnss(
            latE7 = 337000000,     // 33.7 deg
            lonE7 = -1184000000,   // -118.4 deg
            altMm = 150000,        // 150 m
            numSats = 12,
            pdopX10 = 15,          // PDOP 1.5
        )
        val si = converter.convertGNSS(raw)

        assertEquals(33.7, si.lat, 1e-6)
        assertEquals(-118.4, si.lon, 1e-6)
        assertEquals(150.0, si.alt, 1e-3)
        assertEquals(12, si.numSats)
        assertEquals(1.5, si.pdop, 0.01)
    }

    // MARK: - Baro

    @Test
    fun `baro conversion`() {
        val raw = makeBmp585(timeUs = 5000, tempC = 25.0, pressurePa = 101325.0)
        val si = converter.convertBMP585(raw)

        assertEquals(25.0, si.temperature, 0.01)
        assertEquals(101325.0, si.pressure, 1.0)
    }

    // MARK: - Magnetometer

    @Test
    fun `mag zero centered`() {
        // Centre value is 2^17 = 131072; converted output should be 0.
        val raw = makeMag(timeUs = 2000, x = 131072, y = 131072, z = 131072)
        val si = converter.convertMMC5983MA(raw)

        assertEquals(0.0, si.magX, 0.01)
        assertEquals(0.0, si.magY, 0.01)
        assertEquals(0.0, si.magZ, 0.01)
    }

    @Test
    fun `IIS2MDC zero raw`() {
        // Raw 0 -> 0 µT; signed-int axes so no centering needed.
        val raw = makeIis2mdc(timeUs = 3000, x = 0, y = 0, z = 0)
        val si = converter.convertIIS2MDC(raw)

        assertEquals(3000L, si.timeUs)
        assertEquals(0.0, si.magX, 1e-9)
        assertEquals(0.0, si.magY, 1e-9)
        assertEquals(0.0, si.magZ, 1e-9)
    }

    @Test
    fun `IIS2MDC sensitivity is 0_15 uT per LSB`() {
        // Datasheet 9.13: 0.15 µT/LSB.  With zero rotation (default),
        // raw 100 should map directly to 15 µT on each axis.
        val raw = makeIis2mdc(x = 100, y = 200, z = -100)
        val si = converter.convertIIS2MDC(raw)

        assertEquals(15.0, si.magX, 1e-6)
        assertEquals(30.0, si.magY, 1e-6)
        assertEquals(-15.0, si.magZ, 1e-6)
    }

    @Test
    fun `QMC5883P mag scale applies after configureMagScale`() {
        // The mini's #797 mag runs ±8 G at 3750 LSB/gauss = 100/3750 µT/LSB
        // — keyed off the status query's v6 mag_type.  raw 3750 = 1 gauss
        // = 100 µT.
        val c = SensorConverter()
        c.configureMagScale(OutStatusQueryData.QMC5883P_UT_PER_LSB)
        val si = c.convertIIS2MDC(makeIis2mdc(x = 3750, y = -3750, z = 375))

        assertEquals(100.0, si.magX, 1e-6)
        assertEquals(-100.0, si.magY, 1e-6)
        assertEquals(10.0, si.magZ, 1e-6)
    }

    // MARK: - NonSensor flag extraction (#196)

    @Test
    fun `nonSensor burnout flag false when bit clear`() {
        // No NSF_BURNOUT bit set
        val raw = makeNonSensor(flags = 0b0000_0000)
        val si = converter.convertNonSensor(raw)
        assertFalse(
            si.burnoutFlag,
            "burnout_flag should be false when NSF_BURNOUT bit is clear",
        )
    }

    @Test
    fun `nonSensor burnout flag true when bit set`() {
        // NSF_BURNOUT is bit 4 (1 << 4 = 0x10)
        val raw = makeNonSensor(flags = 1 shl 4)
        val si = converter.convertNonSensor(raw)
        assertTrue(
            si.burnoutFlag,
            "burnout_flag should be true when NSF_BURNOUT bit is set",
        )
    }

    @Test
    fun `nonSensor burnout flag independent of other flags`() {
        // All other NSF_* bits set, NSF_BURNOUT clear
        val otherBits = (1 shl 0) or (1 shl 1) or (1 shl 2) or (1 shl 3) or
            (1 shl 5) or (1 shl 6) or (1 shl 7)
        val raw = makeNonSensor(flags = otherBits)
        val si = converter.convertNonSensor(raw)
        assertFalse(
            si.burnoutFlag,
            "burnout_flag must not be set by neighboring flag bits",
        )
        // Sanity: the bits we DID set should be reflected.
        assertTrue(si.altLandedFlag)
        assertTrue(si.altApogeeFlag)
        assertTrue(si.velUApogeeFlag)
        assertTrue(si.launchFlag)
    }

    // MARK: - Rotation pins (Kotlin additions — not in the iOS suite, added to
    // pin the x'=xc−ys / y'=xs+yc convention and the #204 IIS fallback)

    @Test
    fun `mini rotation 90 degrees maps plus-x gyro onto plus-y`() {
        val c = SensorConverter()
        c.configureMiniRotation(imuDeg = 90.0, magDeg = 0.0)
        val si = c.convertISM6HG256(makeIsm6(gyro = Triple(1000, 0, 0)))
        assertEquals(0.0, si.gyroX, 1e-9)     // x' = x·cos90 − y·sin90
        assertEquals(140.0, si.gyroY, 1e-9)   // y' = x·sin90 + y·cos90
    }

    @Test
    fun `IIS rotation falls back to MMC angle when iisDeg absent`() {
        // #204: format_version < 4 logs carry no iis2mdc_rot_z_cdeg — the
        // IIS2MDC path must reuse the MMC angle then.
        val fallback = SensorConverter()
        fallback.configureMiniRotation(imuDeg = 0.0, magDeg = 180.0, iisDeg = null)
        val si = fallback.convertIIS2MDC(makeIis2mdc(x = 100, y = 200, z = -100))
        assertEquals(-15.0, si.magX, 1e-6)
        assertEquals(-30.0, si.magY, 1e-6)
        assertEquals(-15.0, si.magZ, 1e-6)  // z is never rotated

        // And with an explicit iisDeg the MMC angle must NOT leak in.
        val explicit = SensorConverter()
        explicit.configureMiniRotation(imuDeg = 0.0, magDeg = 180.0, iisDeg = 0.0)
        val si2 = explicit.convertIIS2MDC(makeIis2mdc(x = 100, y = 200, z = -100))
        assertEquals(15.0, si2.magX, 1e-6)
        assertEquals(30.0, si2.magY, 1e-6)
    }

    // MARK: - Golden bridges (decode C++-emitted fixture, convert, compare
    // against sidecar raw values scaled by the pinned constants)

    @Test
    fun `golden imu_ism6_22 converts with pinned scales`() {
        val side = WireFixtures.sidecar("logframes/imu_ism6_22.bin")
        val raw = assertNotNull(Ism6hg256Data.decode(WireFixtures.bytes("logframes/imu_ism6_22.bin")))
        val si = converter.convertISM6HG256(raw)

        val accLow = side["acc_low_raw"]!!.jsonArray.map { it.jsonPrimitive.int }
        val accHigh = side["acc_high_raw"]!!.jsonArray.map { it.jsonPrimitive.int }
        val gyro = side["gyro_raw"]!!.jsonArray.map { it.jsonPrimitive.int }

        // Pinned scales, written out independently of the converter internals:
        // accel mg/LSB = FS·1000/32768; gyro dps/LSB = FS·0.035·1e-3 = 0.14.
        val lowScale = 16.0 * 9.80665 / 32768.0
        val highScale = 256.0 * 9.80665 / 32768.0
        val gyroScale = 0.14

        assertEquals(side["time_us"]!!.jsonPrimitive.long, si.timeUs)  // 2147483648 > Int32.MAX
        assertEquals(accLow[0] * lowScale, si.lowGAccX, 1e-9)
        assertEquals(accLow[1] * lowScale, si.lowGAccY, 1e-9)
        assertEquals(accLow[2] * lowScale, si.lowGAccZ, 1e-9)
        assertEquals(accHigh[0] * highScale, si.highGAccX, 1e-9)
        assertEquals(accHigh[1] * highScale, si.highGAccY, 1e-9)
        assertEquals(accHigh[2] * highScale, si.highGAccZ, 1e-9)
        assertEquals(gyro[0] * gyroScale, si.gyroX, 1e-9)
        assertEquals(gyro[1] * gyroScale, si.gyroY, 1e-9)
        assertEquals(gyro[2] * gyroScale, si.gyroZ, 1e-9)
    }

    @Test
    fun `golden mag_mmc5983_16 centers and scales`() {
        val side = WireFixtures.sidecar("logframes/mag_mmc5983_16.bin")
        val raw = assertNotNull(Mmc5983Data.decode(WireFixtures.bytes("logframes/mag_mmc5983_16.bin")))
        val si = converter.convertMMC5983MA(raw)

        fun expect(field: String): Double {
            val v = side[field]!!.jsonPrimitive.long
            return ((v and 0x3FFFFL) - 131072L) * (800.0 / 131072.0)
        }

        assertEquals(side["time_us"]!!.jsonPrimitive.long, si.timeUs)
        assertEquals(expect("mag_x"), si.magX, 1e-9)
        assertEquals(expect("mag_y"), si.magY, 1e-9)
        assertEquals(expect("mag_z"), si.magZ, 1e-9)
    }

    @Test
    fun `golden mag_iis2mdc_10 scales at 0_15 uT per LSB`() {
        val side = WireFixtures.sidecar("logframes/mag_iis2mdc_10.bin")
        val raw = assertNotNull(Iis2mdcData.decode(WireFixtures.bytes("logframes/mag_iis2mdc_10.bin")))
        val si = converter.convertIIS2MDC(raw)

        assertEquals(side["time_us"]!!.jsonPrimitive.long, si.timeUs)
        assertEquals(side["mag_x"]!!.jsonPrimitive.int * 0.15, si.magX, 1e-9)
        assertEquals(side["mag_y"]!!.jsonPrimitive.int * 0.15, si.magY, 1e-9)
        assertEquals(side["mag_z"]!!.jsonPrimitive.int * 0.15, si.magZ, 1e-9)
    }

    @Test
    fun `golden power_10 converts voltage current soc`() {
        val side = WireFixtures.sidecar("logframes/power_10.bin")
        val raw = assertNotNull(PowerData.decode(WireFixtures.bytes("logframes/power_10.bin")))
        val si = converter.convertPOWER(raw)

        assertEquals(side["time_us"]!!.jsonPrimitive.long, si.timeUs)
        assertEquals(side["voltage_raw"]!!.jsonPrimitive.int / 65535.0 * 10.0, si.voltage, 1e-9)
        assertEquals(side["current_raw"]!!.jsonPrimitive.int / 32767.0 * 10000.0, si.current, 1e-9)
        assertEquals(side["soc_raw"]!!.jsonPrimitive.int * 150.0 / 32767.0 - 25.0, si.soc, 1e-9)
    }

    @Test
    fun `golden baro_bmp585_12 converts q-formats`() {
        val side = WireFixtures.sidecar("logframes/baro_bmp585_12.bin")
        val raw = assertNotNull(Bmp585Data.decode(WireFixtures.bytes("logframes/baro_bmp585_12.bin")))
        val si = converter.convertBMP585(raw)

        assertEquals(side["time_us"]!!.jsonPrimitive.long, si.timeUs)  // 4e9 > Int32.MAX
        assertEquals(side["temp_q16"]!!.jsonPrimitive.long / 65536.0, si.temperature, 1e-9)
        assertEquals(side["press_q6"]!!.jsonPrimitive.long / 64.0, si.pressure, 1e-9)
    }

    @Test
    fun `golden nonsensor_50 flags state and ekf_ticks`() {
        val side = WireFixtures.sidecar("logframes/nonsensor_50.bin")
        val raw = assertNotNull(NonSensorData.decode(WireFixtures.bytes("logframes/nonsensor_50.bin")))
        val si = converter.convertNonSensor(raw)

        assertEquals(side["time_us"]!!.jsonPrimitive.long, si.timeUs)
        assertEquals(side["roll_cmd"]!!.jsonPrimitive.int / 100.0, si.rollCmd, 1e-9)
        assertEquals(side["e_pos"]!!.jsonPrimitive.int * 0.01, si.ePos, 1e-9)
        assertEquals(side["n_pos"]!!.jsonPrimitive.int * 0.01, si.nPos, 1e-9)
        assertEquals(side["u_pos"]!!.jsonPrimitive.int * 0.01, si.uPos, 1e-9)
        assertEquals(side["e_vel"]!!.jsonPrimitive.int * 0.01, si.eVel, 1e-9)
        assertEquals(side["n_vel"]!!.jsonPrimitive.int * 0.01, si.nVel, 1e-9)
        assertEquals(side["u_vel"]!!.jsonPrimitive.int * 0.01, si.uVel, 1e-9)
        assertEquals(side["baro_alt_rate_dmps"]!!.jsonPrimitive.int * 0.1, si.altitudeRate, 1e-9)
        assertEquals(side["q0"]!!.jsonPrimitive.int / 10000.0, si.q0, 1e-9)
        assertEquals(side["q1"]!!.jsonPrimitive.int / 10000.0, si.q1, 1e-9)
        assertEquals(side["q2"]!!.jsonPrimitive.int / 10000.0, si.q2, 1e-9)
        assertEquals(side["q3"]!!.jsonPrimitive.int / 10000.0, si.q3, 1e-9)

        // flags = 45 (0b101101), apogee_flags = 5 (0b101), pyro_status = 10 (0b1010)
        val flags = side["flags"]!!.jsonPrimitive.int
        assertEquals((flags and (1 shl 0)) != 0, si.altLandedFlag)
        assertEquals((flags and (1 shl 1)) != 0, si.altApogeeFlag)
        assertEquals((flags and (1 shl 2)) != 0, si.velUApogeeFlag)
        assertEquals((flags and (1 shl 3)) != 0, si.launchFlag)
        assertEquals((flags and (1 shl 4)) != 0, si.burnoutFlag)

        val af = side["apogee_flags"]!!.jsonPrimitive.int
        assertEquals((af and (1 shl 0)) != 0, si.gpsApogeeFlag)
        assertEquals((af and (1 shl 1)) != 0, si.pitchApogeeFlag)
        assertEquals((af and (1 shl 2)) != 0, si.apogeeFlag)
        assertEquals((af and (1 shl 3)) != 0, si.rebootRecovery)
        assertEquals((af and (1 shl 4)) != 0, si.guidanceEnabled)

        val ps = side["pyro_status"]!!.jsonPrimitive.int
        assertEquals((ps and (1 shl 0)) != 0, si.pyro1Continuity)
        assertEquals((ps and (1 shl 1)) != 0, si.pyro1Fired)
        assertEquals((ps and (1 shl 2)) != 0, si.pyro2Continuity)
        assertEquals((ps and (1 shl 3)) != 0, si.pyro2Fired)
        assertEquals((ps and (1 shl 4)) != 0, si.pyro3Continuity)
        assertEquals((ps and (1 shl 5)) != 0, si.pyro3Fired)
        assertEquals((ps and (1 shl 6)) != 0, si.pyro4Continuity)
        assertEquals((ps and (1 shl 7)) != 0, si.pyro4Fired)

        // rocket_state = 6 is outside the enum → iOS `?? .initialization` fallback.
        assertEquals(RocketState.INITIALIZATION, si.rocketState)

        // #529: ekf_ticks carried through verbatim on the 50-byte layout.
        assertEquals(side["ekf_ticks"]!!.jsonPrimitive.int, si.ekfTicks)
    }
}
