package com.tinkerbug.tinkerrocket.protocol

import java.io.ByteArrayOutputStream
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * CsvGenerator behavior pins against the iOS CSVGenerator.swift reference:
 * ground-pressure window selection (all three fallbacks), the 4-second
 * pre-launch trim, forward-fill semantics (at-or-before), the legacy high-G
 * merge, summary event-time latching (#196 burnout guard, #142 apogee gate),
 * the #514 quat-blank rule, and exact header/row string pins.
 *
 * All frames are synthesized in memory through the real wire framing
 * (preamble + CRC16) so the whole pipeline — MessageParser → decoders →
 * SensorConverter → CsvGenerator — runs end to end.
 */
class CsvGeneratorTest {

    // ------------------------------------------------------------------
    // Wire-frame synthesis helpers
    // ------------------------------------------------------------------

    private class LeWriter {
        private val out = ByteArrayOutputStream()
        fun u8(v: Int) = apply { out.write(v and 0xFF) }
        fun u16(v: Int) = apply { u8(v); u8(v shr 8) }
        fun i16(v: Int) = u16(v)
        fun u32(v: Long) = apply {
            u8(v.toInt()); u8((v shr 8).toInt()); u8((v shr 16).toInt()); u8((v shr 24).toInt())
        }
        fun i32(v: Int) = u32(v.toLong())
        fun bytes(): ByteArray = out.toByteArray()
    }

    private fun frame(type: Int, payload: ByteArray): ByteArray {
        val body = ByteArray(2 + payload.size)
        body[0] = type.toByte()
        body[1] = payload.size.toByte()
        payload.copyInto(body, 2)
        val crc = Crc16.compute(body)
        val out = ByteArrayOutputStream()
        out.write(byteArrayOf(0xAA.toByte(), 0x55, 0xAA.toByte(), 0x55))
        out.write(body)
        out.write((crc shr 8) and 0xFF)
        out.write(crc and 0xFF)
        return out.toByteArray()
    }

    private fun log(vararg frames: ByteArray): ByteArray {
        val out = ByteArrayOutputStream()
        frames.forEach { out.write(it) }
        return out.toByteArray()
    }

    private fun imuMini(
        t: Long,
        lx: Int = 0, ly: Int = 0, lz: Int = 0,
        hx: Int = 0, hy: Int = 0, hz: Int = 0,
        gx: Int = 0, gy: Int = 0, gz: Int = 0,
    ) = frame(
        MsgType.IMU,
        LeWriter().u32(t)
            .i16(lx).i16(ly).i16(lz)
            .i16(hx).i16(hy).i16(hz)
            .i16(gx).i16(gy).i16(gz)
            .bytes(),
    )

    private fun baroMini(t: Long, tempQ16: Int, pressQ6: Long) =
        frame(MsgType.BARO, LeWriter().u32(t).i32(tempQ16).u32(pressQ6).bytes())

    /** pressQ6 = Pa * 64. */
    private fun baroPa(t: Long, pa: Long) = baroMini(t, 25 * 65536, pa * 64)

    private fun gnss(
        t: Long, sats: Int, pdopX10: Int,
        latE7: Int, lonE7: Int, altMm: Int,
        velE: Int, velN: Int, velU: Int,
        hAcc: Int, vAcc: Int,
    ) = frame(
        MsgType.GNSS,
        LeWriter().u32(t)
            .u16(2026).u8(7).u8(22).u8(12).u8(0).u8(0).u16(0)
            .u8(3).u8(sats).u8(pdopX10)
            .i32(latE7).i32(lonE7).i32(altMm)
            .i32(velE).i32(velN).i32(velU)
            .u8(hAcc).u8(vAcc)
            .bytes(),
    )

    private fun magMmc(t: Long, x: Long, y: Long, z: Long) =
        frame(MsgType.MAG, LeWriter().u32(t).u32(x).u32(y).u32(z).bytes())

    private fun power(t: Long, vRaw: Int, cRaw: Int, socRaw: Int) =
        frame(MsgType.POWER, LeWriter().u32(t).u16(vRaw).i16(cRaw).i16(socRaw).bytes())

    /** Mini NonSensor 44-byte (ekf == null) or 50-byte (#529) payload. */
    private fun nonSensor(
        t: Long,
        q0: Int = 10000, q1: Int = 0, q2: Int = 0, q3: Int = 0,
        rollCmd: Int = 0,
        ePos: Int = 0, nPos: Int = 0, uPos: Int = 0,
        eVel: Int = 0, nVel: Int = 0, uVel: Int = 0,
        flags: Int = 0, state: Int = 0, altRate: Int = 0,
        pyro: Int = 0, apogeeFlags: Int = 0, ekf: Int? = null,
    ): ByteArray {
        val w = LeWriter().u32(t)
            .i16(q0).i16(q1).i16(q2).i16(q3).i16(rollCmd)
            .i32(ePos).i32(nPos).i32(uPos)
            .i32(eVel).i32(nVel).i32(uVel)
            .u8(flags).u8(state).i16(altRate).u8(pyro).u8(apogeeFlags)
        if (ekf != null) {
            w.u32(0L)  // sensor_health (#303, never surfaced)
            w.u16(ekf)
        }
        return frame(MsgType.NON_SENSOR, w.bytes())
    }

    private fun legacyImu(
        t: Long,
        ax: Int = 0, ay: Int = 0, az: Int = 0,
        gx: Int = 0, gy: Int = 0, gz: Int = 0,
    ) = frame(
        MsgType.IMU,
        LeWriter().u32(t).u32(0L)
            .i32(ax).i32(ay).i32(az)
            .i32(gx).i32(gy).i32(gz)
            .i32(0)
            .bytes(),
    )

    private fun legacyHighG(t: Long, x: Int, y: Int, z: Int) =
        frame(MsgType.HIGH_G_LEGACY, LeWriter().u32(t).i16(x).i16(y).i16(z).bytes())

    private fun dataRows(csv: String): List<String> =
        csv.trimEnd('\n').split("\n").drop(1)

    private fun fields(row: String): List<String> = row.split(",")

    // NSF_* bit values (flags byte)
    private val launchBit = 1 shl 3
    private val burnoutBit = 1 shl 4
    private val altApogeeBit = 1 shl 1
    private val velApogeeBit = 1 shl 2

    // NSF2_* bit values (apogee_flags byte)
    private val masterApogeeBit = 1 shl 2

    // 0-based CSV column indices used in assertions
    private val colPressure = 20
    private val colPressureAlt = 22

    // ------------------------------------------------------------------
    // Header pin
    // ------------------------------------------------------------------

    @Test
    fun `header is the exact iOS column string`() {
        val expected = "Time (ms)," +
            "Latitude (deg),Longitude (deg),GNSS Altitude (m),Number of Satellites,PDOP," +
            "GNSS East Velocity (m/s),GNSS North Velocity (m/s),GNSS Up Velocity (m/s)," +
            "GNSS Horizontal Accuracy (m),GNSS Vertical Accuracy (m)," +
            "Low-G Acceleration X (m/s2),Low-G Acceleration Y (m/s2),Low-G Acceleration Z (m/s2)," +
            "High-G Acceleration X (m/s2),High-G Acceleration Y (m/s2),High-G Acceleration Z (m/s2)," +
            "Gyro X (deg/s),Gyro Y (deg/s),Gyro Z (deg/s)," +
            "Pressure (Pa),Barometer Temperature (C),Pressure Altitude (m)," +
            "Magnetic Field X (uT),Magnetic Field Y (uT),Magnetic Field Z (uT)," +
            "Voltage (V),Current (mA),State of Charge (%)," +
            "Quat q0,Quat q1,Quat q2,Quat q3," +
            "Roll (deg; body-Z azimuth),Pitch (deg; ZYX Euler),Yaw (deg; ZYX Euler)," +
            "Roll Command (deg)," +
            "Position East (m),Position North (m),Position Up (m)," +
            "Velocity East (m/s),Velocity North (m/s),Velocity Up (m/s)," +
            "Altitude Rate (m/s),Landed Flag," +
            "Apogee Detector: Baro,Apogee Detector: Velocity,Apogee Detector: GPS," +
            "Apogee Detector: Pitch,Apogee Flag (Master),Launch Flag,Deployed Flag," +
            "Pyro 1 Continuity,Pyro 2 Continuity,Pyro 3 Continuity,Pyro 4 Continuity," +
            "Pyro 1 Fired,Pyro 2 Fired,Pyro 3 Fired,Pyro 4 Fired," +
            "Reboot Recovery,FC Guidance Enabled,EKF Ticks\n"
        assertEquals(expected, CsvGenerator().buildCsvHeader())
        // #514 regression: no column name may ever contain a comma — the
        // writer does not quote fields and readers split rows blind.
        assertEquals(63, expected.trimEnd('\n').split(",").size)
    }

    // ------------------------------------------------------------------
    // Row format pin (fully-populated row, hand-computed)
    // ------------------------------------------------------------------

    @Test
    fun `fully populated row matches hand-computed iOS format string`() {
        val bin = log(
            gnss(
                t = 1_000_000, sats = 12, pdopX10 = 18,
                latE7 = 377_749_000, lonE7 = -1_224_194_000, altMm = 100_500,
                velE = 1500, velN = -250, velU = 10_000, hAcc = 2, vAcc = 3,
            ),
            baroPa(1_000_000, 100_000),
            magMmc(1_000_000, 131_072 + 8192L, 131_072 - 4096L, 131_072L),
            power(1_000_000, 65_535, 32_767, 32_767),
            nonSensor(
                t = 1_000_000, q0 = 10_000, rollCmd = 250,
                ePos = 100, nPos = -250, uPos = 12_345,
                eVel = 50, nVel = 0, uVel = 2500,
                flags = 0, state = 1, altRate = 55, pyro = 5, ekf = 1234,
            ),
            imuMini(
                t = 1_000_000,
                lx = 1000, ly = -1000, lz = 2048,
                hx = 100, hy = -100, hz = 50,
                gx = 100, gy = 200, gz = -300,
            ),
        )

        val (csv, _) = CsvGenerator().writeCsv(bin)
        val rows = dataRows(csv)
        assertEquals(1, rows.size)

        // Hand-computed per iOS format specifiers.  Notes:
        //  - low-G 2048 LSB = exactly 1 g = 9.80665 m/s² → "9.806650"
        //  - gyro LSB = 0.14 dps (ST fixed sensitivity, #369/#564)
        //  - ground pressure = the single pre-row baro → pressure alt 0.00
        //  - identity quaternion → roll = -atan2(+0, +0) = -0.0 → "-0.00"
        //    (printf keeps the sign of negative zero; so does iOS)
        //  - pyro_status 0x05 → ch1+ch2 continuity, nothing fired
        val expected = "0.000," +
            "37.7749000,-122.4194000,100.500,12,1.8,1.500,-0.250,10.000,2.0,3.0," +
            "4.788403,-4.788403,9.806650," +
            "7.661445,-7.661445,3.830723," +
            "14.000000,28.000000,-42.000000," +
            "100000.00,25.00,0.00," +
            "50.000000,-25.000000,0.000000," +
            "10.000,10000.0,125.0," +
            "1.00000,0.00000,0.00000,0.00000," +
            "-0.00,0.00,0.00," +
            "2.50," +
            "1.00,-2.50,123.45," +
            "0.50,0.00,25.00," +
            "5.5,0," +
            "0,0,0,0,0,0,0," +
            "1,1,0,0,0,0,0,0," +
            "0,0,1234"
        assertEquals(expected, rows[0])
    }

    // ------------------------------------------------------------------
    // Ground-pressure window selection (three fallbacks)
    // ------------------------------------------------------------------

    @Test
    fun `ground pressure averages baro strictly before launch`() {
        // 3 pre-launch readings (mean 100200), one AT launch time (200000 Pa —
        // must be excluded: the window is strictly-before), one in flight.
        val bin = log(
            baroPa(100_000, 100_000),
            baroPa(200_000, 100_200),
            baroPa(300_000, 100_400),
            baroPa(1_000_000, 200_000),
            nonSensor(t = 1_000_000, flags = launchBit),
            baroPa(1_500_000, 100_200),
            imuMini(1_000_000),
            imuMini(1_500_000),
        )
        val (csv, _) = CsvGenerator().writeCsv(bin)
        val rows = dataRows(csv)
        assertEquals(2, rows.size)
        // Row 2 forward-fills the in-flight baro (100200 Pa) == ground
        // pressure mean → pressure altitude exactly 0.00.
        assertEquals("100200.00", fields(rows[1])[colPressure])
        assertEquals("0.00", fields(rows[1])[colPressureAlt])
    }

    @Test
    fun `no launch detected falls back to last 100 baro readings`() {
        val frames = ArrayList<ByteArray>()
        // 20 warmup readings at 90000 Pa, then 100 at 100000 Pa.
        for (i in 0 until 20) frames.add(baroPa(10_000L + i * 10_000L, 90_000))
        for (i in 0 until 100) frames.add(baroPa(210_000L + i * 10_000L, 100_000))
        frames.add(imuMini(1_200_000))
        val (csv, _) = CsvGenerator().writeCsv(log(*frames.toTypedArray()))
        val row = fields(dataRows(csv)[0])
        // gp = mean(last 100) = 100000; forward-filled baro also 100000 → 0.00
        assertEquals("100000.00", row[colPressure])
        assertEquals("0.00", row[colPressureAlt])
    }

    @Test
    fun `empty pre-launch window falls back to last 10 baro readings`() {
        // Launch before any baro sample → strictly-before filter yields
        // nothing → fall back to the last 10 of all readings.
        val frames = ArrayList<ByteArray>()
        frames.add(nonSensor(t = 500_000, flags = launchBit))
        for (i in 0 until 5) frames.add(baroPa(600_000L + i * 10_000L, 200_000))
        for (i in 0 until 10) frames.add(baroPa(650_000L + i * 10_000L, 100_000))
        frames.add(imuMini(740_000))
        val (csv, _) = CsvGenerator().writeCsv(log(*frames.toTypedArray()))
        val row = fields(dataRows(csv)[0])
        assertEquals("100000.00", row[colPressure])
        assertEquals("0.00", row[colPressureAlt])
    }

    // ------------------------------------------------------------------
    // 4-second pre-launch trim
    // ------------------------------------------------------------------

    @Test
    fun `launch later than 4s trims rows to launch minus 4s`() {
        val frames = ArrayList<ByteArray>()
        for (i in 0..100) frames.add(imuMini(i * 100_000L))  // t = 0 .. 10 s
        frames.add(nonSensor(t = 8_000_000, flags = launchBit))
        val (csv, _) = CsvGenerator().writeCsv(log(*frames.toTypedArray()))
        val rows = dataRows(csv)
        // firstTime = max(0, 8s - 4s) = 4s → rows at 4.0 .. 10.0 s = 61 rows,
        // re-zeroed so the first kept row prints 0.000.
        assertEquals(61, rows.size)
        assertEquals("0.000", fields(rows[0])[0])
        assertEquals("6000.000", fields(rows.last())[0])
    }

    @Test
    fun `launch within first 4s does not trim`() {
        val frames = ArrayList<ByteArray>()
        for (i in 0..100) frames.add(imuMini(i * 100_000L))
        frames.add(nonSensor(t = 3_000_000, flags = launchBit))  // 3s ≤ 4s pad
        val (csv, _) = CsvGenerator().writeCsv(log(*frames.toTypedArray()))
        val rows = dataRows(csv)
        assertEquals(101, rows.size)
        assertEquals("0.000", fields(rows[0])[0])
    }

    // ------------------------------------------------------------------
    // Forward-fill: last sample at-or-before the IMU timestamp
    // ------------------------------------------------------------------

    @Test
    fun `forward fill picks the last sample at or before each row time`() {
        val bin = log(
            imuMini(1000),
            imuMini(2000),
            imuMini(3000),
            baroPa(2000, 100_000),  // at-or-equal: visible from row t=2000
            baroPa(2500, 101_000),  // between rows: visible from row t=3000
        )
        val (csv, _) = CsvGenerator().writeCsv(bin)
        val rows = dataRows(csv)
        assertEquals(3, rows.size)
        assertEquals("", fields(rows[0])[colPressure])  // nothing at/before 1000
        assertEquals("", fields(rows[0])[colPressureAlt])
        assertEquals("100000.00", fields(rows[1])[colPressure])
        assertEquals("101000.00", fields(rows[2])[colPressure])
    }

    // ------------------------------------------------------------------
    // Legacy device: H3LIS331 high-G merged into the IMU row
    // ------------------------------------------------------------------

    @Test
    fun `legacy high-G samples replace the IMU row high-G fields`() {
        val bin = log(
            legacyImu(t = 1000, ax = 10_000, ay = 0, az = 20_000),
            legacyHighG(t = 1000, x = 100, y = 200, z = -300),
        )
        val (csv, _) = CsvGenerator().writeCsv(bin)
        val row = fields(dataRows(csv)[0])
        // Legacy scale + app-side 180° rotation (X/Y negated, Z unchanged):
        // low-G: 10000 * 0.00059855 = 5.9855 → X = -5.985500
        assertEquals("-5.985500", row[11])
        assertEquals("11.971000", row[13])
        // high-G from the separate H3LIS331 frame, 0.95788 m/s²/LSB:
        assertEquals("-95.788000", row[14])
        assertEquals("-191.576000", row[15])
        assertEquals("-287.364000", row[16])
    }

    // ------------------------------------------------------------------
    // Flight-summary event latching
    // ------------------------------------------------------------------

    @Test
    fun `summary latches first launch, burnout and master apogee times`() {
        val frames = ArrayList<ByteArray>()
        for (i in 1..6) frames.add(imuMini(i * 1_000_000L))
        frames.add(nonSensor(t = 1_000_000, flags = 0, uVel = 5_000))                  // 50 m/s
        frames.add(nonSensor(t = 2_000_000, flags = launchBit, uVel = 10_000))         // 100 m/s
        frames.add(nonSensor(t = 3_000_000, flags = launchBit or burnoutBit, uVel = 8_000))
        frames.add(
            nonSensor(
                t = 5_000_000, flags = launchBit or burnoutBit,
                apogeeFlags = masterApogeeBit, uVel = 20_000,  // 200 m/s post-apogee
            ),
        )
        val (_, summary) = CsvGenerator().writeCsv(log(*frames.toTypedArray()))
        assertEquals(1.0, summary.burnoutTimeS)   // 3s - 2s
        assertEquals(3.0, summary.apogeeTimeS)    // 5s - 2s
        // Post-apogee 200 m/s must not win: max speed gated by master flag.
        assertEquals(100.0, summary.maxSpeedMps)
        assertTrue(summary.toJson().contains("\"burnout_time_s\""))
    }

    @Test
    fun `burnout at or before launch yields null burnout time`() {
        // Burnout bit latches at t=1s, launch only at t=2s → burnout > launch
        // fails → null (never a wrong proxy value; #196 guard).
        val frames = ArrayList<ByteArray>()
        for (i in 1..3) frames.add(imuMini(i * 1_000_000L))
        frames.add(nonSensor(t = 1_000_000, flags = burnoutBit))
        frames.add(nonSensor(t = 2_000_000, flags = launchBit or burnoutBit))
        val (_, summary) = CsvGenerator().writeCsv(log(*frames.toTypedArray()))
        assertNull(summary.burnoutTimeS)
        assertNull(summary.apogeeTimeS)
        // iOS synthesized Codable omits nil keys entirely (encodeIfPresent).
        assertFalse(summary.toJson().contains("burnout_time_s"))
    }

    @Test
    fun `142 fallback gates max speed when both baro and velocity detectors agree`() {
        // 44-byte-era log: no master apogee flag — both per-detector flags
        // together must gate speed tracking (a single detector must not).
        val frames = ArrayList<ByteArray>()
        for (i in 1..3) frames.add(imuMini(i * 1_000_000L))
        frames.add(nonSensor(t = 1_000_000, flags = launchBit, uVel = 10_000))  // 100 m/s
        frames.add(
            nonSensor(
                t = 2_000_000,
                flags = launchBit or altApogeeBit or velApogeeBit,
                uVel = 30_000,  // 300 m/s — must be gated off
            ),
        )
        val (_, summary) = CsvGenerator().writeCsv(log(*frames.toTypedArray()))
        assertEquals(100.0, summary.maxSpeedMps)
        assertNull(summary.apogeeTimeS)  // no master flag → no apogee latch
    }

    // ------------------------------------------------------------------
    // #514 quat-blank rule
    // ------------------------------------------------------------------

    @Test
    fun `quaternion columns blank when squared norm is not above one half`() {
        // |q|² = 0.09 ≤ 0.5 → "absent" marker → four blank quat columns,
        // while the roll/pitch/yaw columns still emit.
        val bin = log(
            nonSensor(t = 1000, q0 = 3000, q1 = 0, q2 = 0, q3 = 0),
            imuMini(1000),
        )
        val (csv, _) = CsvGenerator().writeCsv(bin)
        val row = fields(dataRows(csv)[0])
        assertEquals(listOf("", "", "", ""), row.subList(29, 33))
        assertTrue(row[33].isNotEmpty(), "roll must still emit")
        assertTrue(row[34].isNotEmpty(), "pitch must still emit")
    }
}
