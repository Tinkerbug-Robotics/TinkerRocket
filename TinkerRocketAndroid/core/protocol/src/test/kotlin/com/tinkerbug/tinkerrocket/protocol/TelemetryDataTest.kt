package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

private typealias SH = TelemetryData.SensorHealth

/**
 * Port of iOS TelemetryDataTests.swift — identical inputs and expected
 * values (never weaken an iOS pin), plus explicit pins on the ASYMMETRIC
 * decode leniency in both directions (#293/#571): tolerant integer keys must
 * degrade one field, strict float/string keys must fail the whole frame.
 *
 * No golden fixtures exist for telemetry (the corpus is binary wire frames;
 * telemetry is BLE JSON), so these are hand-written pins from the iOS suite.
 */
class TelemetryDataTest {

    private fun decodeOk(json: String): TelemetryData =
        assertNotNull(TelemetryData.decode(json), "frame must decode: $json")

    // ── JSON decoding ─────────────────────────────────────────────────────

    @Test
    fun `nominal payload decodes through the short keys`() {
        val t = decodeOk(
            """
            {
                "soc": 85.0,
                "vol": 3.85,
                "cur": 450.0,
                "lat": 33.7,
                "lon": -118.4,
                "st": "INFLIGHT",
                "nsat": 12,
                "gdop": 1.5,
                "q0": 0.707,
                "q1": 0.0,
                "q2": 0.0,
                "q3": 0.707
            }
            """.trimIndent()
        )
        assertEquals(85.0f, assertNotNull(t.soc), 0.1f)
        assertEquals(3.85f, assertNotNull(t.voltage), 0.01f)
        assertEquals(33.7, assertNotNull(t.latitude), 1e-6)
        assertEquals("INFLIGHT", t.state)
        assertEquals(12, t.numSats)
        // gdop is vestigial on iOS (absent from CodingKeys, never decoded) —
        // pin that the decoder leaves it null even when the key is present.
        assertNull(t.gdop)
    }

    @Test
    fun `integer fields tolerate float and string`() {
        // #571: completes the #293 hardening — EVERY integer key must survive
        // firmware-contract drift emitting it as a float or string; one
        // off-type field (e.g. "rid" on a relayed frame) must degrade a single
        // field, not make the rocket vanish from the BS dashboard.
        val t = decodeOk(
            """{"st":"INFLIGHT","nsat":7.0,"rid":"3","frx":"1000","fdr":2.0,
                "hch":"5","nidd":4.0,"h":"9","slrm":"120","ds":1.0}"""
        )
        assertEquals(7, t.numSats)
        assertEquals(3, t.sourceRocketId, "rid is the multi-rocket demux key")
        assertEquals(1000L, t.framesRx)
        assertEquals(2L, t.framesDrop)
        assertEquals(5, t.hopChannel)
        assertEquals(4, t.netidDrops)
        assertEquals(9, t.sensorHealth)
        assertEquals(120, t.bsLogSilenceRemainingS)
        assertEquals(TelemetryData.DataStatus.STALE, t.dataStatus)
        assertEquals("INFLIGHT", t.state, "frame must decode as a whole")
    }

    @Test
    fun `flexInt truncates floats toward zero like Swift Int(d)`() {
        assertEquals(-2, decodeOk("""{"nsat":-2.7}""").numSats)
        assertEquals(3, decodeOk("""{"nsat":"3.9"}""").numSats)
        assertEquals(0, decodeOk("""{"nsat":0.5}""").numSats)
    }

    @Test
    fun `garbage in a tolerant integer key degrades the field not the frame`() {
        // Swift flexInt: bool/array/non-numeric string all fail every branch →
        // nil → default, and the frame survives.
        val t = decodeOk("""{"st":"INFLIGHT","fs":true,"nsat":"twelve","hch":[1]}""")
        assertEquals("INFLIGHT", t.state)
        assertEquals(0, t.flightStatusBits)
        assertEquals(0, t.numSats)
        assertNull(t.hopChannel)
    }

    // ── Strictness (both directions — do NOT weaken either) ───────────────

    @Test
    fun `float key holding a string fails the whole frame`() {
        // iOS decodeIfPresent(Float) throws on a string and the caller's
        // catch discards the frame — the Kotlin port must be exactly as
        // strict, not more lenient (#293/#571 shipped bugs both directions).
        assertNull(TelemetryData.decode("""{"vol":"3.85"}"""))
        assertNull(TelemetryData.decode("""{"st":"INFLIGHT","q0":"1.0"}"""))
    }

    @Test
    fun `double key holding a string fails the whole frame`() {
        assertNull(TelemetryData.decode("""{"lat":"33.7","lon":-118.4}"""))
    }

    @Test
    fun `string key holding a number fails the whole frame`() {
        assertNull(TelemetryData.decode("""{"st":123}"""))
        assertNull(TelemetryData.decode("""{"run":42}"""))
    }

    @Test
    fun `int key holding a numeric string still decodes`() {
        val t = decodeOk("""{"nsat":"12","vol":3.85}""")
        assertEquals(12, t.numSats)
        assertEquals(3.85f, assertNotNull(t.voltage), 0.01f)
    }

    @Test
    fun `malformed json returns null`() {
        assertNull(TelemetryData.decode("{not json"))
        assertNull(TelemetryData.decode("[1,2,3]"))   // non-object top level
    }

    // ── Hop-state keys (#150) ─────────────────────────────────────────────

    @Test
    fun `hop state keys decode when present and stay null when absent`() {
        val t1 = decodeOk("""{"st":"PRELAUNCH","hch":42,"nidd":7}""")
        assertEquals(42, t1.hopChannel)
        assertEquals(7, t1.netidDrops)

        val t2 = decodeOk("""{"st":"PRELAUNCH"}""")
        assertNull(t2.hopChannel, "fixed-mode frames omit hch entirely")
        assertNull(t2.netidDrops, "healthy-nid frames omit nidd entirely")
    }

    // ── Missing keys → defaults ───────────────────────────────────────────

    @Test
    fun `empty object decodes with defaults`() {
        val t = decodeOk("{}")
        assertNull(t.soc)
        assertNull(t.latitude)
        assertEquals("UNKNOWN", t.state)
        assertEquals(0, t.numSats)
        assertEquals("", t.activeFile)
        assertEquals(0L, t.framesRx)
        assertEquals(TelemetryData.DataStatus.LIVE, t.dataStatus)
        assertEquals(0L, t.dataAgeMs)
        assertFalse(t.fieldsTrimmed)
    }

    @Test
    fun `unknown ds raw value reads as LIVE`() {
        // Swift DataStatus(rawValue: 9) is nil → .live
        assertEquals(TelemetryData.DataStatus.LIVE, decodeOk("""{"ds":9}""").dataStatus)
        assertEquals(TelemetryData.DataStatus.SYNCING, decodeOk("""{"ds":2}""").dataStatus)
    }

    @Test
    fun `u32 and u16 conversions clamp like Swift clamping inits`() {
        // UInt32(clamping:) and UInt16(clamping:) — negative → 0, over → max.
        val t = decodeOk("""{"frx":-5,"fdr":5000000000,"age":-1,"slrm":70000}""")
        assertEquals(0L, t.framesRx)
        assertEquals(4294967295L, t.framesDrop)
        assertEquals(0L, t.dataAgeMs)
        assertEquals(65535, t.bsLogSilenceRemainingS)
    }

    // ── Flight status bitfield (#138 MTU pack) ────────────────────────────
    // b0=lnch b1=vapo b2=aapo b3=land b4=pwr b5=cam b6=log b7=bslog
    // b8=sim (#393) b9=burnout (#191)

    @Test
    fun `flight status bits all resolve`() {
        val allSet = decodeOk("""{"fs":255}""")
        assertTrue(allSet.launchFlag)
        assertTrue(allSet.velApo)
        assertTrue(allSet.altApo)
        assertTrue(allSet.landedFlag)
        assertTrue(allSet.pwrPinOn)
        assertTrue(allSet.cameraRecording)
        assertTrue(allSet.loggingActive)
        assertTrue(allSet.bsLoggingActive)

        // 0x55 = 0b0101_0101 — even bits: lnch, aapo, pwr, log.  Verifies each
        // property reads its assigned bit (not an off-by-one shift).
        val evenSet = decodeOk("""{"fs":85}""")
        assertTrue(evenSet.launchFlag)
        assertFalse(evenSet.velApo)
        assertTrue(evenSet.altApo)
        assertFalse(evenSet.landedFlag)
        assertTrue(evenSet.pwrPinOn)
        assertFalse(evenSet.cameraRecording)
        assertTrue(evenSet.loggingActive)
        assertFalse(evenSet.bsLoggingActive)
    }

    @Test
    fun `fs bits 8 and 9 are sim and burnout`() {
        val both = decodeOk("""{"fs":768}""")   // 0x300
        assertTrue(both.simActive)
        assertTrue(both.burnoutFlag)
        val simOnly = decodeOk("""{"fs":256}""")
        assertTrue(simOnly.simActive)
        assertFalse(simOnly.burnoutFlag)
        // 255 sets b0..b7 only — neither high bit.
        val low = decodeOk("""{"fs":255}""")
        assertFalse(low.simActive)
        assertFalse(low.burnoutFlag)
    }

    @Test
    fun `missing fs key means all flags false`() {
        val t = decodeOk("{}")
        assertEquals(0, t.flightStatusBits)
        assertFalse(t.launchFlag)
        assertFalse(t.altApo)
        assertFalse(t.landedFlag)
        assertFalse(t.bsLoggingActive)
    }

    // ── Pyro status bitfield ("ps") ───────────────────────────────────────
    // b0 = global armed, then (cont, fired) pairs for channels 1..4.

    @Test
    fun `pyro status bits resolve per channel`() {
        // armed + ch1 cont + ch3 fired: 0x001 | 0x002 | 0x040 = 0x043
        val t = decodeOk("""{"ps":67}""")
        assertTrue(t.pyroArmed)
        assertTrue(t.pyroCont(1))
        assertFalse(t.pyroFired(1))
        assertFalse(t.pyroCont(3))
        assertTrue(t.pyroFired(3))
        assertFalse(t.pyroCont(2))
        assertFalse(t.pyroFired(4))
        // Out-of-range channels read false, matching the Swift default case.
        assertFalse(t.pyroCont(0))
        assertFalse(t.pyroFired(5))

        val all = decodeOk("""{"ps":511}""")   // 9 bits set
        assertTrue(all.pyroArmed)
        for (ch in 1..4) {
            assertTrue(all.pyroCont(ch))
            assertTrue(all.pyroFired(ch))
        }
    }

    // ── Quaternion-derived attitude ───────────────────────────────────────

    @Test
    fun `identity quaternion yields zero euler angles`() {
        // The roll/pitch/yaw properties reject near-zero norms, so use a valid
        // unit quaternion, not defaults.
        val t = TelemetryData(q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f)
        assertEquals(1.0f, assertNotNull(t.q0), 1e-6f)
        assertEquals(0.0f, assertNotNull(t.roll), 1e-3f)
        assertEquals(0.0f, assertNotNull(t.pitch), 1e-3f)
        assertEquals(0.0f, assertNotNull(t.yaw), 1e-3f)
    }

    @Test
    fun `malformed quaternion yields null attitude`() {
        // All-zeros (uninitialized EKF) fails the norm window.
        val zeros = TelemetryData(q0 = 0.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f)
        assertNull(zeros.roll)
        assertNull(zeros.pitch)
        assertNull(zeros.yaw)
        // Missing components → null.
        assertNull(TelemetryData(q0 = 1.0f).roll)
    }

    // ── Sensor health scorecard (#303) ────────────────────────────────────

    /** Pack an "h" bitfield.  Shifts mirror RocketComputerTypes.h SH_*_SHIFT
     *  (baro 0, imu 2, ekf 4, mag 6, gnss 8, batt 10, pyro1..4 at 12/14/16/18,
     *  storage 20). */
    private fun pack(
        baro: SH = SH.NA, imu: SH = SH.NA, ekf: SH = SH.NA, mag: SH = SH.NA,
        gnss: SH = SH.NA, batt: SH = SH.NA, storage: SH = SH.NA,
        p1: SH = SH.NA, p2: SH = SH.NA, p3: SH = SH.NA, p4: SH = SH.NA,
    ): Int =
        baro.raw or (imu.raw shl 2) or (ekf.raw shl 4) or (mag.raw shl 6) or
            (gnss.raw shl 8) or (batt.raw shl 10) or (storage.raw shl 20) or
            (p1.raw shl 12) or (p2.raw shl 14) or (p3.raw shl 16) or (p4.raw shl 18)

    private fun decodeHealth(health: Int): TelemetryData = decodeOk("""{"h":$health}""")

    @Test
    fun `sensor health raw bit positions`() {
        // Pinned as raw ints, independent of pack(), so a systematic shift
        // error can't hide behind a matching bug in the test helper.
        assertEquals(SH.OK, decodeHealth(1 shl 0).baroHealth)
        assertEquals(SH.OK, decodeHealth(1 shl 2).imuHealth)
        assertEquals(SH.OK, decodeHealth(1 shl 4).ekfHealth)
        assertEquals(SH.OK, decodeHealth(1 shl 6).magHealth)
        assertEquals(SH.OK, decodeHealth(1 shl 8).gnssHealth)
        assertEquals(SH.OK, decodeHealth(1 shl 10).battHealth)
        assertEquals(SH.OK, decodeHealth(1 shl 18).pyroHealth(4))
        assertEquals(SH.OK, decodeHealth(1 shl 20).storageHealth)   // #281/#278
        // A single field set leaves everything else N/A (no bleed).
        val onlyGnss = decodeHealth(1 shl 8)
        assertEquals(SH.NA, onlyGnss.baroHealth)
        assertEquals(SH.NA, onlyGnss.pyroHealth(1))
        // BAD encodes as 0b11 in-place (verifies the 2-bit mask).
        assertEquals(SH.BAD, decodeHealth(3 shl 4).ekfHealth)
        // #557: GNSS-absent degraded-flight flag at shift 22 (BAD = active),
        // distinct from the gnss fix-health item at shift 8.
        assertTrue(decodeHealth(3 shl 22).gnssAbsentMode)
        assertFalse(decodeHealth(1 shl 8).gnssAbsentMode)   // fix health ≠ absent mode
        assertFalse(decodeHealth(0).gnssAbsentMode)
    }

    @Test
    fun `missing h key means no scorecard and unknown readiness`() {
        val t = decodeOk("{}")
        assertEquals(0, t.sensorHealth)
        assertFalse(t.hasSensorHealth)
        assertEquals(TelemetryData.FlightReadiness.UNKNOWN, t.flightReadiness)
    }

    @Test
    fun `readiness all OK is ready`() {
        val t = decodeHealth(pack(baro = SH.OK, imu = SH.OK, ekf = SH.OK, mag = SH.OK, gnss = SH.OK, batt = SH.OK))
        assertTrue(t.hasSensorHealth)
        assertEquals(TelemetryData.FlightReadiness.READY, t.flightReadiness)
        assertEquals(6, t.sensorHealthRows.size)               // 6 core, no pyros
    }

    @Test
    fun `readiness hard faults are notReady`() {
        val baro = decodeHealth(pack(baro = SH.BAD, imu = SH.OK, ekf = SH.OK, gnss = SH.OK, batt = SH.OK))
        assertEquals(TelemetryData.FlightReadiness.NOT_READY, baro.flightReadiness)
        val batt = decodeHealth(pack(baro = SH.OK, imu = SH.OK, ekf = SH.OK, gnss = SH.OK, batt = SH.BAD))
        assertEquals(TelemetryData.FlightReadiness.NOT_READY, batt.flightReadiness)
        // A configured pyro with no continuity is a hard fault.
        val pyro = decodeHealth(pack(baro = SH.OK, imu = SH.OK, ekf = SH.OK, gnss = SH.OK, batt = SH.OK, p1 = SH.BAD))
        assertEquals(SH.BAD, pyro.pyroHealth(1))
        assertEquals(TelemetryData.FlightReadiness.NOT_READY, pyro.flightReadiness)
    }

    @Test
    fun `readiness ekf or gnss not OK is caution`() {
        // EKF not converged or GNSS without a fix gate green down to amber,
        // but aren't hard faults (you wait them out).
        val ekf = decodeHealth(pack(baro = SH.OK, imu = SH.OK, ekf = SH.DEGRADED, gnss = SH.OK, batt = SH.OK))
        assertEquals(TelemetryData.FlightReadiness.CAUTION, ekf.flightReadiness)
        val gnss = decodeHealth(pack(baro = SH.OK, imu = SH.OK, ekf = SH.OK, gnss = SH.BAD, batt = SH.OK))
        assertEquals(TelemetryData.FlightReadiness.CAUTION, gnss.flightReadiness)   // no fix → amber, not red
        // A configured-but-untested pyro is amber too.
        val pyro = decodeHealth(pack(baro = SH.OK, imu = SH.OK, ekf = SH.OK, gnss = SH.OK, batt = SH.OK, p1 = SH.DEGRADED))
        assertEquals(TelemetryData.FlightReadiness.CAUTION, pyro.flightReadiness)
    }

    @Test
    fun `readiness storage gates and NA is ignored`() {
        // #281/#278: BAD (full/failing NAND) is a hard fault; DEGRADED (low
        // space) gates green down to amber; N/A (older firmware) is ignored.
        val full = decodeHealth(pack(baro = SH.OK, imu = SH.OK, ekf = SH.OK, gnss = SH.OK, batt = SH.OK, storage = SH.BAD))
        assertEquals(SH.BAD, full.storageHealth)
        assertEquals(TelemetryData.FlightReadiness.NOT_READY, full.flightReadiness)
        val low = decodeHealth(pack(baro = SH.OK, imu = SH.OK, ekf = SH.OK, gnss = SH.OK, batt = SH.OK, storage = SH.DEGRADED))
        assertEquals(TelemetryData.FlightReadiness.CAUTION, low.flightReadiness)
        val ok = decodeHealth(pack(baro = SH.OK, imu = SH.OK, ekf = SH.OK, mag = SH.OK, gnss = SH.OK, batt = SH.OK, storage = SH.OK))
        assertEquals(TelemetryData.FlightReadiness.READY, ok.flightReadiness)
        assertEquals(7, ok.sensorHealthRows.size)              // 6 core + Storage
        assertTrue(ok.sensorHealthRows.any { it.name == "Storage" })
        // N/A storage (older firmware / BS frame) is ignored: no row, still green.
        val naStor = decodeHealth(pack(baro = SH.OK, imu = SH.OK, ekf = SH.OK, mag = SH.OK, gnss = SH.OK, batt = SH.OK))
        assertEquals(SH.NA, naStor.storageHealth)
        assertEquals(TelemetryData.FlightReadiness.READY, naStor.flightReadiness)
        assertFalse(naStor.sensorHealthRows.any { it.name == "Storage" })
    }

    @Test
    fun `readiness mag is advisory and does not gate`() {
        val t = decodeHealth(pack(baro = SH.OK, imu = SH.OK, ekf = SH.OK, mag = SH.BAD, gnss = SH.OK, batt = SH.OK))
        assertEquals(SH.BAD, t.magHealth)
        assertEquals(TelemetryData.FlightReadiness.READY, t.flightReadiness)
    }

    @Test
    fun `health rows include only configured pyros`() {
        val none = decodeHealth(pack(baro = SH.OK, imu = SH.OK))
        assertFalse(none.sensorHealthRows.any { it.name.startsWith("Pyro") })
        val two = decodeHealth(pack(baro = SH.OK, imu = SH.OK, p1 = SH.OK, p3 = SH.BAD))
        assertEquals(6 + 2, two.sensorHealthRows.size)
        assertTrue(two.sensorHealthRows.any { it.name == "Pyro 1" })
        assertTrue(two.sensorHealthRows.any { it.name == "Pyro 3" })
        assertFalse(two.sensorHealthRows.any { it.name == "Pyro 2" })
    }

    // ── MTU trim flag (#282) ──────────────────────────────────────────────

    @Test
    fun `trim flag decodes and defaults false`() {
        val plain = decodeOk("""{"st":"INFLIGHT"}""")
        assertFalse(plain.fieldsTrimmed)

        val trimmed = decodeOk("""{"st":"INFLIGHT","fs":7,"tr":1}""")
        assertTrue(trimmed.fieldsTrimmed)
        assertEquals("INFLIGHT", trimmed.state)
        assertTrue(trimmed.launchFlag)   // tail trim must not corrupt earlier fields
    }

    // ── Logging indicator (#137) ──────────────────────────────────────────

    @Test
    fun `rocket logging active suppressed only in LANDED`() {
        // fs bit 6 = log.
        val landed = decodeOk("""{"st":"LANDED","fs":64}""")
        assertTrue(landed.loggingActive)
        assertFalse(landed.rocketLoggingActive, "post-flight drain window must be suppressed")
        // Trust the flag in every other state — bench logging from READY is real.
        val ready = decodeOk("""{"st":"READY","fs":64}""")
        assertTrue(ready.rocketLoggingActive)
        val off = decodeOk("""{"st":"READY","fs":0}""")
        assertFalse(off.rocketLoggingActive)
    }

    // ── Relayed IMU orientation (#390: "imo" = (mode << 5) | code) ────────

    private fun decodeIMO(imo: Int?): TelemetryData {
        val body = imo?.let { """"imo": $it,""" } ?: ""
        return decodeOk("""{ $body "st": "PRELAUNCH" }""")
    }

    @Test
    fun `relayed orientation auto snap code`() {
        // mode 3 (auto), code 21 = -Z r90: (3 << 5) | 21 = 117
        val t = decodeIMO(117)
        assertEquals(IMUOrientationMode.AUTO_SNAP, t.relayedOrientationMode)
        assertEquals("-Z r90", t.relayedOrientationName)
    }

    @Test
    fun `relayed orientation manual and default`() {
        val manual = decodeIMO((2 shl 5) or 0)     // manual, +X
        assertEquals(IMUOrientationMode.MANUAL, manual.relayedOrientationMode)
        assertEquals("+X", manual.relayedOrientationName)

        val def = decodeIMO((1 shl 5) or 0)        // default mounting
        assertEquals(IMUOrientationMode.DEFAULT_MOUNTING, def.relayedOrientationMode)
        assertEquals("+X", def.relayedOrientationName)
    }

    @Test
    fun `relayed orientation auto exact sentinel`() {
        // Code 31 = auto-exact, no discrete code — must still produce a
        // non-empty name (the IMU card's line gates on it), not "?" or "".
        val t = decodeIMO((3 shl 5) or 31)
        assertEquals(IMUOrientationMode.AUTO_EXACT, t.relayedOrientationMode)
        assertEquals("custom", t.relayedOrientationName)
    }

    @Test
    fun `relayed orientation absent or legacy is unknown`() {
        // Missing key (pre-#390 firmware / trimmed tail) → unknown, empty
        // name → the BS view renders no orientation line at all.
        val absent = decodeIMO(null)
        assertEquals(IMUOrientationMode.UNKNOWN, absent.relayedOrientationMode)
        assertEquals("", absent.relayedOrientationName)

        // Wire mode 0 should never be emitted, but a zero must not render as
        // a confident "+X default".
        val zero = decodeIMO(0)
        assertEquals(IMUOrientationMode.UNKNOWN, zero.relayedOrientationMode)
        assertEquals("", zero.relayedOrientationName)
    }

    @Test
    fun `soc display never shows a negative percent`() {
        // A rocket on USB sits below the 2S curve, so the OC clamps SOC to 0
        // before packing.  The i16 spans -25..125% for headroom, so an exact
        // 0% comes back as -0.00077 and printed as "-0.0%" on the dashboard.
        // The one that actually shipped: the OC prints SOC to one decimal,
        // so an exact 0% arrives as the literal -0.0, which is == 0.0 and so
        // survives any clamp untouched.
        assertEquals("0.0%", TelemetryData(soc = -0.0f).socDisplay)
        assertEquals("0.0%", TelemetryData(soc = -0.00077f).socDisplay)
        assertEquals("0.0%", TelemetryData(soc = -25f).socDisplay)
        assertEquals("100.0%", TelemetryData(soc = 125f).socDisplay)
        // In-range values are untouched, and absent stays absent.
        assertEquals("42.5%", TelemetryData(soc = 42.5f).socDisplay)
        assertEquals("\u2014", TelemetryData(soc = null).socDisplay)
    }

    // ── FC boot progress ("bs"/"bt"/"bd") ─────────────────────────────────
    // Twin of iOS FcBootProgressTests — same inputs, same rendered strings.

    @Test
    fun `boot keys decode together`() {
        val t = decodeOk("""{"st":"INITIALIZATION","bs":2,"bt":4200,"bd":2}""")
        assertTrue(t.fcBooting)
        assertEquals(2, t.fcBootStepRaw)
        assertEquals(FcBootStep.SENSORS, t.fcBootStep)
        assertEquals(4200, t.fcBootElapsedMs)
        assertEquals(FCB_DEG_GNSS, t.fcBootDegraded)
        assertEquals(listOf("GNSS"), t.fcBootDegradedSubsystems)
    }

    @Test
    fun `absent bd is the normal boot`() {
        val t = decodeOk("""{"st":"INITIALIZATION","bs":4,"bt":9000}""")
        assertEquals(0, t.fcBootDegraded)
        assertEquals(emptyList(), t.fcBootDegradedSubsystems)
        assertEquals("Starting servos", t.fcBootStatusLine())
    }

    @Test
    fun `all boot keys absent is the running FC`() {
        // Every frame from a running FC.  No progress to show, no error.
        val t = decodeOk("""{"st":"PRELAUNCH","nsat":9}""")
        assertFalse(t.fcBooting)
        assertNull(t.fcBootStepRaw)
        assertNull(t.fcBootElapsedMs)
        assertEquals(0, t.fcBootDegraded)
        assertNull(t.fcBootStepLabel)
        assertNull(t.fcBootStatusLine(), "no \"bs\" = the FC is past boot (or silent)")
        assertEquals("PRELAUNCH", t.state, "frame must decode as a whole")
    }

    @Test
    fun `boot integer keys tolerate float and string`() {
        // Same lenient-optional convention as hch/nidd (#571): one off-type
        // key must degrade that key, never throw away the whole frame.
        val t = decodeOk("""{"st":"INITIALIZATION","bs":"3","bt":7000.0,"bd":"4"}""")
        assertEquals(FcBootStep.GNSS, t.fcBootStep)
        assertEquals(7000, t.fcBootElapsedMs)
        assertEquals(FCB_DEG_SERVOS, t.fcBootDegraded)
        assertEquals("INITIALIZATION", t.state)
    }

    @Test
    fun `every step has its label`() {
        val expected = mapOf(
            FcBootStep.LINKS to "Linking to flight computer",
            FcBootStep.NVS to "Loading saved settings",
            FcBootStep.SENSORS to "Starting sensors",
            FcBootStep.GNSS to "Starting GNSS",
            FcBootStep.SERVOS to "Starting servos",
            FcBootStep.COMPLETE to "Flight computer ready",
        )
        for ((step, label) in expected) {
            assertEquals(label, step.label)
            assertEquals(label, decodeOk("""{"bs":${step.raw}}""").fcBootStatusLine())
        }
        assertEquals(expected.size, FcBootStep.entries.size)
        assertEquals(expected.size, FcBootStep.STEP_COUNT, "FCB_STEP_COUNT")
    }

    @Test
    fun `unknown step renders generically`() {
        // FcBootStep is append-only: a newer FC will report steps this build
        // has never heard of, and the line must still say something useful.
        val t = decodeOk("""{"st":"INITIALIZATION","bs":99,"bt":1200}""")
        assertTrue(t.fcBooting)
        assertNull(t.fcBootStep)
        assertEquals(99, t.fcBootStepRaw, "the raw wire value is kept")
        assertEquals(FC_BOOT_UNKNOWN_LABEL, t.fcBootStatusLine())
        assertEquals("Starting up\u2026", t.fcBootStatusLine())
    }

    @Test
    fun `degraded bits name their subsystem`() {
        fun names(bits: Int): List<String> =
            decodeOk("""{"bs":5,"bd":$bits}""").fcBootDegradedSubsystems
        assertEquals(listOf("sensors"), names(0x01))
        assertEquals(listOf("GNSS"), names(0x02))
        assertEquals(listOf("servos"), names(0x04))
        assertEquals(listOf("saved settings"), names(0x08))
        assertEquals(listOf("sensors", "GNSS", "servos", "saved settings"), names(0x0F))
    }

    @Test
    fun `degraded is surfaced on the line`() {
        // Boot CONTINUES past a degraded step, so the line is the only place
        // the operator learns the receiver never came up.
        val t = decodeOk("""{"bs":5,"bt":9000,"bd":2}""")
        assertEquals("Flight computer ready — degraded: GNSS", t.fcBootStatusLine())
    }

    @Test
    fun `a fresh step is not stalled`() {
        val t = decodeOk("""{"bs":2,"bt":2000}""")
        assertFalse(t.fcBootStalled(dwellS = 3))
        assertEquals("Starting sensors", t.fcBootStatusLine(dwellS = 3))
    }

    @Test
    fun `a stalled sensor step blames GNSS`() {
        // sensor_collector.begin() blocks to its ~35 s deadline when the
        // module is dead, so a boot parked on SENSORS IS the dead-GNSS stall.
        val t = decodeOk("""{"bs":2,"bt":2000}""")
        assertTrue(t.fcBootStalled(dwellS = 50))
        assertEquals(
            "Starting sensors — no progress for 50s, GNSS may be dead",
            t.fcBootStatusLine(dwellS = 50),
        )
    }

    @Test
    fun `a stall on another step says so without blame`() {
        assertEquals(
            "Starting servos — no progress for 20s",
            decodeOk("""{"bs":4,"bt":5000}""").fcBootStatusLine(dwellS = 20),
        )
        // Stall and degraded ride the same line, stall first.
        assertEquals(
            "Starting servos — no progress for 20s — degraded: GNSS",
            decodeOk("""{"bs":4,"bt":5000,"bd":2}""").fcBootStatusLine(dwellS = 20),
        )
    }

    /**
     * Regression, measured on the bench 2026-08-19: a HEALTHY board sits on
     * SENSORS for ~21 s, because GNSS bootstrap runs inside
     * sensor_collector.begin() (it walks 460800 -> swapped RX/TX -> 9600 ->
     * 38400 before settling, then reads OTP).  This shipped with a single 12 s
     * threshold and so cried "GNSS may be dead" on every normal boot -- the
     * fastest way to teach an operator to ignore the warning.  Pin the real
     * measured duration as explicitly NOT a stall.
     */
    @Test
    fun `the measured healthy sensor duration is not a stall`() {
        val t = decodeOk("""{"bs":2,"bt":199}""")
        assertFalse(t.fcBootStalled(dwellS = 21))
        assertEquals("Starting sensors", t.fcBootStatusLine(dwellS = 21))
        // The servo step's own measured 4.2 s must not trip its budget either.
        val servos = decodeOk("""{"bs":4,"bt":21156}""")
        assertFalse(servos.fcBootStalled(dwellS = 5))
    }

    @Test
    fun `the complete step never stalls`() {
        // FCB_COMPLETE means setup_fc() finished; the app is only waiting for
        // the first real NonSensorData, which is not the FC's fault.
        val t = decodeOk("""{"bs":5,"bt":10000}""")
        assertFalse(t.fcBootStalled(dwellS = 60))
        assertEquals("Flight computer ready", t.fcBootStatusLine(dwellS = 60))
    }

    @Test
    fun `no boot keys while INITIALIZATION reads as waiting`() {
        // The whole point of the line: a zeroed rocket_state must not pass for
        // a live flight computer that is "initializing".
        assertEquals(
            "Waiting for flight computer\u2026",
            decodeOk("""{"st":"INITIALIZATION"}""").fcBootStatusLine(),
        )
        assertEquals(
            FC_BOOT_SILENT_LABEL,
            decodeOk("""{"st":"INITIALIZATION"}""").fcBootStatusLine(dwellS = 30),
        )
    }

    @Test
    fun `running states get no secondary line`() {
        for (st in listOf("READY", "PRELAUNCH", "INFLIGHT", "COMPLETE", "LANDED")) {
            assertNull(
                decodeOk("""{"st":"$st"}""").fcBootStatusLine(),
                "$st is a live state — nothing to qualify",
            )
        }
        assertNull(decodeOk("{}").fcBootStatusLine())
    }

    @Test
    fun `base station soc display follows the same rules as the rocket's`() {
        // The BS pack empties the same way and would print the same "-0.0%".
        assertEquals("0.0%", TelemetryData(bsSoc = -0.0f).bsSocDisplay)
        assertEquals("100.0%", TelemetryData(bsSoc = 125f).bsSocDisplay)
        assertEquals("98.2%", TelemetryData(bsSoc = 98.2f).bsSocDisplay)
        // Absent on a direct rocket link, where the row is not rendered at all.
        assertEquals("\u2014", TelemetryData(bsSoc = null).bsSocDisplay)
    }
}
