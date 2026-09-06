package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertIs
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * Telemetry-characteristic demux: the `"type"` ladder from iOS
 * BLEDevice.parseTelemetryData, the #253 config sentinel matrix, and the
 * config / config_pyro merge that keeps the two half-messages from
 * clobbering each other.
 */
class TelemetryDispatchTest {

    // ── ladder routing ────────────────────────────────────────────────────

    @Test
    fun `absent type decodes as plain telemetry`() {
        // Port of the PowerStateGate protocol surface: fs bit 0x10 is
        // pwr_pin_on and a first frame must decode (the session's
        // hasReceivedTelemetry gate hangs off exactly this succeeding).
        val msg = TelemetryDispatch.parse("""{"st":"READY","fs":16}""")
        val t = assertIs<TelemetryCharMessage.Telemetry>(msg)
        assertEquals("READY", t.data.state)
        assertTrue(t.data.pwrPinOn)
    }

    @Test
    fun `relayed frame decodes with source rocket id`() {
        val msg = TelemetryDispatch.parse("""{"rid":7,"run":"Atlas","st":"READY","fs":16}""")
        val t = assertIs<TelemetryCharMessage.Telemetry>(msg)
        assertEquals(7, t.data.sourceRocketId)
        assertEquals("Atlas", t.data.sourceUnitName)
    }

    @Test
    fun `unrecognized type falls through to telemetry decode - pinned iOS oddity`() {
        // iOS checks only the known type strings; anything else hits the
        // telemetry decoder, which ignores unknown keys — so this IS a
        // telemetry frame on iOS, not a dropped one.
        val msg = TelemetryDispatch.parse("""{"type":"bogus","st":"READY"}""")
        val t = assertIs<TelemetryCharMessage.Telemetry>(msg)
        assertEquals("READY", t.data.state)
    }

    @Test
    fun `non-string type falls through to telemetry decode`() {
        val msg = TelemetryDispatch.parse("""{"type":7,"st":"READY"}""")
        assertIs<TelemetryCharMessage.Telemetry>(msg)
    }

    @Test
    fun `garbage and non-object frames are Unknown`() {
        assertEquals(
            TelemetryCharMessage.Unknown(type = null),
            TelemetryDispatch.parse("not json"),
        )
        assertEquals(
            TelemetryCharMessage.Unknown(type = null),
            TelemetryDispatch.parse("[1,2,3]"),
        )
        // Strict-field mismatch fails the whole telemetry frame (see
        // TelemetryData) — the unrecognized type string is surfaced.
        assertEquals(
            TelemetryCharMessage.Unknown(type = "bogus"),
            TelemetryDispatch.parse("""{"type":"bogus","vol":"garbage"}"""),
        )
    }

    @Test
    fun `each readback type routes to its variant`() {
        assertIs<TelemetryCharMessage.Config>(TelemetryDispatch.parse("""{"type":"config"}"""))
        assertIs<TelemetryCharMessage.ConfigPyro>(
            TelemetryDispatch.parse("""{"type":"config_pyro"}"""),
        )
        assertIs<TelemetryCharMessage.ConfigIdentity>(
            TelemetryDispatch.parse("""{"type":"config_identity"}"""),
        )
        assertIs<TelemetryCharMessage.FcIdentity>(
            TelemetryDispatch.parse("""{"type":"fc_identity"}"""),
        )
        assertIs<TelemetryCharMessage.ImuOrient>(
            TelemetryDispatch.parse("""{"type":"imu_orient"}"""),
        )
        assertIs<TelemetryCharMessage.GuidTarget>(
            TelemetryDispatch.parse("""{"type":"guid_target","seq":1}"""),
        )
    }

    @Test
    fun `bytes entry point matches string entry point`() {
        val text = """{"type":"config_identity","uid":"a1b2c3d4"}"""
        assertEquals(TelemetryDispatch.parse(text), TelemetryDispatch.parse(text.encodeToByteArray()))
    }

    // ── config readback ───────────────────────────────────────────────────

    private fun config(json: String, previous: RocketConfig? = null): RocketConfig {
        val msg = assertIs<TelemetryCharMessage.Config>(TelemetryDispatch.parse(json))
        return msg.msg.applyTo(previous)
    }

    @Test
    fun `config full short-key set decodes field-for-field`() {
        val cfg = config(
            """
            {"type":"config","sb1":12,"shz":300,"smn":1100,"smx":1900,
             "kp":0.1,"ki":0.01,"kd":0.002,"pmn":-8.0,"pmx":8.0,
             "sen":false,"gs":false,"ac":true,"rdly":250,"rmspd":28.0,
             "rcap":45.0,"kpang":1.5,"iwind":30.0,"ge":true,"camt":1,
             "irate":1920,"lf":915.5,"lsf":8,"lbw":250.0,"lcr":5,"lpw":-3,
             "lhd":true,"lhdw":400,"ltxd":true}
            """.trimIndent(),
        )
        assertEquals(12, cfg.servoBias1)
        assertEquals(300, cfg.servoHz)
        assertEquals(1100, cfg.servoMinUs)
        assertEquals(1900, cfg.servoMaxUs)
        assertEquals(0.1f, cfg.pidKp)
        assertEquals(0.01f, cfg.pidKi)
        assertEquals(0.002f, cfg.pidKd)
        assertEquals(-8.0f, cfg.pidMinCmd)
        assertEquals(8.0f, cfg.pidMaxCmd)
        assertFalse(cfg.servoEnabled)
        assertFalse(cfg.gainScheduleEnabled)
        assertTrue(cfg.useAngleControl)
        assertEquals(250, cfg.rollDelayMs)
        assertEquals(28.0f, cfg.rollMinSpeedMps)
        assertEquals(45.0f, cfg.rateCapDps)
        assertEquals(1.5f, cfg.kpAngle)
        assertEquals(30.0f, cfg.integralSepThreshold)
        assertTrue(cfg.guidanceEnabled)
        assertEquals(1, cfg.cameraType)
        assertEquals(1920, cfg.imuRateHz)
        assertEquals(915.5f, cfg.loraFreqMHz)
        assertEquals(8, cfg.loraSF)
        assertEquals(250.0f, cfg.loraBwKHz)
        assertEquals(5, cfg.loraCR)
        assertEquals(-3, cfg.loraTxPower)
        assertEquals(true, cfg.loraHopDisabled)
        assertEquals(400, cfg.loraHopDwell)
        assertEquals(true, cfg.loraTxDisabled)
    }

    @Test
    fun `config ltxd absent stays null, never false`() {
        // The three states are distinct and the app leans on it: true =
        // muted, false = on the air, null = firmware predates "LoRa off".
        // Collapsing null to false would make the dashboard advisory and the
        // settings switch claim a radio state the rocket never reported.
        assertNull(config("""{"type":"config","sb1":1}""").loraTxDisabled)
        assertEquals(false, config("""{"type":"config","ltxd":false}""").loraTxDisabled)
        assertEquals(true, config("""{"type":"config","ltxd":true}""").loraTxDisabled)
    }

    @Test
    fun `config absent keys fall back to defaults not previous values`() {
        // iOS rebuilds from RocketConfig() — a previous non-pyro edit does
        // NOT survive a readback that omits the key.
        val previous = RocketConfig(servoHz = 100, rateCapDps = 99f)
        val cfg = config("""{"type":"config","sb1":5}""", previous)
        assertEquals(5, cfg.servoBias1)
        assertEquals(333, cfg.servoHz)      // default, not 100
        assertEquals(60f, cfg.rateCapDps)   // default, not 99
    }

    @Test
    fun `config sentinel matrix - rcap and kpang gate on greater than zero`() {
        // #253: <= 0 means "firmware default, keep local value".
        assertEquals(60f, config("""{"type":"config","rcap":0}""").rateCapDps)
        assertEquals(60f, config("""{"type":"config","rcap":-1.0}""").rateCapDps)
        assertEquals(0.5f, config("""{"type":"config","rcap":0.5}""").rateCapDps)
        assertEquals(45f, config("""{"type":"config","rcap":45.0}""").rateCapDps)
        assertEquals(2.0f, config("""{"type":"config","kpang":0}""").kpAngle)
        assertEquals(2.0f, config("""{"type":"config","kpang":-3.5}""").kpAngle)
        assertEquals(0.25f, config("""{"type":"config","kpang":0.25}""").kpAngle)
    }

    @Test
    fun `config sentinel matrix - iwind gates on greater or equal zero`() {
        // 0 is a VALID value for iwind (disables integral separation) —
        // only negatives are the sentinel.
        assertEquals(40f, config("""{"type":"config","iwind":-1}""").integralSepThreshold)
        assertEquals(0f, config("""{"type":"config","iwind":0}""").integralSepThreshold)
        assertEquals(25f, config("""{"type":"config","iwind":25.0}""").integralSepThreshold)
    }

    @Test
    fun `config irate lands only when it fits u16`() {
        assertEquals(3840, config("""{"type":"config","irate":3840}""").imuRateHz)
        assertNull(config("""{"type":"config","irate":70000}""").imuRateHz)
        assertNull(config("""{"type":"config","irate":-1}""").imuRateHz)
        assertNull(config("""{"type":"config"}""").imuRateHz)
    }

    @Test
    fun `config absent lora keys read as null`() {
        val cfg = config("""{"type":"config","sb1":1}""")
        assertNull(cfg.loraFreqMHz)
        assertNull(cfg.loraSF)
        assertNull(cfg.loraHopDisabled)   // #106: device doesn't report it
        assertNull(cfg.loraHopDwell)      // #150: pre-#150 firmware
    }

    @Test
    fun `config resets imuOrientSetting - it only arrives on imu_orient`() {
        val previous = RocketConfig(imuOrientSetting = 0xFF)
        assertNull(config("""{"type":"config"}""", previous).imuOrientSetting)
    }

    @Test
    fun `config preserves pyro fields from previous config`() {
        val previous = RocketConfig(
            pyro2Enabled = true,
            pyro2TriggerMode = 3,
            pyro2TriggerValue = 42.5f,
            pyro4Enabled = true,
        )
        val cfg = config("""{"type":"config","sb1":9}""", previous)
        assertEquals(9, cfg.servoBias1)
        assertTrue(cfg.pyro2Enabled)
        assertEquals(3, cfg.pyro2TriggerMode)
        assertEquals(42.5f, cfg.pyro2TriggerValue)
        assertTrue(cfg.pyro4Enabled)
        assertFalse(cfg.pyro1Enabled)
        assertEquals(1.0f, cfg.pyro1TriggerValue)   // channel-1 default
    }

    @Test
    fun `config int fields tolerate integral floats and reject fractional`() {
        // JSONSerialization `as? Int` bridging: 300.0 casts, 300.5 doesn't
        // (falls back to the default).
        assertEquals(300, config("""{"type":"config","shz":300.0}""").servoHz)
        assertEquals(333, config("""{"type":"config","shz":300.5}""").servoHz)
        // Strings never bridge to Int.
        assertEquals(333, config("""{"type":"config","shz":"300"}""").servoHz)
    }

    @Test
    fun `config float fields accept int and numeric string like iOS parseFloat`() {
        assertEquals(45f, config("""{"type":"config","rcap":45}""").rateCapDps)
        assertEquals(45.5f, config("""{"type":"config","rcap":"45.5"}""").rateCapDps)
        // Garbage string → nil → default.
        assertEquals(60f, config("""{"type":"config","rcap":"x"}""").rateCapDps)
    }

    // ── config_pyro readback ──────────────────────────────────────────────

    private fun pyro(json: String, previous: RocketConfig?): RocketConfig {
        val msg = assertIs<TelemetryCharMessage.ConfigPyro>(TelemetryDispatch.parse(json))
        return msg.msg.applyTo(previous)
    }

    @Test
    fun `config_pyro merge preserves the non-pyro side`() {
        val previous = RocketConfig(
            servoHz = 250,
            rateCapDps = 45f,
            loraFreqMHz = 915.5f,
            imuOrientSetting = 4,
        )
        val cfg = pyro(
            """{"type":"config_pyro","p1e":true,"p1m":1,"p1v":150.0,"p3e":true,"p3m":2,"p3v":0.5}""",
            previous,
        )
        // Non-pyro fields untouched.
        assertEquals(250, cfg.servoHz)
        assertEquals(45f, cfg.rateCapDps)
        assertEquals(915.5f, cfg.loraFreqMHz)
        assertEquals(4, cfg.imuOrientSetting)
        // Present pyro keys applied.
        assertTrue(cfg.pyro1Enabled)
        assertEquals(1, cfg.pyro1TriggerMode)
        assertEquals(150.0f, cfg.pyro1TriggerValue)
        assertTrue(cfg.pyro3Enabled)
        assertEquals(2, cfg.pyro3TriggerMode)
        assertEquals(0.5f, cfg.pyro3TriggerValue)
        // Absent pyro keys keep previous values (here: defaults).
        assertFalse(cfg.pyro2Enabled)
        assertEquals(100.0f, cfg.pyro2TriggerValue)
    }

    @Test
    fun `config_pyro without previous config starts from defaults`() {
        val cfg = pyro("""{"type":"config_pyro","p2e":true}""", previous = null)
        assertTrue(cfg.pyro2Enabled)
        assertEquals(333, cfg.servoHz)
        assertEquals(1.0f, cfg.pyro1TriggerValue)
    }

    @Test
    fun `config then config_pyro then config keeps both sides`() {
        // The real connect-time sequence: OC pushes config, then config_pyro,
        // then a later refresh pushes config again — pyro must survive it.
        var cfg: RocketConfig? = null
        cfg = config("""{"type":"config","shz":300}""", cfg)
        cfg = pyro("""{"type":"config_pyro","p1e":true,"p1v":123.0}""", cfg)
        cfg = config("""{"type":"config","shz":301}""", cfg)
        assertEquals(301, cfg.servoHz)
        assertTrue(cfg.pyro1Enabled)
        assertEquals(123.0f, cfg.pyro1TriggerValue)
    }

    // ── identity / orientation messages ───────────────────────────────────

    @Test
    fun `config_identity decodes all keys`() {
        val msg = assertIs<TelemetryCharMessage.ConfigIdentity>(
            TelemetryDispatch.parse(
                """{"type":"config_identity","uid":"a1b2c3d4","un":"Atlas","nid":42,"rid":3,
                   "dt":"R","fw":"abc1234+20260722-0930"}""",
            ),
        )
        assertEquals("a1b2c3d4", msg.msg.unitId)
        assertEquals("Atlas", msg.msg.unitName)
        assertEquals(42, msg.msg.networkId)
        assertEquals(3, msg.msg.rocketId)
        assertEquals("R", msg.msg.deviceType)
        assertEquals("abc1234+20260722-0930", msg.msg.firmwareVersion)
    }

    @Test
    fun `config_identity absent keys are null - leave previous state alone`() {
        val msg = assertIs<TelemetryCharMessage.ConfigIdentity>(
            TelemetryDispatch.parse("""{"type":"config_identity","un":"Atlas"}"""),
        )
        assertNull(msg.msg.unitId)
        assertEquals("Atlas", msg.msg.unitName)
        assertNull(msg.msg.networkId)
        assertNull(msg.msg.deviceType)
        assertNull(msg.msg.firmwareVersion)
    }

    @Test
    fun `fc_identity carries the FC's own version`() {
        val msg = assertIs<TelemetryCharMessage.FcIdentity>(
            TelemetryDispatch.parse("""{"type":"fc_identity","fc_fw":"def5678+20260721-1800"}"""),
        )
        assertEquals("def5678+20260721-1800", msg.msg.fcFirmwareVersion)
        assertNull(
            assertIs<TelemetryCharMessage.FcIdentity>(
                TelemetryDispatch.parse("""{"type":"fc_identity"}"""),
            ).msg.fcFirmwareVersion,
        )
    }

    @Test
    fun `imu_orient decodes name mode and setting`() {
        val msg = assertIs<TelemetryCharMessage.ImuOrient>(
            TelemetryDispatch.parse("""{"type":"imu_orient","name":"-Z r90","mode":3,"set":255}"""),
        )
        assertEquals("-Z r90", msg.msg.name)
        assertEquals(IMUOrientationMode.AUTO_EXACT, msg.msg.mode)
        assertEquals(0xFF, msg.msg.setting)
    }

    @Test
    fun `imu_orient unknown mode raw maps to UNKNOWN and absent mode to null`() {
        val unknown = assertIs<TelemetryCharMessage.ImuOrient>(
            TelemetryDispatch.parse("""{"type":"imu_orient","mode":9}"""),
        )
        assertEquals(IMUOrientationMode.UNKNOWN, unknown.msg.mode)
        val absent = assertIs<TelemetryCharMessage.ImuOrient>(
            TelemetryDispatch.parse("""{"type":"imu_orient","name":"+X"}"""),
        )
        assertNull(absent.msg.mode)
        assertNull(absent.msg.setting)
    }
}
