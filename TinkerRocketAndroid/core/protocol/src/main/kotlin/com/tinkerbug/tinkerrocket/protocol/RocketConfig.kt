package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.JsonObject

/**
 * Device configuration mirror — a field-for-field port of iOS
 * `RocketConfig` (BLETypes.swift), populated from the `"type":"config"` and
 * `"type":"config_pyro"` JSON readbacks on the telemetry characteristic.
 *
 * Defaults are the iOS defaults, which in turn track RocketProfile/config.h
 * (#561 servoBias1 = 0, #407 servoMinUs/MaxUs = 1000/2000).  Original wire
 * widths are noted per field; Kotlin stores plain Int/Float/Boolean.
 *
 * Population semantics (see [ConfigMessage] / [ConfigPyroMessage]):
 *  - a `config` message REBUILDS the whole struct from defaults + the message
 *    (absent key → default, NOT the previous value) and carries NO pyro
 *    fields — those are copied over from the previous config so the two
 *    messages merge without clobbering each other;
 *  - a `config_pyro` message edits ONLY the pyro fields of the previous
 *    config (or of a fresh default when none exists yet).
 */
public data class RocketConfig(
    val servoBias1: Int = 0,                    // i16, "sb1" (#561: match RocketProfile/config.h)
    val servoHz: Int = 333,                     // i16, "shz"
    // #407: match RocketProfile (source of truth) and config.h
    // SERVO_MIN_US/MAX_US = 1000/2000.
    val servoMinUs: Int = 1000,                 // i16, "smn"
    val servoMaxUs: Int = 2000,                 // i16, "smx"
    // #915: match RocketProfile and config.h KP/KI/KD/MIN_CMD/MAX_CMD, the
    // same alignment #407 and #561 made for the servo fields.  These defaults
    // survive into the profile whenever a readback omits the key, so a stale
    // set here is a silent re-tune.
    val pidKp: Float = 0.12f,                   // "kp"
    val pidKi: Float = 0.01f,                   // "ki"
    val pidKd: Float = 0.0f,                    // "kd"
    val pidMinCmd: Float = -20.0f,              // "pmn"
    val pidMaxCmd: Float = 20.0f,               // "pmx"
    val servoEnabled: Boolean = true,           // "sen"
    val gainScheduleEnabled: Boolean = true,    // "gs"
    val useAngleControl: Boolean = false,       // "ac"
    val rollDelayMs: Int = 0,                   // u16, "rdly"
    val rateCapDps: Float = 60f,                // "rcap" (#253 sentinel: <= 0 → keep default)
    /** Outer angle-loop P-gain (cascaded angle control). */
    val kpAngle: Float = 2.0f,                  // "kpang" (#253 sentinel: <= 0 → keep default)
    /** PID integral-separation anti-windup threshold (deg/s); 0 disables. */
    val integralSepThreshold: Float = 40f,      // "iwind" (#253 sentinel: < 0 → keep default)
    /**
     * True once the rocket reported real roll-control gains rather than the
     * #253 "use firmware default" sentinels.  Until then the three fields
     * above hold the APP's defaults, not the rocket's values — #915 adoption
     * must not mistake them for a report.  Not a wire field.
     */
    val rollGainsReported: Boolean = false,
    val guidanceEnabled: Boolean = false,       // "ge"
    val cameraType: Int = 2,                    // u8, "camt"
    /** 0xFF auto / 0..23 manual (null = not reported).  NOT carried by the
     *  `config` message — arrives on `imu_orient` ("set"); a `config`
     *  readback therefore resets it to null, exactly like iOS. */
    val imuOrientSetting: Int? = null,          // u8?
    /** ISM6 logging rate readback (null = not reported). */
    val imuRateHz: Int? = null,                 // u16?, "irate"
    val loraFreqMHz: Float? = null,             // "lf"
    val loraSF: Int? = null,                    // u8?, "lsf"
    val loraBwKHz: Float? = null,               // "lbw"
    val loraCR: Int? = null,                    // u8?, "lcr"
    val loraTxPower: Int? = null,               // i8?, "lpw"
    /** #106 fixed-frequency override (BS-controlled); null if not reported. */
    val loraHopDisabled: Boolean? = null,       // "lhd"
    /**
     * #150: firmware-computed hop dwell for the current modulation.
     * 0 = hopping not legally possible at this preset (GUI greys the option);
     * null = device doesn't report it (pre-#150 firmware).
     */
    val loraHopDwell: Int? = null,              // "lhdw"
    val pyro1Enabled: Boolean = false,          // "p1e" (config_pyro only)
    val pyro1TriggerMode: Int = 0,              // u8, "p1m"
    val pyro1TriggerValue: Float = 1.0f,        // "p1v"
    val pyro2Enabled: Boolean = false,
    val pyro2TriggerMode: Int = 0,
    val pyro2TriggerValue: Float = 100.0f,
    val pyro3Enabled: Boolean = false,
    val pyro3TriggerMode: Int = 0,
    val pyro3TriggerValue: Float = 0.0f,
    val pyro4Enabled: Boolean = false,
    val pyro4TriggerMode: Int = 0,
    val pyro4TriggerValue: Float = 0.0f,
)

/**
 * Parsed `"type":"config"` readback — the raw per-key values, before the
 * default-fill / sentinel / merge step in [applyTo].  Split from the apply so
 * tests can pin both halves; the demux ([TelemetryDispatch]) hands this out
 * and the session layer folds it into its cached [RocketConfig].
 */
public data class ConfigMessage(
    val sb1: Long? = null,
    val shz: Long? = null,
    val smn: Long? = null,
    val smx: Long? = null,
    val kp: Float? = null,
    val ki: Float? = null,
    val kd: Float? = null,
    val pmn: Float? = null,
    val pmx: Float? = null,
    val sen: Boolean? = null,
    val gs: Boolean? = null,
    val ac: Boolean? = null,
    val rdly: Long? = null,
    val rcap: Float? = null,
    val kpang: Float? = null,
    val iwind: Float? = null,
    val ge: Boolean? = null,
    val camt: Long? = null,
    val irate: Long? = null,
    val lf: Float? = null,
    val lsf: Long? = null,
    val lbw: Float? = null,
    val lcr: Long? = null,
    val lpw: Long? = null,
    val lhd: Boolean? = null,
    val lhdw: Long? = null,
) {
    /**
     * Port of the iOS `"type":"config"` handler (BLEDevice.parseTelemetryData):
     * starts from a FRESH default [RocketConfig] — an absent key falls back to
     * the default, never to [previous] — then:
     *  - #253 sentinels: `rcap`/`kpang` <= 0 and `iwind` < 0 mean "firmware
     *    default, keep the local value" → the field keeps the default;
     *  - `irate` only lands when it fits u16 (iOS `UInt16(exactly:)`);
     *  - LoRa fields are nullable pass-throughs (absent → null);
     *  - the pyro fields are copied from [previous] (a `config` frame never
     *    carries them — they arrive on `config_pyro`).
     *
     * Out-of-range integers are CLAMPED to the iOS field's width; iOS would
     * crash on its trapping `Int16(...)`/`UInt8(...)` inits there, which is
     * not portable behavior — see port NOTES.
     */
    public fun applyTo(previous: RocketConfig?): RocketConfig {
        val d = RocketConfig()
        return RocketConfig(
            servoBias1 = (sb1 ?: d.servoBias1.toLong()).coerceIn(-32768, 32767).toInt(),
            servoHz = (shz ?: d.servoHz.toLong()).coerceIn(-32768, 32767).toInt(),
            servoMinUs = (smn ?: d.servoMinUs.toLong()).coerceIn(-32768, 32767).toInt(),
            servoMaxUs = (smx ?: d.servoMaxUs.toLong()).coerceIn(-32768, 32767).toInt(),
            pidKp = kp ?: d.pidKp,
            pidKi = ki ?: d.pidKi,
            pidKd = kd ?: d.pidKd,
            pidMinCmd = pmn ?: d.pidMinCmd,
            pidMaxCmd = pmx ?: d.pidMaxCmd,
            servoEnabled = sen ?: d.servoEnabled,
            gainScheduleEnabled = gs ?: d.gainScheduleEnabled,
            useAngleControl = ac ?: d.useAngleControl,
            rollDelayMs = (rdly ?: d.rollDelayMs.toLong()).coerceIn(0, 0xFFFF).toInt(),
            // #253 sentinel gates — keep the default on sentinel values.
            rateCapDps = rcap?.takeIf { it > 0 } ?: d.rateCapDps,
            kpAngle = kpang?.takeIf { it > 0 } ?: d.kpAngle,
            integralSepThreshold = iwind?.takeIf { it >= 0 } ?: d.integralSepThreshold,
            rollGainsReported = rcap?.let { it > 0 } ?: false,
            guidanceEnabled = ge ?: d.guidanceEnabled,
            cameraType = (camt ?: d.cameraType.toLong()).coerceIn(0, 0xFF).toInt(),
            imuOrientSetting = d.imuOrientSetting,   // config never carries it (iOS: fresh nil)
            imuRateHz = irate?.takeIf { it in 0..0xFFFF }?.toInt(),
            loraFreqMHz = lf,
            loraSF = lsf?.coerceIn(0, 0xFF)?.toInt(),
            loraBwKHz = lbw,
            loraCR = lcr?.coerceIn(0, 0xFF)?.toInt(),
            loraTxPower = lpw?.coerceIn(-128, 127)?.toInt(),
            loraHopDisabled = lhd,                   // #106 (null if device doesn't report it)
            loraHopDwell = lhdw?.coerceIn(Int.MIN_VALUE.toLong(), Int.MAX_VALUE.toLong())?.toInt(),
            // A config message does NOT carry pyro fields — preserve them.
            pyro1Enabled = previous?.pyro1Enabled ?: d.pyro1Enabled,
            pyro1TriggerMode = previous?.pyro1TriggerMode ?: d.pyro1TriggerMode,
            pyro1TriggerValue = previous?.pyro1TriggerValue ?: d.pyro1TriggerValue,
            pyro2Enabled = previous?.pyro2Enabled ?: d.pyro2Enabled,
            pyro2TriggerMode = previous?.pyro2TriggerMode ?: d.pyro2TriggerMode,
            pyro2TriggerValue = previous?.pyro2TriggerValue ?: d.pyro2TriggerValue,
            pyro3Enabled = previous?.pyro3Enabled ?: d.pyro3Enabled,
            pyro3TriggerMode = previous?.pyro3TriggerMode ?: d.pyro3TriggerMode,
            pyro3TriggerValue = previous?.pyro3TriggerValue ?: d.pyro3TriggerValue,
            pyro4Enabled = previous?.pyro4Enabled ?: d.pyro4Enabled,
            pyro4TriggerMode = previous?.pyro4TriggerMode ?: d.pyro4TriggerMode,
            pyro4TriggerValue = previous?.pyro4TriggerValue ?: d.pyro4TriggerValue,
        )
    }

    public companion object {
        /** Field extraction; never fails (each key individually optional). */
        public fun parse(json: JsonObject): ConfigMessage = ConfigMessage(
            sb1 = JsonBridging.nsInt(json, "sb1"),
            shz = JsonBridging.nsInt(json, "shz"),
            smn = JsonBridging.nsInt(json, "smn"),
            smx = JsonBridging.nsInt(json, "smx"),
            kp = JsonBridging.parseFloatIos(json, "kp"),
            ki = JsonBridging.parseFloatIos(json, "ki"),
            kd = JsonBridging.parseFloatIos(json, "kd"),
            pmn = JsonBridging.parseFloatIos(json, "pmn"),
            pmx = JsonBridging.parseFloatIos(json, "pmx"),
            sen = JsonBridging.nsBool(json, "sen"),
            gs = JsonBridging.nsBool(json, "gs"),
            ac = JsonBridging.nsBool(json, "ac"),
            rdly = JsonBridging.nsInt(json, "rdly"),
            rcap = JsonBridging.parseFloatIos(json, "rcap"),
            kpang = JsonBridging.parseFloatIos(json, "kpang"),
            iwind = JsonBridging.parseFloatIos(json, "iwind"),
            ge = JsonBridging.nsBool(json, "ge"),
            camt = JsonBridging.nsInt(json, "camt"),
            irate = JsonBridging.nsInt(json, "irate"),
            lf = JsonBridging.parseFloatIos(json, "lf"),
            lsf = JsonBridging.nsInt(json, "lsf"),
            lbw = JsonBridging.parseFloatIos(json, "lbw"),
            lcr = JsonBridging.nsInt(json, "lcr"),
            lpw = JsonBridging.nsInt(json, "lpw"),
            lhd = JsonBridging.nsBool(json, "lhd"),
            lhdw = JsonBridging.nsInt(json, "lhdw"),
        )
    }
}

/**
 * Parsed `"type":"config_pyro"` readback (4 channels).  Unlike
 * [ConfigMessage], [applyTo] EDITS the previous config in place — absent keys
 * keep the previous channel values, and every non-pyro field is preserved.
 */
public data class ConfigPyroMessage(
    val p1e: Boolean? = null,
    val p1m: Long? = null,
    val p1v: Float? = null,
    val p2e: Boolean? = null,
    val p2m: Long? = null,
    val p2v: Float? = null,
    val p3e: Boolean? = null,
    val p3m: Long? = null,
    val p3v: Float? = null,
    val p4e: Boolean? = null,
    val p4m: Long? = null,
    val p4v: Float? = null,
) {
    /**
     * Port of the iOS `"type":"config_pyro"` handler: base is [previous] (or
     * a fresh default when no config exists yet), pyro fields overridden only
     * where present.  Modes are clamped to u8 (iOS traps — see NOTES).
     */
    public fun applyTo(previous: RocketConfig?): RocketConfig {
        val base = previous ?: RocketConfig()
        fun mode(v: Long?, current: Int): Int = (v ?: current.toLong()).coerceIn(0, 0xFF).toInt()
        return base.copy(
            pyro1Enabled = p1e ?: base.pyro1Enabled,
            pyro1TriggerMode = mode(p1m, base.pyro1TriggerMode),
            pyro1TriggerValue = p1v ?: base.pyro1TriggerValue,
            pyro2Enabled = p2e ?: base.pyro2Enabled,
            pyro2TriggerMode = mode(p2m, base.pyro2TriggerMode),
            pyro2TriggerValue = p2v ?: base.pyro2TriggerValue,
            pyro3Enabled = p3e ?: base.pyro3Enabled,
            pyro3TriggerMode = mode(p3m, base.pyro3TriggerMode),
            pyro3TriggerValue = p3v ?: base.pyro3TriggerValue,
            pyro4Enabled = p4e ?: base.pyro4Enabled,
            pyro4TriggerMode = mode(p4m, base.pyro4TriggerMode),
            pyro4TriggerValue = p4v ?: base.pyro4TriggerValue,
        )
    }

    public companion object {
        /** Field extraction; never fails (each key individually optional). */
        public fun parse(json: JsonObject): ConfigPyroMessage = ConfigPyroMessage(
            p1e = JsonBridging.nsBool(json, "p1e"),
            p1m = JsonBridging.nsInt(json, "p1m"),
            p1v = JsonBridging.parseFloatIos(json, "p1v"),
            p2e = JsonBridging.nsBool(json, "p2e"),
            p2m = JsonBridging.nsInt(json, "p2m"),
            p2v = JsonBridging.parseFloatIos(json, "p2v"),
            p3e = JsonBridging.nsBool(json, "p3e"),
            p3m = JsonBridging.nsInt(json, "p3m"),
            p3v = JsonBridging.parseFloatIos(json, "p3v"),
            p4e = JsonBridging.nsBool(json, "p4e"),
            p4m = JsonBridging.nsInt(json, "p4m"),
            p4v = JsonBridging.parseFloatIos(json, "p4v"),
        )
    }
}
