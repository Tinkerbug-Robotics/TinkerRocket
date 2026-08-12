package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonNull
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.jsonObject
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.withSign

/**
 * How the FC determined its board→rocket mounting orientation
 * (mirrors TR_Orientation ORIENT_MODE_*; iOS SensorTypes.swift
 * IMUOrientationMode).  Lives here until the FlightSettingsData chunk lands —
 * if that chunk also defines it, keep ONE copy (coordinator: dedupe).
 */
public enum class IMUOrientationMode(public val raw: Int) {
    UNKNOWN(-1),
    DEFAULT_MOUNTING(0),   // identity, nothing configured
    MANUAL(1),             // user/config supplied
    AUTO_SNAP(2),          // pad-gravity detect, snapped to an axis
    AUTO_EXACT(3);         // pad-gravity detect, exact off-axis rotation

    public val label: String
        get() = when (this) {
            UNKNOWN -> "—"
            DEFAULT_MOUNTING -> "default"
            MANUAL -> "manual"
            AUTO_SNAP -> "auto"
            AUTO_EXACT -> "auto, off-axis"
        }
}

/**
 * "−Z r90"-style mounting name (mirrors firmware orientCodeName and iOS
 * FlightSettingsData.b2rName): which board axis points at the nose plus the
 * quarter-turn clocking.  Codes 0..23; anything else is "?".
 * Internal until the FlightSettingsData chunk lands (coordinator: dedupe).
 */
internal fun b2rOrientationName(code: Int): String {
    val axes = listOf("+X", "-X", "+Y", "-Y", "+Z", "-Z")
    if (code !in 0 until 24) return "?"
    val base = axes[code / 4]
    val clock = (code % 4) * 90
    return if (clock == 0) base else "$base r$clock"
}

/**
 * Live BLE telemetry JSON model — a semantics-exact port of iOS
 * `TelemetryData.swift` (the behavioral reference; short JSON keys mirror its
 * CodingKeys, which in turn mirror TR_BLE_To_APP.cpp:buildTelemetryJSON).
 *
 * Decode leniency is deliberately ASYMMETRIC and must stay that way:
 *  - every INTEGER key goes through flexInt (#293/#571) — accepts JSON int,
 *    float (truncated toward zero, like Swift `Int(d)`), or numeric string;
 *    garbage degrades that ONE field to its default, never the frame;
 *  - Float / Double / String keys are STRICT — a type mismatch (e.g. "vol"
 *    carrying a string) fails the WHOLE frame, exactly like iOS's throwing
 *    `decodeIfPresent` whose caller catches and discards.  Do not make these
 *    tolerant: iOS shipped bugs in BOTH directions (#293 too strict, and
 *    over-leniency would un-pin the contract).
 *  - missing keys fall back to iOS defaults: state "UNKNOWN", numSats 0,
 *    bitfields 0, dataStatus LIVE, u32/u16 conversions clamp into range.
 */
public data class TelemetryData(
    val soc: Float? = null,                   // Battery state of charge %
    val current: Float? = null,               // "cur"  Battery current mA
    val voltage: Float? = null,               // "vol"  Battery voltage V
    val latitude: Double? = null,             // "lat"  GPS latitude degrees
    val longitude: Double? = null,            // "lon"  GPS longitude degrees
    // gdop exists on the iOS struct but is ABSENT from CodingKeys and never
    // read in init(from:) — it is vestigial and always nil after a decode.
    // Ported as-is: decode() never sets it.
    val gdop: Float? = null,
    val numSats: Int = 0,                     // "nsat"
    val state: String = "UNKNOWN",            // "st"
    val activeFile: String = "",              // "af"
    val rxKbs: Float? = null,                 // "rxk"  I2C RX rate kB/s
    val wrKbs: Float? = null,                 // "wrk"  Flash write rate kB/s
    val framesRx: Long = 0,                   // "frx"  u32 (UInt32 clamping)
    val framesDrop: Long = 0,                 // "fdr"  u32 (UInt32 clamping)
    val maxAltM: Float? = null,               // "malt"
    val maxSpeedMps: Float? = null,           // "mspd"
    val pressureAlt: Float? = null,           // "palt" baro pressure altitude m
    val altitudeRate: Float? = null,          // "arate" vertical rate m/s
    val gnssAlt: Float? = null,               // "galt" GNSS altitude m (BS only)

    // EKF launch-relative ENU velocity (#191).  null = firmware predates the
    // fields or no FC data — the ascent landing prediction gates on these.
    val velE: Float? = null,                  // "ve" East m/s
    val velN: Float? = null,                  // "vn" North m/s
    val velU: Float? = null,                  // "vu" Up m/s (EKF; altitudeRate is the baro KF)

    // IMU data (ISM6HG256)
    val lowGX: Float? = null,                 // "lx" m/s²
    val lowGY: Float? = null,                 // "ly"
    val lowGZ: Float? = null,                 // "lz"
    val highGX: Float? = null,                // "hx"
    val highGY: Float? = null,                // "hy"
    val highGZ: Float? = null,                // "hz"
    val gyroX: Float? = null,                 // "gx" deg/s
    val gyroY: Float? = null,                 // "gy"
    val gyroZ: Float? = null,                 // "gz"

    // Attitude (FlightComputer onboard estimation)
    val rollCmd: Float? = null,               // "rcmd" Roll command deg (PID output)
    val q0: Float? = null,                    // Quaternion w (scalar-first, body-to-NED)
    val q1: Float? = null,                    // x
    val q2: Float? = null,                    // y
    val q3: Float? = null,                    // z

    // LoRa signal quality (base station only)
    val rssi: Float? = null,                  // dBm
    val snr: Float? = null,                   // dB
    // #150: hop-state surface — present only while hopping / after nid drops.
    val hopChannel: Int? = null,              // "hch"
    val netidDrops: Int? = null,              // "nidd"

    // Base station (base station only)
    val bsSoc: Float? = null,                 // "bsoc"
    val bsVoltage: Float? = null,             // "bvol"
    val bsCurrent: Float? = null,             // "bcur"
    // Seconds until the BS silence-timeout closes the active log; the BS omits
    // the key unless bsLoggingActive — null = no countdown.  (iOS UInt16?)
    val bsLogSilenceRemainingS: Int? = null,  // "slrm"

    // #390: rocket board→rocket mounting orientation relayed over LoRa flags2,
    // packed (mode << 5) | code.  null = not reported (pre-#390 firmware, FC
    // not up, or trimmed tail).
    val imuOrientPacked: Int? = null,         // "imo"

    // Packed flight-status bits ("fs") — bit layout must stay in sync with
    // TR_BLE_To_APP.cpp:buildTelemetryJSON.
    val flightStatusBits: Int = 0,
    // Packed pyro status ("ps"): b0 = global armed, then (cont, fired) pairs
    // for channels 1..4.  9 bits total.
    val pyroStatusBits: Int = 0,
    // #303 sensor health scorecard ("h"): 2 bits/item, shifts mirror
    // RocketComputerTypes.h SH_*_SHIFT.  0 = nothing reported → card hidden.
    val sensorHealth: Int = 0,

    // Source rocket identity (base station relay only, null for direct BLE)
    val sourceRocketId: Int? = null,          // "rid"
    val sourceUnitName: String? = null,       // "run"

    // Telemetry freshness (#95): missing "ds" → LIVE (older firmware).
    val dataStatus: DataStatus = DataStatus.LIVE,
    val dataAgeMs: Long = 0,                  // "age" u32, only meaningful when STALE

    // #282: BS set "tr":1 — frame trimmed to fit the BLE MTU window; frame is
    // valid, low-priority tail fields absent by design.
    val fieldsTrimmed: Boolean = false,
) {
    /**
     * iOS `socDisplay` twin.  Clamped to 0..100 for display only — the stored
     * value and the CSV keep whatever came off the wire.  SOC is packed as an
     * i16 spanning -25..125% for headroom, so an exact 0% round-trips to
     * -0.00077 and "%.1f%%" prints it as "-0.0%", which is what a rocket
     * running off USB shows on every line.  Em dash for absent, as elsewhere
     * on this dashboard (iOS says "N/A" here).
     */
    /** The base station's own pack, same formatting rules as [socDisplay]. */
    public val bsSocDisplay: String
        get() = bsSoc?.let {
            String.format(java.util.Locale.ROOT, "%.1f%%", it.coerceIn(0f, 100f) + 0f)
        } ?: "—"

    public val socDisplay: String
        get() = soc?.let {
            // The `+ 0f` is what actually kills "-0.0%", and it is not
            // redundant with coerceIn: the OC prints SOC to one decimal, so an
            // exact 0% arrives as the literal -0.0, and -0.0 == 0.0 means
            // coerceIn considers it already in range and hands it back
            // untouched.  Adding positive zero is the IEEE way to drop the
            // sign.  coerceIn still earns its place for a genuinely
            // out-of-range value.
            String.format(java.util.Locale.ROOT, "%.1f%%", it.coerceIn(0f, 100f) + 0f)
        } ?: "—"

    // ── Telemetry freshness (#95) ─────────────────────────────────────────
    public enum class DataStatus(public val raw: Int) {
        LIVE(0), STALE(1), SYNCING(2);

        public companion object {
            /** Unknown raw values read as LIVE, mirroring iOS `?? .live`. */
            public fun fromRaw(raw: Int): DataStatus =
                entries.firstOrNull { it.raw == raw } ?: LIVE
        }
    }

    // ── Quaternion-derived attitude (not sent over BLE) ───────────────────
    // Guard: reject malformed quaternions (e.g. all zeros from uninitialized
    // EKF) via the norm window.  All-Float math, matching Swift.

    public val roll: Float?
        get() {
            val w = q0 ?: return null; val x = q1 ?: return null
            val y = q2 ?: return null; val z = q3 ?: return null
            val norm = w * w + x * x + y * y + z * z
            if (!(norm > 0.1f && norm < 2.0f)) return null
            // Body-Z roll, matching the FC roll controller
            // (actual_roll_deg = -atan2(z_east, z_north)); well-defined when
            // vertical, unlike ZYX-Euler roll which gimbal-locks near ±90° pitch.
            val zNorth = 2.0f * (x * z + w * y)
            val zEast = 2.0f * (y * z - w * x)
            return -atan2(zEast, zNorth) * 180.0f / PI.toFloat()
        }

    public val pitch: Float?
        get() {
            val w = q0 ?: return null; val x = q1 ?: return null
            val y = q2 ?: return null; val z = q3 ?: return null
            val norm = w * w + x * x + y * y + z * z
            if (!(norm > 0.1f && norm < 2.0f)) return null
            val sinp = 2.0f * (w * y - z * x)
            if (abs(sinp) >= 1.0f) return 90.0f.withSign(sinp)
            return asin(sinp) * 180.0f / PI.toFloat()
        }

    public val yaw: Float?
        get() {
            val w = q0 ?: return null; val x = q1 ?: return null
            val y = q2 ?: return null; val z = q3 ?: return null
            val norm = w * w + x * x + y * y + z * z
            if (!(norm > 0.1f && norm < 2.0f)) return null
            val siny = 2.0f * (w * z + x * y)
            val cosy = 1.0f - 2.0f * (y * y + z * z)
            return atan2(siny, cosy) * 180.0f / PI.toFloat()
        }

    // ── Flight-status bitfield ("fs", #138 MTU pack) ──────────────────────
    public val launchFlag: Boolean get() = (flightStatusBits and 0x01) != 0
    public val velApo: Boolean get() = (flightStatusBits and 0x02) != 0
    public val altApo: Boolean get() = (flightStatusBits and 0x04) != 0
    public val landedFlag: Boolean get() = (flightStatusBits and 0x08) != 0
    public val pwrPinOn: Boolean get() = (flightStatusBits and 0x10) != 0
    public val cameraRecording: Boolean get() = (flightStatusBits and 0x20) != 0
    public val loggingActive: Boolean get() = (flightStatusBits and 0x40) != 0
    public val bsLoggingActive: Boolean get() = (flightStatusBits and 0x80) != 0
    // #393: simulated flight in progress (NSF_SIM_ACTIVE → fs bit 8).
    public val simActive: Boolean get() = (flightStatusBits and 0x100) != 0
    // #191: motor burnout detected (fs bit 9) — gates ascent landing prediction.
    public val burnoutFlag: Boolean get() = (flightStatusBits and 0x200) != 0

    // ── Pyro status bitfield ("ps") ───────────────────────────────────────
    public val pyroArmed: Boolean get() = (pyroStatusBits and 0x001) != 0
    public val pyro1Cont: Boolean get() = (pyroStatusBits and 0x002) != 0
    public val pyro1Fired: Boolean get() = (pyroStatusBits and 0x004) != 0
    public val pyro2Cont: Boolean get() = (pyroStatusBits and 0x008) != 0
    public val pyro2Fired: Boolean get() = (pyroStatusBits and 0x010) != 0
    public val pyro3Cont: Boolean get() = (pyroStatusBits and 0x020) != 0
    public val pyro3Fired: Boolean get() = (pyroStatusBits and 0x040) != 0
    public val pyro4Cont: Boolean get() = (pyroStatusBits and 0x080) != 0
    public val pyro4Fired: Boolean get() = (pyroStatusBits and 0x100) != 0

    public fun pyroCont(channel: Int): Boolean = when (channel) {
        1 -> pyro1Cont; 2 -> pyro2Cont; 3 -> pyro3Cont; 4 -> pyro4Cont
        else -> false
    }

    public fun pyroFired(channel: Int): Boolean = when (channel) {
        1 -> pyro1Fired; 2 -> pyro2Fired; 3 -> pyro3Fired; 4 -> pyro4Fired
        else -> false
    }

    // ── Sensor health scorecard (#303) ────────────────────────────────────
    public enum class SensorHealth(public val raw: Int) {
        NA(0), OK(1), DEGRADED(2), BAD(3);

        public val label: String
            get() = when (this) {
                NA -> "N/A"; OK -> "OK"; DEGRADED -> "DEGRADED"; BAD -> "BAD"
            }
    }

    private fun shState(shift: Int): SensorHealth {
        val raw = (sensorHealth shr shift) and 0x3
        return SensorHealth.entries.firstOrNull { it.raw == raw } ?: SensorHealth.NA
    }

    public val baroHealth: SensorHealth get() = shState(0)
    public val imuHealth: SensorHealth get() = shState(2)
    public val ekfHealth: SensorHealth get() = shState(4)   // init + isHealthy + covariance converged
    public val magHealth: SensorHealth get() = shState(6)   // advisory only — never gates go/no-go
    public val gnssHealth: SensorHealth get() = shState(8)
    public val battHealth: SensorHealth get() = shState(10)
    public val storageHealth: SensorHealth get() = shState(20)  // #281/#278: flight-log NAND

    // #557: GNSS-absent degraded flight (shift 22) — DISTINCT from gnssHealth
    // (fix health, shift 8).  BAD (0b11) = FC initialized the EKF on the
    // baro+IMU path (dead/deaf module): no absolute position, guidance off.
    public val gnssAbsentMode: Boolean get() = shState(22) == SensorHealth.BAD

    /** Channel 1..4; NA = not configured (and for out-of-range channels). */
    public fun pyroHealth(channel: Int): SensorHealth {
        if (channel !in 1..4) return SensorHealth.NA
        return shState(12 + (channel - 1) * 2)
    }

    public val hasSensorHealth: Boolean get() = sensorHealth != 0

    /** One row of the pre-launch health card (iOS SensorHealthRow; id = name). */
    public data class SensorHealthRow(val name: String, val state: SensorHealth)

    // Core sensors always shown; Storage only when reported; a pyro channel
    // only when configured for the flight (state != NA).
    public val sensorHealthRows: List<SensorHealthRow>
        get() {
            val rows = mutableListOf(
                SensorHealthRow("Baro", baroHealth),
                SensorHealthRow("IMU", imuHealth),
                SensorHealthRow("EKF", ekfHealth),
                SensorHealthRow("GNSS", gnssHealth),
                SensorHealthRow("Battery", battHealth),
                SensorHealthRow("Mag", magHealth),
            )
            if (storageHealth != SensorHealth.NA) {
                rows.add(SensorHealthRow("Storage", storageHealth))
            }
            for (ch in 1..4) {
                if (pyroHealth(ch) != SensorHealth.NA) {
                    rows.add(SensorHealthRow("Pyro $ch", pyroHealth(ch)))
                }
            }
            return rows
        }

    // ── Go/no-go rollup (#303) ────────────────────────────────────────────
    public enum class FlightReadiness {
        UNKNOWN, READY, CAUTION, NOT_READY;

        public val label: String
            get() = when (this) {
                UNKNOWN -> "Waiting for telemetry…"
                READY -> "Ready to fly"
                CAUTION -> "Not ready — check sensors"
                NOT_READY -> "Do not fly"
            }
    }

    // Red = a hard fault waiting won't fix (baro/IMU/battery BAD, full/failing
    // flight-log store, or a configured pyro with no continuity).  Green needs
    // every required item OK — including EKF converged and a GNSS fix.  Mag is
    // advisory and never gates.  Storage absent (NA, older firmware) is
    // ignored, not red.  Amber = anything in between.
    public val flightReadiness: FlightReadiness
        get() {
            if (!hasSensorHealth) return FlightReadiness.UNKNOWN
            val hardFault =
                listOf(baroHealth, imuHealth, battHealth, storageHealth).contains(SensorHealth.BAD) ||
                    (1..4).any { pyroHealth(it) == SensorHealth.BAD }
            if (hardFault) return FlightReadiness.NOT_READY
            val mustBeOK = mutableListOf(baroHealth, imuHealth, battHealth, ekfHealth, gnssHealth)
            if (storageHealth != SensorHealth.NA) mustBeOK.add(storageHealth)  // #281/#278
            for (ch in 1..4) {
                if (pyroHealth(ch) != SensorHealth.NA) mustBeOK.add(pyroHealth(ch))
            }
            return if (mustBeOK.all { it == SensorHealth.OK }) FlightReadiness.READY
            else FlightReadiness.CAUTION
        }

    // ── Logging indicator (#137) ──────────────────────────────────────────
    // Suppress only during the post-flight drain window (state == LANDED),
    // where loggingActive stays true while the end-flight queue drains (and
    // can stick if the drain wedges).  Trust the flag in every other state —
    // manual pre-flight bench logging from READY/PRELAUNCH is real, and a
    // stuck-true surviving a reset to READY is worth surfacing, not hiding.
    public val rocketLoggingActive: Boolean
        get() {
            if (state == "LANDED") return false
            return loggingActive
        }

    // ── Relayed IMU orientation (#390) ────────────────────────────────────
    // Wire layout mirrors LORA2_ORIENT_*: bits 0-4 = discrete code (31 =
    // auto-exact, no code), bits 5-6 = mode (1 default / 2 manual / 3 auto;
    // 0 never arrives — the BS omits "imo").

    public val relayedOrientationMode: IMUOrientationMode
        get() {
            val packed = imuOrientPacked ?: return IMUOrientationMode.UNKNOWN
            val code = packed and 0x1F
            return when ((packed shr 5) and 0x3) {
                1 -> IMUOrientationMode.DEFAULT_MOUNTING
                2 -> IMUOrientationMode.MANUAL
                3 -> if (code == 31) IMUOrientationMode.AUTO_EXACT else IMUOrientationMode.AUTO_SNAP
                else -> IMUOrientationMode.UNKNOWN
            }
        }

    public val relayedOrientationName: String
        get() {
            val packed = imuOrientPacked ?: return ""
            if (relayedOrientationMode == IMUOrientationMode.UNKNOWN) return ""
            val code = packed and 0x1F
            // Auto-exact carries no discrete code — name it so the IMU card's
            // line still renders (it gates on a non-empty name).
            return if (code < 24) b2rOrientationName(code) else "custom"
        }

    public companion object {

        /** Strict-field type mismatch → the whole frame is discarded. */
        private class FrameTypeMismatch : RuntimeException(null, null, false, false)

        /**
         * Parse + decode one telemetry notification.  Returns null on
         * malformed JSON, a non-object top level, or a strict-field type
         * mismatch — mirroring the iOS caller's try/catch-and-skip.
         */
        public fun decode(text: String): TelemetryData? = try {
            decode(Json.parseToJsonElement(text).jsonObject)
        } catch (_: Exception) {
            null
        }

        /** Decode a parsed frame; null on a strict-field type mismatch. */
        public fun decode(json: JsonObject): TelemetryData? = try {
            decodeOrThrow(json)
        } catch (_: FrameTypeMismatch) {
            null
        }

        // #293/#571 tolerant integer read: JSON int, float (truncate toward
        // zero, like Swift Int(d)), or numeric string ("5", "3.9").  Garbage
        // (bool/object/array/non-numeric string) → null, NEVER a frame fail.
        // Swift's flexInt yields a 64-bit Int; the per-field UInt32/UInt16
        // clamps happen at the call sites, so keep the full Long here.
        private fun flexLong(json: JsonObject, key: String): Long? {
            val el = json[key] ?: return null
            if (el is JsonNull) return null
            val p = el as? JsonPrimitive ?: return null
            return if (p.isString) swiftNumericString(p.content) else jsonNumberToLong(p.content)
        }

        // Int-typed iOS fields are 64-bit; Kotlin Int is 32-bit.  Coerce into
        // Int range (values on this wire are far smaller; see NOTES).
        private fun flexInt(json: JsonObject, key: String): Int? =
            flexLong(json, key)?.coerceIn(Int.MIN_VALUE.toLong(), Int.MAX_VALUE.toLong())?.toInt()

        /** JSON number literal → Long, integral literals exact, floats truncated toward zero. */
        private fun jsonNumberToLong(content: String): Long? {
            content.toLongOrNull()?.let { return it }
            val d = content.toDoubleOrNull() ?: return null
            if (!d.isFinite()) return null
            return d.toLong()   // truncates toward zero, saturates at Long bounds
        }

        // Swift `Int(s) ?? Double(s).map { Int($0) }`.  Java's parseDouble is
        // MORE lenient than Swift Double(String) — it trims whitespace and
        // accepts f/F/d/D suffixes — so reject those to avoid being more
        // lenient than iOS.  NaN/inf strings are rejected instead of ported:
        // Swift Int(NaN) would trap (app crash) — we degrade the field.
        private fun swiftNumericString(s: String): Long? {
            s.toLongOrNull()?.let { return it }
            if (s.isEmpty() || s.trim() != s) return null
            if (s.last() in "fFdD") return null
            val d = s.toDoubleOrNull() ?: return null
            if (!d.isFinite()) return null
            return d.toLong()
        }

        // Strict reads — mirror Swift's throwing decodeIfPresent: absent or
        // JSON null → null; any type mismatch kills the WHOLE frame.

        private fun strictFloat(json: JsonObject, key: String): Float? {
            val el = json[key] ?: return null
            if (el is JsonNull) return null
            val p = el as? JsonPrimitive ?: throw FrameTypeMismatch()
            if (p.isString) throw FrameTypeMismatch()
            val d = p.content.toDoubleOrNull() ?: throw FrameTypeMismatch()  // true/false land here
            // Swift throws when a JSON number does not fit in Float.
            if (abs(d) > Float.MAX_VALUE.toDouble()) throw FrameTypeMismatch()
            return d.toFloat()
        }

        private fun strictDouble(json: JsonObject, key: String): Double? {
            val el = json[key] ?: return null
            if (el is JsonNull) return null
            val p = el as? JsonPrimitive ?: throw FrameTypeMismatch()
            if (p.isString) throw FrameTypeMismatch()
            return p.content.toDoubleOrNull() ?: throw FrameTypeMismatch()
        }

        private fun strictString(json: JsonObject, key: String): String? {
            val el = json[key] ?: return null
            if (el is JsonNull) return null
            val p = el as? JsonPrimitive ?: throw FrameTypeMismatch()
            if (!p.isString) throw FrameTypeMismatch()
            return p.content
        }

        private fun clampU32(v: Long): Long = v.coerceIn(0L, 0xFFFF_FFFFL)
        private fun clampU16(v: Long): Int = v.coerceIn(0L, 0xFFFFL).toInt()

        private fun decodeOrThrow(json: JsonObject): TelemetryData = TelemetryData(
            soc = strictFloat(json, "soc"),
            current = strictFloat(json, "cur"),
            voltage = strictFloat(json, "vol"),
            latitude = strictDouble(json, "lat"),
            longitude = strictDouble(json, "lon"),
            // gdop: intentionally NOT decoded — absent from iOS CodingKeys.
            numSats = flexInt(json, "nsat") ?: 0,                       // #571
            state = strictString(json, "st") ?: "UNKNOWN",
            activeFile = strictString(json, "af") ?: "",
            rxKbs = strictFloat(json, "rxk"),
            wrKbs = strictFloat(json, "wrk"),
            framesRx = clampU32(flexLong(json, "frx") ?: 0),            // #571
            framesDrop = clampU32(flexLong(json, "fdr") ?: 0),          // #571
            maxAltM = strictFloat(json, "malt"),
            maxSpeedMps = strictFloat(json, "mspd"),
            pressureAlt = strictFloat(json, "palt"),
            altitudeRate = strictFloat(json, "arate"),
            gnssAlt = strictFloat(json, "galt"),
            velE = strictFloat(json, "ve"),
            velN = strictFloat(json, "vn"),
            velU = strictFloat(json, "vu"),
            lowGX = strictFloat(json, "lx"),
            lowGY = strictFloat(json, "ly"),
            lowGZ = strictFloat(json, "lz"),
            highGX = strictFloat(json, "hx"),
            highGY = strictFloat(json, "hy"),
            highGZ = strictFloat(json, "hz"),
            gyroX = strictFloat(json, "gx"),
            gyroY = strictFloat(json, "gy"),
            gyroZ = strictFloat(json, "gz"),
            rollCmd = strictFloat(json, "rcmd"),
            q0 = strictFloat(json, "q0"),
            q1 = strictFloat(json, "q1"),
            q2 = strictFloat(json, "q2"),
            q3 = strictFloat(json, "q3"),
            rssi = strictFloat(json, "rssi"),
            snr = strictFloat(json, "snr"),
            hopChannel = flexInt(json, "hch"),                          // #571
            netidDrops = flexInt(json, "nidd"),                         // #571
            bsSoc = strictFloat(json, "bsoc"),
            bsVoltage = strictFloat(json, "bvol"),
            bsCurrent = strictFloat(json, "bcur"),
            bsLogSilenceRemainingS = flexLong(json, "slrm")?.let(::clampU16),  // #571
            imuOrientPacked = flexInt(json, "imo"),          // #293: tolerant, like fs/ps
            flightStatusBits = flexInt(json, "fs") ?: 0,     // #293: tolerant
            pyroStatusBits = flexInt(json, "ps") ?: 0,       // #293: tolerant
            sensorHealth = flexInt(json, "h") ?: 0,                     // #571
            sourceRocketId = flexInt(json, "rid"),           // #571: rid is the demux key
            sourceUnitName = strictString(json, "run"),
            // #95: missing "ds" → LIVE (older firmware doesn't emit it);
            // unknown raw values also read as LIVE.
            dataStatus = DataStatus.fromRaw(flexInt(json, "ds") ?: 0),  // #571
            dataAgeMs = clampU32(flexLong(json, "age") ?: 0),           // #293: tolerant
            fieldsTrimmed = (flexInt(json, "tr") ?: 0) != 0,            // #282
        )
    }
}
