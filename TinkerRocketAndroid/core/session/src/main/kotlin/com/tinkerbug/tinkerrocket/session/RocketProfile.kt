package com.tinkerbug.tinkerrocket.session

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.boolean
import kotlinx.serialization.json.buildJsonArray
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.double
import kotlinx.serialization.json.doubleOrNull
import kotlinx.serialization.json.float
import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import java.util.UUID

/**
 * Per-rocket settings profile (#132) — port of iOS RocketProfile.swift.
 * The app is source-of-truth: on connect the active profile is pushed to the
 * rocket (see [ActiveRocketSyncer]), so EVERY DEFAULT HERE MUST EQUAL the
 * firmware config.h factory default — a mismatch silently re-tunes the
 * rocket.  Pinned by FirmwareDefaultsTest against the config.h values.
 *
 * Stored JSON is SCHEMA-IDENTICAL to iOS Codable output (key names, Apple-
 * epoch dates as Double seconds since 2001-01-01, uppercase UUID strings,
 * nil fields omitted) so profiles could round-trip across platforms.
 */
public data class RocketProfile(
    // Identity / meta
    val id: UUID = UUID.randomUUID(),
    val name: String,
    val notes: String = "",
    val createdAtMs: Long,
    val updatedAtMs: Long,
    val lastUsedUnitID: String? = null,

    // Rocket toggles
    val soundsEnabled: Boolean = false,
    val servoControlEnabled: Boolean = true,
    val gainScheduleEnabled: Boolean = true,
    val useAngleControl: Boolean = false,
    val rollDelayMs: Int = 0,
    val rateCapDps: Float = 60f,           // config::KP_ANGLE_RATE_CAP_DPS
    val kpAngle: Float = 2.0f,             // config::KP_ANGLE
    val guidanceEnabled: Boolean = false,
    val pnNavGain: Float = 5.0f,           // config::PN_NAV_GAIN
    val pnMaxAccel: Float = 20.0f,         // config::PN_MAX_ACCEL_MPS2
    val pnAccelToFin: Float = 4.0f,        // config::PN_ACCEL_TO_FIN_DEG
    val pnMaxFinDeg: Float = 15.0f,        // config::PN_MAX_FIN_DEG
    val pnMinSpeed: Float = 15.0f,         // config::PN_MIN_SPEED_MPS
    val pnCoastDelayMs: Int = 0,           // config::PN_COAST_DELAY_MS
    val pnTargetAltM: Float = 600.0f,      // config::PN_TARGET_ALT_M
    val pnTargetMode: Int = 0,             // GUIDE_TARGET_OVERHEAD
    val pnTargetE: Float = 0f,
    val pnTargetN: Float = 0f,
    val pnKpPos: Float = 0.8f,             // config::PN_KP_POS_PER_S2
    val pnKdVel: Float = 1.5f,             // config::PN_KD_VEL_PER_S
    val pnGuidanceLaw: Int = 0,            // config::GUIDANCE_LAW_DEFAULT
    val cameraType: Int = 2,               // config::CAMERA_TYPE (RunCam)
    val imuOrientSetting: Int = 0,         // manual identity; 0xFF = auto
    val imuRateHz: Int = 1920,

    // Servo (#561: bias defaults 0 — a generic default must not carry trim)
    val servoBias1: Int = 0,
    val servoBias2: Int = 0,
    val servoBias3: Int = 0,
    val servoBias4: Int = 0,
    val servoHz: Int = 333,                // config::SERVO_HZ
    val servoMinUs: Int = 1000,            // config::SERVO_MIN_US
    val servoMaxUs: Int = 2000,            // config::SERVO_MAX_US
    /** #449: total fin travel between the endpoints — fin angles are DERIVED. */
    val finTravelDeg: Float = 120.0f,

    // Fin layout (defaults reproduce the firmware "+" mix)
    val finRingMode: Int = 0,
    val finServoAtSlot: List<Int> = listOf(1, 2, 3, 4),
    val finReverse: List<Boolean> = listOf(false, false, false, false),
    val finRollReverse: List<Boolean> = listOf(false, false, false, false),

    // PID (config::KP/KI/KD/MIN_CMD/MAX_CMD/INTEGRAL_SEP_THRESHOLD_DPS)
    val pidKp: Float = 0.12f,
    val pidKi: Float = 0.01f,
    val pidKd: Float = 0.0f,
    val pidMinCmd: Float = -20.0f,
    val pidMaxCmd: Float = 20.0f,
    val integralSepThreshold: Float = 40f,

    // Roll profile
    val rollWaypoints: List<ProfileRollWaypoint> = emptyList(),

    // Pyro (auto-pushed; ARMING stays a separate explicit action)
    val pyro1Enabled: Boolean = false,
    val pyro1TriggerMode: Int = 0,
    val pyro1TriggerValue: Float = 1.0f,
    val pyro2Enabled: Boolean = false,
    val pyro2TriggerMode: Int = 0,
    val pyro2TriggerValue: Float = 100.0f,
    val pyro3Enabled: Boolean = false,
    val pyro3TriggerMode: Int = 0,
    val pyro3TriggerValue: Float = 0.0f,
    val pyro4Enabled: Boolean = false,
    val pyro4TriggerMode: Int = 0,
    val pyro4TriggerValue: Float = 0.0f,

    // Recovery (#156)
    val drogueRateFps: Double = 60.0,
    val mainRateFps: Double = 12.0,
    val mainDeployAltAglFt: Double = 700.0,
    val ballisticDragK: Double = 5e-4,

    // Calibration (board-specific)
    val magCal: MagCalData? = null,
    val sensorCal: SensorCalData? = null,
) {
    /** #449 derived fin calibration — endpoint angles are ±travel/2, always. */
    public val finMinDeg: Float get() = -finTravelDeg / 2.0f
    public val finMaxDeg: Float get() = finTravelDeg / 2.0f

    public val controlModeLabel: String
        get() = when {
            guidanceEnabled -> "Guidance"
            useAngleControl -> "Track Profile"
            else -> "Null Roll"
        }

    public companion object {
        /** Standard hobby servo: 120° across a 1000 µs span. */
        public const val STANDARD_FIN_DEG_PER_US: Float = 120.0f / 1000.0f

        public fun standardFinTravelDeg(minUs: Int, maxUs: Int): Float =
            (maxUs - minUs) * STANDARD_FIN_DEG_PER_US

        public fun makeDefault(name: String, nowMs: Long): RocketProfile =
            RocketProfile(name = name, createdAtMs = nowMs, updatedAtMs = nowMs)
    }
}

public data class ProfileRollWaypoint(
    val id: UUID = UUID.randomUUID(),
    val timeSeconds: Float,
    val angleDeg: Float,
    /** Legacy wire field — always sent as 0 (.angle); kept for old profiles. */
    val mode: Int = 0,
)

public data class MagCalData(
    val offsetX: Int,
    val offsetY: Int,
    val offsetZ: Int,
    val fieldRuT: Float,
    val residualUT: Float,
    val calibratedOnUnitID: String,
    val calibratedAtMs: Long,
)

public data class SensorCalData(
    val gyroX: Int,
    val gyroY: Int,
    val gyroZ: Int,
    val hgX: Float,
    val hgY: Float,
    val hgZ: Float,
    val calibratedOnUnitID: String,
    val calibratedAtMs: Long,
)

/**
 * iOS-schema-identical JSON codec.  Decode-with-defaults for EVERY field
 * (a missing key never fails the profile — the iOS custom decoder rule that
 * keeps old saved profiles loading), with the #449 exclusion: the legacy
 * `finMinDeg`/`finMaxDeg` keys are DELIBERATELY NOT READ — reading them
 * back would resurrect the 2×-off fin calibration; travel is seeded from
 * the pulse endpoints on the standard-servo line instead.
 */
public object RocketProfileCodec {

    private const val APPLE_EPOCH_OFFSET_S = 978307200.0  // 2001-01-01T00:00:00Z

    private fun appleDate(ms: Long): JsonPrimitive =
        JsonPrimitive(ms / 1000.0 - APPLE_EPOCH_OFFSET_S)

    private fun JsonObject.dateMs(key: String, default: Long): Long =
        (this[key] as? JsonPrimitive)?.doubleOrNull
            ?.let { ((it + APPLE_EPOCH_OFFSET_S) * 1000.0).toLong() } ?: default

    public fun encode(p: RocketProfile): String = buildJsonObject {
        put("id", JsonPrimitive(p.id.toString().uppercase()))
        put("name", JsonPrimitive(p.name))
        put("notes", JsonPrimitive(p.notes))
        put("createdAt", appleDate(p.createdAtMs))
        put("updatedAt", appleDate(p.updatedAtMs))
        p.lastUsedUnitID?.let { put("lastUsedUnitID", JsonPrimitive(it)) }
        put("soundsEnabled", JsonPrimitive(p.soundsEnabled))
        put("servoControlEnabled", JsonPrimitive(p.servoControlEnabled))
        put("gainScheduleEnabled", JsonPrimitive(p.gainScheduleEnabled))
        put("useAngleControl", JsonPrimitive(p.useAngleControl))
        put("rollDelayMs", JsonPrimitive(p.rollDelayMs))
        put("rateCapDps", JsonPrimitive(p.rateCapDps))
        put("kpAngle", JsonPrimitive(p.kpAngle))
        put("guidanceEnabled", JsonPrimitive(p.guidanceEnabled))
        put("pnNavGain", JsonPrimitive(p.pnNavGain))
        put("pnMaxAccel", JsonPrimitive(p.pnMaxAccel))
        put("pnAccelToFin", JsonPrimitive(p.pnAccelToFin))
        put("pnMaxFinDeg", JsonPrimitive(p.pnMaxFinDeg))
        put("pnMinSpeed", JsonPrimitive(p.pnMinSpeed))
        put("pnCoastDelayMs", JsonPrimitive(p.pnCoastDelayMs))
        put("pnTargetAltM", JsonPrimitive(p.pnTargetAltM))
        put("pnTargetMode", JsonPrimitive(p.pnTargetMode))
        put("pnTargetE", JsonPrimitive(p.pnTargetE))
        put("pnTargetN", JsonPrimitive(p.pnTargetN))
        put("pnKpPos", JsonPrimitive(p.pnKpPos))
        put("pnKdVel", JsonPrimitive(p.pnKdVel))
        put("pnGuidanceLaw", JsonPrimitive(p.pnGuidanceLaw))
        put("cameraType", JsonPrimitive(p.cameraType))
        put("imuOrientSetting", JsonPrimitive(p.imuOrientSetting))
        put("imuRateHz", JsonPrimitive(p.imuRateHz))
        put("servoBias1", JsonPrimitive(p.servoBias1))
        put("servoBias2", JsonPrimitive(p.servoBias2))
        put("servoBias3", JsonPrimitive(p.servoBias3))
        put("servoBias4", JsonPrimitive(p.servoBias4))
        put("servoHz", JsonPrimitive(p.servoHz))
        put("servoMinUs", JsonPrimitive(p.servoMinUs))
        put("servoMaxUs", JsonPrimitive(p.servoMaxUs))
        put("finTravelDeg", JsonPrimitive(p.finTravelDeg))
        put("finRingMode", JsonPrimitive(p.finRingMode))
        put("finServoAtSlot", buildJsonArray { p.finServoAtSlot.forEach { add(JsonPrimitive(it)) } })
        put("finReverse", buildJsonArray { p.finReverse.forEach { add(JsonPrimitive(it)) } })
        put("finRollReverse", buildJsonArray { p.finRollReverse.forEach { add(JsonPrimitive(it)) } })
        put("pidKp", JsonPrimitive(p.pidKp))
        put("pidKi", JsonPrimitive(p.pidKi))
        put("pidKd", JsonPrimitive(p.pidKd))
        put("pidMinCmd", JsonPrimitive(p.pidMinCmd))
        put("pidMaxCmd", JsonPrimitive(p.pidMaxCmd))
        put("integralSepThreshold", JsonPrimitive(p.integralSepThreshold))
        put(
            "rollWaypoints",
            buildJsonArray {
                p.rollWaypoints.forEach { wp ->
                    add(
                        buildJsonObject {
                            put("id", JsonPrimitive(wp.id.toString().uppercase()))
                            put("timeSeconds", JsonPrimitive(wp.timeSeconds))
                            put("angleDeg", JsonPrimitive(wp.angleDeg))
                            put("mode", JsonPrimitive(wp.mode))
                        },
                    )
                }
            },
        )
        put("pyro1Enabled", JsonPrimitive(p.pyro1Enabled))
        put("pyro1TriggerMode", JsonPrimitive(p.pyro1TriggerMode))
        put("pyro1TriggerValue", JsonPrimitive(p.pyro1TriggerValue))
        put("pyro2Enabled", JsonPrimitive(p.pyro2Enabled))
        put("pyro2TriggerMode", JsonPrimitive(p.pyro2TriggerMode))
        put("pyro2TriggerValue", JsonPrimitive(p.pyro2TriggerValue))
        put("pyro3Enabled", JsonPrimitive(p.pyro3Enabled))
        put("pyro3TriggerMode", JsonPrimitive(p.pyro3TriggerMode))
        put("pyro3TriggerValue", JsonPrimitive(p.pyro3TriggerValue))
        put("pyro4Enabled", JsonPrimitive(p.pyro4Enabled))
        put("pyro4TriggerMode", JsonPrimitive(p.pyro4TriggerMode))
        put("pyro4TriggerValue", JsonPrimitive(p.pyro4TriggerValue))
        put("drogueRateFps", JsonPrimitive(p.drogueRateFps))
        put("mainRateFps", JsonPrimitive(p.mainRateFps))
        put("mainDeployAltAglFt", JsonPrimitive(p.mainDeployAltAglFt))
        put("ballisticDragK", JsonPrimitive(p.ballisticDragK))
        p.magCal?.let { cal ->
            put(
                "magCal",
                buildJsonObject {
                    put("offsetX", JsonPrimitive(cal.offsetX))
                    put("offsetY", JsonPrimitive(cal.offsetY))
                    put("offsetZ", JsonPrimitive(cal.offsetZ))
                    put("fieldR_uT", JsonPrimitive(cal.fieldRuT))
                    put("residualUT", JsonPrimitive(cal.residualUT))
                    put("calibratedOnUnitID", JsonPrimitive(cal.calibratedOnUnitID))
                    put("calibratedAt", appleDate(cal.calibratedAtMs))
                },
            )
        }
        p.sensorCal?.let { cal ->
            put(
                "sensorCal",
                buildJsonObject {
                    put("gyroX", JsonPrimitive(cal.gyroX))
                    put("gyroY", JsonPrimitive(cal.gyroY))
                    put("gyroZ", JsonPrimitive(cal.gyroZ))
                    put("hgX", JsonPrimitive(cal.hgX))
                    put("hgY", JsonPrimitive(cal.hgY))
                    put("hgZ", JsonPrimitive(cal.hgZ))
                    put("calibratedOnUnitID", JsonPrimitive(cal.calibratedOnUnitID))
                    put("calibratedAt", appleDate(cal.calibratedAtMs))
                },
            )
        }
    }.toString()

    /** Returns null only on unparseable JSON or a missing `name` (iOS parity). */
    public fun decode(text: String, nowMs: Long): RocketProfile? {
        val o = runCatching { Json.parseToJsonElement(text).jsonObject }.getOrNull() ?: return null
        val d = RocketProfile(name = "", createdAtMs = nowMs, updatedAtMs = nowMs)
        val name = o.str("name") ?: return null

        fun uuid(key: String): UUID? =
            o.str(key)?.let { runCatching { UUID.fromString(it) }.getOrNull() }

        val servoMinUs = o.int("servoMinUs") ?: d.servoMinUs
        val servoMaxUs = o.int("servoMaxUs") ?: d.servoMaxUs
        val finSlots = o.intList("finServoAtSlot") ?: d.finServoAtSlot
        val finRev = o.boolList("finReverse") ?: d.finReverse
        val finRollRev = o.boolList("finRollReverse") ?: d.finRollReverse

        return RocketProfile(
            id = uuid("id") ?: UUID.randomUUID(),
            name = name,
            notes = o.str("notes") ?: d.notes,
            createdAtMs = o.dateMs("createdAt", d.createdAtMs),
            updatedAtMs = o.dateMs("updatedAt", d.updatedAtMs),
            lastUsedUnitID = o.str("lastUsedUnitID"),
            soundsEnabled = o.bool("soundsEnabled") ?: d.soundsEnabled,
            servoControlEnabled = o.bool("servoControlEnabled") ?: d.servoControlEnabled,
            gainScheduleEnabled = o.bool("gainScheduleEnabled") ?: d.gainScheduleEnabled,
            useAngleControl = o.bool("useAngleControl") ?: d.useAngleControl,
            rollDelayMs = o.int("rollDelayMs") ?: d.rollDelayMs,
            rateCapDps = o.float("rateCapDps") ?: d.rateCapDps,
            kpAngle = o.float("kpAngle") ?: d.kpAngle,
            guidanceEnabled = o.bool("guidanceEnabled") ?: d.guidanceEnabled,
            pnNavGain = o.float("pnNavGain") ?: d.pnNavGain,
            pnMaxAccel = o.float("pnMaxAccel") ?: d.pnMaxAccel,
            pnAccelToFin = o.float("pnAccelToFin") ?: d.pnAccelToFin,
            pnMaxFinDeg = o.float("pnMaxFinDeg") ?: d.pnMaxFinDeg,
            pnMinSpeed = o.float("pnMinSpeed") ?: d.pnMinSpeed,
            pnCoastDelayMs = o.int("pnCoastDelayMs") ?: d.pnCoastDelayMs,
            pnTargetAltM = o.float("pnTargetAltM") ?: d.pnTargetAltM,
            pnTargetMode = o.int("pnTargetMode") ?: d.pnTargetMode,
            pnTargetE = o.float("pnTargetE") ?: d.pnTargetE,
            pnTargetN = o.float("pnTargetN") ?: d.pnTargetN,
            pnKpPos = o.float("pnKpPos") ?: d.pnKpPos,
            pnKdVel = o.float("pnKdVel") ?: d.pnKdVel,
            pnGuidanceLaw = o.int("pnGuidanceLaw") ?: d.pnGuidanceLaw,
            cameraType = o.int("cameraType") ?: d.cameraType,
            imuOrientSetting = o.int("imuOrientSetting") ?: d.imuOrientSetting,
            imuRateHz = o.int("imuRateHz") ?: d.imuRateHz,
            servoBias1 = o.int("servoBias1") ?: d.servoBias1,
            servoBias2 = o.int("servoBias2") ?: d.servoBias2,
            servoBias3 = o.int("servoBias3") ?: d.servoBias3,
            servoBias4 = o.int("servoBias4") ?: d.servoBias4,
            servoHz = o.int("servoHz") ?: d.servoHz,
            servoMinUs = servoMinUs,
            servoMaxUs = servoMaxUs,
            // #449: legacy finMinDeg/finMaxDeg keys deliberately NOT read.
            finTravelDeg = o.float("finTravelDeg")
                ?: RocketProfile.standardFinTravelDeg(servoMinUs, servoMaxUs),
            finRingMode = o.int("finRingMode") ?: d.finRingMode,
            finServoAtSlot = if (finSlots.size == 4) finSlots else d.finServoAtSlot,
            finReverse = if (finRev.size == 4) finRev else d.finReverse,
            finRollReverse = if (finRollRev.size == 4) finRollRev else d.finRollReverse,
            pidKp = o.float("pidKp") ?: d.pidKp,
            pidKi = o.float("pidKi") ?: d.pidKi,
            pidKd = o.float("pidKd") ?: d.pidKd,
            pidMinCmd = o.float("pidMinCmd") ?: d.pidMinCmd,
            pidMaxCmd = o.float("pidMaxCmd") ?: d.pidMaxCmd,
            integralSepThreshold = o.float("integralSepThreshold") ?: d.integralSepThreshold,
            rollWaypoints = (o["rollWaypoints"] as? JsonArray)?.mapNotNull { el ->
                val wp = el as? JsonObject ?: return@mapNotNull null
                ProfileRollWaypoint(
                    id = wp.str("id")?.let { runCatching { UUID.fromString(it) }.getOrNull() }
                        ?: UUID.randomUUID(),
                    timeSeconds = wp.float("timeSeconds") ?: return@mapNotNull null,
                    angleDeg = wp.float("angleDeg") ?: return@mapNotNull null,
                    mode = wp.int("mode") ?: 0,
                )
            } ?: d.rollWaypoints,
            pyro1Enabled = o.bool("pyro1Enabled") ?: d.pyro1Enabled,
            pyro1TriggerMode = o.int("pyro1TriggerMode") ?: d.pyro1TriggerMode,
            pyro1TriggerValue = o.float("pyro1TriggerValue") ?: d.pyro1TriggerValue,
            pyro2Enabled = o.bool("pyro2Enabled") ?: d.pyro2Enabled,
            pyro2TriggerMode = o.int("pyro2TriggerMode") ?: d.pyro2TriggerMode,
            pyro2TriggerValue = o.float("pyro2TriggerValue") ?: d.pyro2TriggerValue,
            pyro3Enabled = o.bool("pyro3Enabled") ?: d.pyro3Enabled,
            pyro3TriggerMode = o.int("pyro3TriggerMode") ?: d.pyro3TriggerMode,
            pyro3TriggerValue = o.float("pyro3TriggerValue") ?: d.pyro3TriggerValue,
            pyro4Enabled = o.bool("pyro4Enabled") ?: d.pyro4Enabled,
            pyro4TriggerMode = o.int("pyro4TriggerMode") ?: d.pyro4TriggerMode,
            pyro4TriggerValue = o.float("pyro4TriggerValue") ?: d.pyro4TriggerValue,
            drogueRateFps = o.dbl("drogueRateFps") ?: d.drogueRateFps,
            mainRateFps = o.dbl("mainRateFps") ?: d.mainRateFps,
            mainDeployAltAglFt = o.dbl("mainDeployAltAglFt") ?: d.mainDeployAltAglFt,
            ballisticDragK = o.dbl("ballisticDragK") ?: d.ballisticDragK,
            magCal = (o["magCal"] as? JsonObject)?.let { c ->
                MagCalData(
                    offsetX = c.int("offsetX") ?: return@let null,
                    offsetY = c.int("offsetY") ?: return@let null,
                    offsetZ = c.int("offsetZ") ?: return@let null,
                    fieldRuT = c.float("fieldR_uT") ?: 0f,
                    residualUT = c.float("residualUT") ?: 0f,
                    calibratedOnUnitID = c.str("calibratedOnUnitID") ?: "",
                    calibratedAtMs = c.dateMs("calibratedAt", nowMs),
                )
            },
            sensorCal = (o["sensorCal"] as? JsonObject)?.let { c ->
                SensorCalData(
                    gyroX = c.int("gyroX") ?: return@let null,
                    gyroY = c.int("gyroY") ?: return@let null,
                    gyroZ = c.int("gyroZ") ?: return@let null,
                    hgX = c.float("hgX") ?: 0f,
                    hgY = c.float("hgY") ?: 0f,
                    hgZ = c.float("hgZ") ?: 0f,
                    calibratedOnUnitID = c.str("calibratedOnUnitID") ?: "",
                    calibratedAtMs = c.dateMs("calibratedAt", nowMs),
                )
            },
        )
    }

    // JsonObject accessors (tolerant: wrong type → null → default).
    private fun JsonObject.str(k: String) =
        (this[k] as? JsonPrimitive)?.takeIf { it.isString }?.content

    private fun JsonObject.bool(k: String) =
        (this[k] as? JsonPrimitive)?.let { runCatching { it.boolean }.getOrNull() }

    private fun JsonObject.int(k: String) =
        (this[k] as? JsonPrimitive)?.let { runCatching { it.int }.getOrNull() }

    private fun JsonObject.float(k: String) =
        (this[k] as? JsonPrimitive)?.let { runCatching { it.float }.getOrNull() }

    private fun JsonObject.dbl(k: String) =
        (this[k] as? JsonPrimitive)?.let { runCatching { it.double }.getOrNull() }

    private fun JsonObject.intList(k: String) =
        (this[k] as? JsonArray)?.mapNotNull { (it as? JsonPrimitive)?.let { p -> runCatching { p.int }.getOrNull() } }

    private fun JsonObject.boolList(k: String) =
        (this[k] as? JsonArray)?.mapNotNull { (it as? JsonPrimitive)?.let { p -> runCatching { p.boolean }.getOrNull() } }
}
