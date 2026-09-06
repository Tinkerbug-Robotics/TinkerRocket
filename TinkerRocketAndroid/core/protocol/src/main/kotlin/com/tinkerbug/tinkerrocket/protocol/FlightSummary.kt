package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonElement
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import java.math.BigDecimal
import java.math.RoundingMode
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.log10
import kotlin.math.pow

/*
 * Flight summary sidecar — port of the FlightSummary / FlightSettings structs
 * in iOS CSVGenerator.swift (#165, #196, #142).
 *
 * JSON parity contract: iOS writes the sidecar with a synthesized-Codable
 * JSONEncoder (.prettyPrinted, .sortedKeys).  Synthesized Codable encodes
 * optional properties with `encodeIfPresent`, which OMITS the key entirely
 * when the value is nil (it does NOT emit an explicit `null`) — despite what
 * some iOS-side comments say ("JSON emits null"); the CODE's behavior is the
 * reference.  [toJson] therefore omits absent keys and sorts keys at every
 * nesting level.  Semantic key/value parity is guaranteed; byte-for-byte
 * formatting (indent width, `5` vs `5.0` for whole doubles) is not.
 */

/**
 * Round a float32 to [digits] significant figures so the JSON shows clean
 * gains (e.g. 0.04, not 0.039999999105930) instead of float32→Double noise.
 * Port of iOS `sigFig` — Swift `.rounded()` is ties-away-from-zero on the
 * exact binary value, reproduced here via BigDecimal HALF_UP (Kotlin's
 * `kotlin.math.round` is rint/half-even and would diverge on exact ties).
 */
internal fun sigFig(v: Float, digits: Int = 6): Double {
    val d = v.toDouble()
    if (d == 0.0 || !d.isFinite()) return d
    val mag = floor(log10(abs(d)))
    val factor = 10.0.pow((digits - 1) - mag)
    return roundAwayFromZero(d * factor) / factor
}

private fun roundAwayFromZero(x: Double): Double =
    BigDecimal(x).setScale(0, RoundingMode.HALF_UP).toDouble()

/** Sorted-key, absent-key-omitting object builder (mirrors iOS .sortedKeys + encodeIfPresent). */
private fun jsonObj(vararg entries: Pair<String, JsonElement?>): JsonObject =
    JsonObject(
        entries.mapNotNull { (k, v) -> v?.let { k to it } }
            .sortedBy { it.first }
            .toMap(),
    )

private val prettyJson = Json {
    prettyPrint = true
    prettyPrintIndent = "  "
}

/**
 * Per-flight summary sidecar (`.json` next to the CSV).
 * Kotlin properties are camelCase; the wire/JSON keys are the exact iOS
 * Codable names (snake_case), emitted by [toJson].
 */
public data class FlightSummary(
    /** Maximum pressure altitude above ground level (meters). */
    val maxAltitudeM: Double?,
    /** Maximum 3D speed from EKF velocity estimate (m/s). */
    val maxSpeedMps: Double?,
    /** Time from launch to motor burnout (seconds) — first NSF_BURNOUT latch (#196). */
    val burnoutTimeS: Double?,
    /** Time from launch to apogee (seconds) — first master voted apogee latch (#142). */
    val apogeeTimeS: Double?,
    /**
     * Runtime roll-control / IMU settings the FC flew with, snapshotted into
     * the log at launch (#165). null for flights logged before the firmware
     * emitted the settings frame.
     */
    val settings: FlightSettings?,
) {
    public fun toJsonObject(): JsonObject = jsonObj(
        "max_altitude_m" to maxAltitudeM?.let(::JsonPrimitive),
        "max_speed_mps" to maxSpeedMps?.let(::JsonPrimitive),
        "burnout_time_s" to burnoutTimeS?.let(::JsonPrimitive),
        "apogee_time_s" to apogeeTimeS?.let(::JsonPrimitive),
        "settings" to settings?.toJsonObject(),
    )

    /** Pretty-printed sidecar JSON — same keys/values as iOS `writeSummary`. */
    public fun toJson(): String =
        prettyJson.encodeToString(JsonObject.serializer(), toJsonObject())
}

// MARK: - Flight Settings (#165)

/**
 * Settings block written into the per-flight summary JSON. Mirrors every
 * per-rocket setting editable in the app's settings UI (#165) plus the
 * issue's IMU full-scale and outer-loop knobs.
 */
public data class FlightSettings(
    val fwGitSha: String,
    val fwDirty: Boolean,
    val soundsEnabled: Boolean,
    val rollControl: RollControlSettings,
    val servo: ServoSettings,
    val camera: CameraSettings,
    val pyro: PyroSettings,
    val imu: ImuSettings,
) {
    public fun toJsonObject(): JsonObject = jsonObj(
        "fw_git_sha" to JsonPrimitive(fwGitSha),
        "fw_dirty" to JsonPrimitive(fwDirty),
        "sounds_enabled" to JsonPrimitive(soundsEnabled),
        "roll_control" to rollControl.toJsonObject(),
        "servo" to servo.toJsonObject(),
        "camera" to camera.toJsonObject(),
        "pyro" to pyro.toJsonObject(),
        "imu" to imu.toJsonObject(),
    )

    public companion object {
        public fun from(raw: FlightSettingsData): FlightSettings = FlightSettings(
            fwGitSha = raw.fwGitSha,
            fwDirty = raw.fwDirty,
            soundsEnabled = raw.soundsEnabled,
            rollControl = RollControlSettings.from(raw),
            servo = ServoSettings.from(raw),
            camera = CameraSettings(type = CameraSettings.label(raw.cameraType)),
            pyro = PyroSettings.from(raw),
            imu = ImuSettings(
                gyroFsDps = raw.ism6GyroFsDps,
                lowGFsG = raw.ism6LowGFsG,
                highGFsG = raw.ism6HighGFsG,
                updateRateHz = raw.ism6UpdateRateHz,
                dynamicRate = raw.imuRateDynamic,
                mounting = MountingSettings.from(raw),
            ),
        )
    }
}

public data class RollControlSettings(
    /**
     * "rate" (null-rate inner loop), "angle" (cascaded, single setpoint),
     * or "angle_profile" (cascaded, follows the waypoint profile).
     */
    val mode: String,
    val kp: Double,
    val ki: Double,
    val kd: Double,
    val dLpfHz: Double,
    val kpAngle: Double,
    val cmdLimitMinDeg: Double,
    val cmdLimitMaxDeg: Double,
    val delayMs: Int,
    /**
     * Control-authority speed gate (m/s) that flew; null on pre-v8 firmware,
     * which had no speed gate.  Control waited for the delay AND this speed.
     */
    val minSpeedMps: Double?,
    val rateCapDps: Double,
    val rollRateSetPoint: Double,
    val guidanceEnabled: Boolean,
    val gainSchedule: GainScheduleSettings,
    /** "ramp" (fw v4+: lerp between waypoints) or "step" (pre-v4). */
    val profileSemantics: String,
    val profile: List<RollWaypointJson>,
) {
    public fun toJsonObject(): JsonObject = jsonObj(
        "mode" to JsonPrimitive(mode),
        "kp" to JsonPrimitive(kp),
        "ki" to JsonPrimitive(ki),
        "kd" to JsonPrimitive(kd),
        "d_lpf_hz" to JsonPrimitive(dLpfHz),
        "kp_angle" to JsonPrimitive(kpAngle),
        "cmd_limit_min_deg" to JsonPrimitive(cmdLimitMinDeg),
        "cmd_limit_max_deg" to JsonPrimitive(cmdLimitMaxDeg),
        "delay_ms" to JsonPrimitive(delayMs),
        "min_speed_mps" to minSpeedMps?.let { JsonPrimitive(it) },
        "rate_cap_dps" to JsonPrimitive(rateCapDps),
        "roll_rate_set_point" to JsonPrimitive(rollRateSetPoint),
        "guidance_enabled" to JsonPrimitive(guidanceEnabled),
        "gain_schedule" to gainSchedule.toJsonObject(),
        "profile_semantics" to JsonPrimitive(profileSemantics),
        "profile" to JsonArray(profile.map { it.toJsonObject() }),
    )

    public companion object {
        public fun from(raw: FlightSettingsData): RollControlSettings {
            // The firmware only runs the angle cascade when use_angle_control
            // is set (Null Roll / Track Profile toggle) — a stored waypoint
            // profile alone is inert, so it must not decide the exported mode.
            val mode = when {
                !raw.useAngleControl -> "rate"
                raw.numWaypoints > 0 -> "angle_profile"
                else -> "angle"
            }
            return RollControlSettings(
                mode = mode,
                kp = sigFig(raw.kp),
                ki = sigFig(raw.ki),
                kd = sigFig(raw.kd),
                dLpfHz = sigFig(raw.dLpfHz),
                kpAngle = sigFig(raw.kpAngle),
                cmdLimitMinDeg = sigFig(raw.minCmdDeg),
                cmdLimitMaxDeg = sigFig(raw.maxCmdDeg),
                delayMs = raw.rollDelayMs,
                minSpeedMps = raw.rollMinSpeedMps?.let { sigFig(it) },
                rateCapDps = sigFig(raw.kpAngleRateCapDps),
                rollRateSetPoint = sigFig(raw.rollRateSetPoint),
                guidanceEnabled = raw.guidanceEnabled,
                gainSchedule = GainScheduleSettings(
                    enabled = raw.gainScheduleEnabled,
                    vRef = sigFig(raw.gsVRef),
                    vMin = sigFig(raw.gsVMin),
                    scaleCap = sigFig(raw.gsScaleCap),
                ),
                // v4+ firmware ramps the target linearly between waypoints and
                // ignores the legacy per-waypoint mode bytes; pre-v4 stepped to
                // the NEXT waypoint's angle honoring per-waypoint null_rate
                // modes.  Post-flight analysis branches on this marker, so it
                // must reflect the FIRMWARE version that flew.
                profileSemantics = if (raw.version >= 4) "ramp" else "step",
                profile = raw.waypoints.map {
                    RollWaypointJson(
                        timeS = sigFig(it.timeS),
                        angleDeg = sigFig(it.angleDeg),
                        mode = if (raw.version >= 4) {
                            null
                        } else {
                            if (it.mode == 1) "null_rate" else "angle"
                        },
                    )
                },
            )
        }
    }
}

public data class GainScheduleSettings(
    val enabled: Boolean,
    val vRef: Double,
    val vMin: Double,
    val scaleCap: Double,
) {
    public fun toJsonObject(): JsonObject = jsonObj(
        "enabled" to JsonPrimitive(enabled),
        "v_ref" to JsonPrimitive(vRef),
        "v_min" to JsonPrimitive(vMin),
        "scale_cap" to JsonPrimitive(scaleCap),
    )
}

public data class RollWaypointJson(
    val timeS: Double,
    val angleDeg: Double,
    /** pre-v4 only: "angle" or "null_rate"; key omitted on ramp-semantics exports. */
    val mode: String?,
) {
    public fun toJsonObject(): JsonObject = jsonObj(
        "time_s" to JsonPrimitive(timeS),
        "angle_deg" to JsonPrimitive(angleDeg),
        "mode" to mode?.let(::JsonPrimitive),
    )
}

public data class ServoSettings(
    val enabled: Boolean,
    val biasUs: List<Int>,
    val frequencyHz: Int,
    val minPulseUs: Int,
    val maxPulseUs: Int,
    /** #267: physical fin angle at min_pulse_us (null pre-v3). */
    val finMinDeg: Double?,
    /** #267: physical fin angle at max_pulse_us (null pre-v3). */
    val finMaxDeg: Double?,
) {
    public fun toJsonObject(): JsonObject = jsonObj(
        "enabled" to JsonPrimitive(enabled),
        "bias_us" to JsonArray(biasUs.map(::JsonPrimitive)),
        "frequency_hz" to JsonPrimitive(frequencyHz),
        "min_pulse_us" to JsonPrimitive(minPulseUs),
        "max_pulse_us" to JsonPrimitive(maxPulseUs),
        "fin_min_deg" to finMinDeg?.let(::JsonPrimitive),
        "fin_max_deg" to finMaxDeg?.let(::JsonPrimitive),
    )

    public companion object {
        public fun from(raw: FlightSettingsData): ServoSettings = ServoSettings(
            enabled = raw.servoEnabled,
            biasUs = raw.servoBiasUs.toList(),
            frequencyHz = raw.servoHz,
            minPulseUs = raw.servoMinUs,
            maxPulseUs = raw.servoMaxUs,
            finMinDeg = raw.finMinDeg?.let { sigFig(it) },
            finMaxDeg = raw.finMaxDeg?.let { sigFig(it) },
        )
    }
}

public data class CameraSettings(
    /** "none" | "gopro" | "runcam". */
    val type: String,
) {
    public fun toJsonObject(): JsonObject = jsonObj(
        "type" to JsonPrimitive(type),
    )

    public companion object {
        public fun label(t: Int): String = when (t) {
            1 -> "gopro"
            2 -> "runcam"
            else -> "none"
        }
    }
}

public data class PyroSettings(
    val ch1: PyroChannelSettings,
    val ch2: PyroChannelSettings,
    val ch3: PyroChannelSettings,
    val ch4: PyroChannelSettings,
) {
    public fun toJsonObject(): JsonObject = jsonObj(
        "ch1" to ch1.toJsonObject(),
        "ch2" to ch2.toJsonObject(),
        "ch3" to ch3.toJsonObject(),
        "ch4" to ch4.toJsonObject(),
    )

    public companion object {
        public fun from(raw: FlightSettingsData): PyroSettings {
            fun ch(i: Int) = PyroChannelSettings.from(
                enabled = raw.pyroEnabled[i],
                mode = raw.pyroTriggerMode[i],
                value = raw.pyroTriggerValue[i],
            )
            return PyroSettings(ch1 = ch(0), ch2 = ch(1), ch3 = ch(2), ch4 = ch(3))
        }
    }
}

public data class PyroChannelSettings(
    val enabled: Boolean,
    /** "time_after_apogee" (value = seconds) or "altitude_on_descent" (value = meters AGL). */
    val triggerMode: String,
    val triggerValue: Double,
) {
    public fun toJsonObject(): JsonObject = jsonObj(
        "enabled" to JsonPrimitive(enabled),
        "trigger_mode" to JsonPrimitive(triggerMode),
        "trigger_value" to JsonPrimitive(triggerValue),
    )

    public companion object {
        public fun from(enabled: Boolean, mode: Int, value: Float): PyroChannelSettings =
            PyroChannelSettings(
                enabled = enabled,
                triggerMode = if (mode == 1) "altitude_on_descent" else "time_after_apogee",
                triggerValue = sigFig(value),
            )
    }
}

public data class ImuSettings(
    val gyroFsDps: Int,
    val lowGFsG: Int,
    val highGFsG: Int,
    /**
     * Logged IMU sample rate in Hz (v5 settings frames). null on older logs,
     * which ran the then-fixed build default.
     */
    val updateRateHz: Int?,
    /**
     * The rocket flew the dynamic logging rate: [updateRateHz] through
     * boost/coast, stepping down at the first "Deployed Flag" row.
     */
    val dynamicRate: Boolean,
    /**
     * Board→rocket mounting orientation (v2 settings frames). null on
     * pre-orientation logs, which always meant the +X-nose mounting.
     */
    val mounting: MountingSettings?,
) {
    public fun toJsonObject(): JsonObject = jsonObj(
        "gyro_fs_dps" to JsonPrimitive(gyroFsDps),
        "low_g_fs_g" to JsonPrimitive(lowGFsG),
        "high_g_fs_g" to JsonPrimitive(highGFsG),
        "update_rate_hz" to updateRateHz?.let(::JsonPrimitive),
        "dynamic_rate" to JsonPrimitive(dynamicRate),
        "mounting" to mounting?.toJsonObject(),
    )
}

public data class MountingSettings(
    /** Which board axis pointed at the nose + clocking, e.g. "+X", "-Z r90". */
    val orientation: String,
    /** How it was determined: "default" | "manual" | "auto_snap" | "auto_exact". */
    val mode: String,
    /** Auto-snap residual angle (deg); 0 for default/manual. */
    val residualDeg: Double,
) {
    public fun toJsonObject(): JsonObject = jsonObj(
        "orientation" to JsonPrimitive(orientation),
        "mode" to JsonPrimitive(mode),
        "residual_deg" to JsonPrimitive(residualDeg),
    )

    public companion object {
        /** null unless the frame carries the v2 orientation tail (mirrors iOS failable init). */
        public fun from(raw: FlightSettingsData): MountingSettings? {
            val code = raw.b2rCode ?: return null
            val mode = raw.b2rMode ?: return null
            return MountingSettings(
                orientation = FlightSettingsData.b2rName(code),
                mode = when (mode) {
                    1 -> "manual"
                    2 -> "auto_snap"
                    3 -> "auto_exact"
                    else -> "default"
                },
                residualDeg = sigFig(raw.b2rResidualDeg ?: 0f, 3),
            )
        }
    }
}
