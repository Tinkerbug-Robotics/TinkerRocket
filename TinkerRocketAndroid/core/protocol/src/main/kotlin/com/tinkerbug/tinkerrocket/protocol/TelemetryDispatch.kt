package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject

/**
 * Demux for the TELEMETRY characteristic — every notification is JSON, and
 * the message kind rides the top-level `"type"` key.  A line-for-line port of
 * the ladder at the top of iOS `BLEDevice.parseTelemetryData(_:)`, in the
 * same order: config → config_pyro → config_identity → fc_identity →
 * imu_orient → guid_target → plain telemetry.
 *
 * Pinned iOS oddities:
 *  - a frame with NO `"type"` key is plain telemetry (the common case — the
 *    firmware only stamps `type` on the readback messages);
 *  - a frame with an UNRECOGNIZED (or non-string) `"type"` also falls
 *    through to the telemetry decoder — and since [TelemetryData] ignores
 *    unknown keys, `{"type":"bogus","st":"READY"}` decodes as a REAL
 *    telemetry frame on iOS.  Mirrored: it returns [TelemetryCharMessage.Telemetry],
 *    NOT [TelemetryCharMessage.Unknown];
 *  - only when the telemetry decode itself fails (malformed JSON, non-object
 *    top level, or a strict-field type mismatch — see TelemetryData) is the
 *    frame dropped; that terminal state is surfaced as [TelemetryCharMessage.Unknown]
 *    (iOS just logs and discards — the session layer must do nothing with it).
 */
public object TelemetryDispatch {

    public fun parse(bytes: ByteArray): TelemetryCharMessage = parse(bytes.decodeToString())

    public fun parse(text: String): TelemetryCharMessage {
        val json = try {
            Json.parseToJsonElement(text) as? JsonObject
        } catch (_: Exception) {
            null
        } ?: return TelemetryCharMessage.Unknown(type = null)

        return when (val type = JsonBridging.nsString(json, "type")) {
            "config" -> TelemetryCharMessage.Config(ConfigMessage.parse(json))
            "config_pyro" -> TelemetryCharMessage.ConfigPyro(ConfigPyroMessage.parse(json))
            "config_identity" -> TelemetryCharMessage.ConfigIdentity(ConfigIdentityMsg.parse(json))
            "fc_identity" -> TelemetryCharMessage.FcIdentity(FcIdentityMsg.parse(json))
            "imu_orient" -> TelemetryCharMessage.ImuOrient(ImuOrientMsg.parse(json))
            "guid_target" ->
                // The type gate inside fromJson() is already satisfied here.
                TelemetryCharMessage.GuidTarget(GuidanceTargetEcho.fromJson(json)!!)
            else -> {
                // Absent, non-string, or unrecognized type → telemetry decode,
                // exactly like the iOS fall-through.
                val telemetry = TelemetryData.decode(json)
                if (telemetry != null) {
                    TelemetryCharMessage.Telemetry(telemetry)
                } else {
                    TelemetryCharMessage.Unknown(type)
                }
            }
        }
    }
}

/** One decoded telemetry-characteristic notification. */
public sealed interface TelemetryCharMessage {

    /**
     * A live telemetry frame.  Session-layer reminders from the iOS handler:
     * the first decoded frame flips the #377 power-state-confirmed gate, and
     * a frame with `sourceRocketId > 0` on a base-station link routes to the
     * relayed-rocket roster (#390) instead of the device's own mirror.
     */
    public data class Telemetry(val data: TelemetryData) : TelemetryCharMessage

    /**
     * `"type":"config"` readback — fold with
     * [ConfigMessage.applyTo]`(currentConfig)` (pyro fields preserved, all
     * other absent keys reset to defaults, #253 sentinels honored).
     */
    public data class Config(val msg: ConfigMessage) : TelemetryCharMessage

    /**
     * `"type":"config_pyro"` readback — fold with
     * [ConfigPyroMessage.applyTo]`(currentConfig)` (edits only pyro fields).
     */
    public data class ConfigPyro(val msg: ConfigPyroMessage) : TelemetryCharMessage

    /** `"type":"config_identity"` — device identity readback. */
    public data class ConfigIdentity(val msg: ConfigIdentityMsg) : TelemetryCharMessage

    /** `"type":"fc_identity"` — the FC's own firmware version (OC-relayed). */
    public data class FcIdentity(val msg: FcIdentityMsg) : TelemetryCharMessage

    /** `"type":"imu_orient"` — active board→rocket mounting orientation. */
    public data class ImuOrient(val msg: ImuOrientMsg) : TelemetryCharMessage

    /** `"type":"guid_target"` — #435 guidance aim-point echo. */
    public data class GuidTarget(val echo: GuidanceTargetEcho) : TelemetryCharMessage

    /**
     * Undecodable frame: malformed JSON, non-object top level, or a
     * strict-field telemetry type mismatch.  [type] is the frame's `"type"`
     * string when one was present (never one of the known readback types).
     * iOS logs and discards these — do nothing with them.
     */
    public data class Unknown(val type: String?) : TelemetryCharMessage
}
