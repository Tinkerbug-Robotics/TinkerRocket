package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.JsonObject

/**
 * Small identity/status JSON frames on the telemetry characteristic —
 * `config_identity`, `fc_identity`, and `imu_orient`.  Behavioral reference:
 * the corresponding ladder branches in iOS BLEDevice.parseTelemetryData.
 * (The fourth small frame, the #435 `guid_target` echo, lives in
 * GuidanceSendFlow.kt with its confirmation state machine.)
 */

/**
 * `"type":"config_identity"` — device identity readback.  Every field is
 * individually optional; the iOS handler applies only present keys onto the
 * device state, so null here means "leave the previous value alone".
 */
public data class ConfigIdentityMsg(
    /** Immutable hardware ID, e.g. "a1b2c3d4". */
    val unitId: String?,          // "uid"
    /** User-settable unit name, e.g. "Atlas" (20-BYTE clamp on the set path). */
    val unitName: String?,        // "un"
    /** Wire u8; clamped 0..255 (iOS `UInt8(...)` traps out-of-range). */
    val networkId: Int?,          // "nid"
    /** Wire u8; clamped 0..255. */
    val rocketId: Int?,           // "rid"
    /**
     * Device-type letter, raw: "R" rocket / "B" base station.  Kept as the
     * wire string — the iOS `BLEDeviceType(rawValue:)` mapping (unknown →
     * keep previous) belongs to the device layer that owns the enum.
     */
    val deviceType: String?,      // "dt"
    /**
     * Firmware version stamp (#8): `<git_short_sha>+<build_yyyymmdd-hhmm>`,
     * optionally with `-dirty` before the `+`.  This is the CONNECTED
     * device's version (the OC's, on a rocket link).
     */
    val firmwareVersion: String?, // "fw"
) {
    public companion object {
        public fun parse(json: JsonObject): ConfigIdentityMsg = ConfigIdentityMsg(
            unitId = JsonBridging.nsString(json, "uid"),
            unitName = JsonBridging.nsString(json, "un"),
            networkId = JsonBridging.nsInt(json, "nid")?.coerceIn(0, 0xFF)?.toInt(),
            rocketId = JsonBridging.nsInt(json, "rid")?.coerceIn(0, 0xFF)?.toInt(),
            deviceType = JsonBridging.nsString(json, "dt"),
            firmwareVersion = JsonBridging.nsString(json, "fw"),
        )
    }
}

/**
 * `"type":"fc_identity"` — the Flight Computer's OWN firmware version,
 * relayed by the OUT computer over I2C (#8 Phase 4).  Same stamp format as
 * [ConfigIdentityMsg.firmwareVersion]; "unknown" when the OC can't reach the
 * FC.
 *
 * IMPORTANT: for an FC-targeted OTA, rollback detection MUST compare THIS
 * value — the connected device's `config_identity` "fw" is the OC's and
 * never changes when only the FC is updated.
 */
public data class FcIdentityMsg(
    val fcFirmwareVersion: String?,   // "fc_fw"
) {
    public companion object {
        public fun parse(json: JsonObject): FcIdentityMsg =
            FcIdentityMsg(fcFirmwareVersion = JsonBridging.nsString(json, "fc_fw"))
    }
}

/**
 * `"type":"imu_orient"` — the FC's active board→rocket mounting orientation
 * (which board axis maps to the rocket nose and how it was decided), relayed
 * by the OC and re-sent whenever the FC re-orients on the pad.
 */
public data class ImuOrientMsg(
    /** Display name like "+X" or "-Z r90"; null = key absent. */
    val name: String?,            // "name"
    /** Raw mode int; null = key absent (leave previous mode alone). */
    val modeRaw: Int?,            // "mode"
    /**
     * The user's setting readback: 0xFF auto / 0..23 manual; clamped 0..255
     * (iOS `UInt8(clamping:)`).  Cached into [RocketConfig.imuOrientSetting]
     * by the session layer — iOS only stores it when a config already exists.
     */
    val setting: Int?,            // "set"
) {
    /**
     * Typed mode: null when the key was absent; unknown raw values map to
     * [IMUOrientationMode.UNKNOWN] (iOS `?? .unknown`).
     */
    val mode: IMUOrientationMode?
        get() = modeRaw?.let { raw ->
            IMUOrientationMode.entries.firstOrNull { it.raw == raw } ?: IMUOrientationMode.UNKNOWN
        }

    public companion object {
        public fun parse(json: JsonObject): ImuOrientMsg = ImuOrientMsg(
            name = JsonBridging.nsString(json, "name"),
            modeRaw = JsonBridging.nsInt(json, "mode")
                ?.coerceIn(Int.MIN_VALUE.toLong(), Int.MAX_VALUE.toLong())?.toInt(),
            setting = JsonBridging.nsInt(json, "set")?.coerceIn(0, 0xFF)?.toInt(),
        )
    }
}
