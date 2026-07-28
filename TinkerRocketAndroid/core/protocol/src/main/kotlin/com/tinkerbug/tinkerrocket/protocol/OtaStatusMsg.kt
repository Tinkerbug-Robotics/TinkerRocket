package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.jsonObject

/**
 * OTA receiver-side status, parsed from `ota_status` JSON notifications on
 * the file-ops characteristic (#8 phase 2) — port of iOS `OTAStatusUpdate`
 * (OTAStatus.swift).  The firmware sends one on every state transition
 * (begin / verify_failed / ready_to_boot / aborted) plus rate-limited ~2 Hz
 * "writing" updates during the chunk pump.
 *
 * There is no progress math here (none on iOS either): progress = [bytes]
 * over the image size the OTA session already knows.
 */
public data class OtaStatusUpdate(
    val state: State,
    /** Cumulative bytes written so far (0 outside `writing`/`ready` states). */
    val bytes: Long,
    /**
     * Stable error token from the firmware (`sha_mismatch`, `bad_offset`,
     * `size_overflow`, `write_failed`, ...).  null on non-failure states.
     */
    val err: String?,
    /**
     * Version stamp of the image currently running on the device.  Present on
     * `ready_to_boot` (= the image about to take over after reboot — used as
     * a debug anchor while waiting for the post-reboot identity).
     */
    val fw: String?,
) {
    public enum class State(public val wire: String) {
        /** OTA_BEGIN accepted, awaiting chunks. */
        READY("ready"),

        /** Chunks landing; [OtaStatusUpdate.bytes] is cumulative. */
        WRITING("writing"),

        /** Finish OK; device about to esp_restart. */
        READY_TO_BOOT("ready_to_boot"),

        /** Terminal — see [OtaStatusUpdate.err]. */
        VERIFY_FAILED("verify_failed"),

        /** OTA_ABORT acknowledged. */
        ABORTED("aborted"),

        IDLE("idle"),
        UNKNOWN("unknown");

        public companion object {
            /** Unmapped wire strings → [UNKNOWN] (iOS `?? .unknown`). */
            public fun fromWire(raw: String): State =
                entries.firstOrNull { it.wire == raw } ?: UNKNOWN
        }
    }

    public companion object {
        /**
         * Mirror of iOS `OTAStatusUpdate.parse(_:)`: null on malformed JSON,
         * a non-object top level, or `"type" != "ota_status"`.  Fields are
         * individually tolerant: non-string `state` reads as "" → [State.UNKNOWN],
         * missing/non-integral `bytes` → 0.
         */
        public fun parse(bytes: ByteArray): OtaStatusUpdate? = parse(bytes.decodeToString())

        public fun parse(text: String): OtaStatusUpdate? {
            val json = try {
                Json.parseToJsonElement(text).jsonObject
            } catch (_: Exception) {
                return null
            }
            return parse(json)
        }

        public fun parse(json: JsonObject): OtaStatusUpdate? {
            if (JsonBridging.nsString(json, "type") != "ota_status") return null
            return OtaStatusUpdate(
                state = State.fromWire(JsonBridging.nsString(json, "state") ?: ""),
                bytes = JsonBridging.nsInt(json, "bytes") ?: 0L,
                err = JsonBridging.nsString(json, "err"),
                fw = JsonBridging.nsString(json, "fw"),
            )
        }
    }
}
