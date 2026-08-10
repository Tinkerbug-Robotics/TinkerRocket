package com.tinkerbug.tinkerrocket.protocol

import kotlin.math.roundToLong

/**
 * Wire encoding for `BLE_BS_CMD_SET_PHONE_FIX` (47) — the phone's GPS fix,
 * pushed to the base station so its CSV records where the base station was.
 * Port of iOS `PhoneFixCodec.swift`.
 *
 * The base station has no GNSS: every lat/lon column in its log is the
 * ROCKET's relayed position.  The app computes and shows the range between
 * the two ends from this fix and then discards it, so the one number a range
 * test is about was never written down.  The 2026-08-08 Kaua'i test could
 * only be analysed because the operator remembered the site afterwards.
 *
 * Pure function, no BLE or location dependency, so the byte layout is
 * testable without a radio or a device.
 */
public object PhoneFixCodec {

    /** Payload length the firmware requires before it will parse. */
    public const val PAYLOAD_LENGTH: Int = 11

    /**
     * Horizontal accuracy sentinel.  A phone fix worse than 255 m is useless
     * for range, and clamping to the sentinel keeps it distinguishable from
     * a real reading instead of wrapping into a plausible one.
     */
    public const val ACCURACY_UNKNOWN: Int = 255

    /**
     * `[lat_e7 i32][lon_e7 i32][alt_m i16][h_acc_m u8]`, little-endian.
     *
     * 1e-7 degrees matches GNSSData's convention and resolves ~1 cm — far
     * finer than any phone fix, so the encoding never limits accuracy.
     * Every field clamps rather than wraps: a wrapped coordinate would read
     * as a valid position somewhere else on Earth, and be undetectable in
     * the log afterwards.
     *
     * @param accuracyM horizontal accuracy in metres; null or negative (the
     *   platform's "invalid fix" signal) encodes as [ACCURACY_UNKNOWN].
     */
    public fun encode(
        latDeg: Double,
        lonDeg: Double,
        altitudeM: Double?,
        accuracyM: Double?,
    ): ByteArray {
        val latE7 = clampToInt((latDeg * 1e7).roundToLong())
        val lonE7 = clampToInt((lonDeg * 1e7).roundToLong())
        val altM = clampToShort((altitudeM ?: 0.0).roundToLong())
        val accM =
            if (accuracyM == null || accuracyM < 0) ACCURACY_UNKNOWN
            else accuracyM.roundToLong().coerceIn(0L, 255L).toInt()

        val out = ByteArray(PAYLOAD_LENGTH)
        out[0] = (latE7 and 0xFF).toByte()
        out[1] = ((latE7 shr 8) and 0xFF).toByte()
        out[2] = ((latE7 shr 16) and 0xFF).toByte()
        out[3] = ((latE7 shr 24) and 0xFF).toByte()
        out[4] = (lonE7 and 0xFF).toByte()
        out[5] = ((lonE7 shr 8) and 0xFF).toByte()
        out[6] = ((lonE7 shr 16) and 0xFF).toByte()
        out[7] = ((lonE7 shr 24) and 0xFF).toByte()
        out[8] = (altM and 0xFF).toByte()
        out[9] = ((altM shr 8) and 0xFF).toByte()
        out[10] = accM.toByte()
        return out
    }

    /** Swift `Int32(clamping:)`. */
    private fun clampToInt(v: Long): Int =
        v.coerceIn(Int.MIN_VALUE.toLong(), Int.MAX_VALUE.toLong()).toInt()

    /** Swift `Int16(clamping:)`. */
    private fun clampToShort(v: Long): Int =
        v.coerceIn(Short.MIN_VALUE.toLong(), Short.MAX_VALUE.toLong()).toInt()
}
