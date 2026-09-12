package com.tinkerbug.tinkerrocket.session

import kotlin.math.abs
import kotlin.math.sqrt

/**
 * iOS OnPadCalibrationView's post-cal sanity check (#1059): a rocket that is
 * sitting still reads 1 g on the low-g accelerometer, and more than 3 % off
 * right after a calibration is worth a warning.  A warning, not a refusal —
 * the calibration is already saved on the rocket and in the profile; the
 * operator decides whether to run a bench calibration before flying.
 */
public object SensorCalGravity {
    public const val G_MPS2: Float = 9.80665f
    public const val WARN_PCT: Float = 3.0f

    /** The low-g magnitude that was read and how far it is from 1 g, in percent. */
    public data class Warning(val magnitudeMps2: Float, val errorPct: Float)

    /**
     * Null when the reading is within [WARN_PCT] of 1 g — or when there is no
     * reading to judge: a magnitude at or under 0.1 m/s² is missing accel data,
     * not a rocket in free fall (the iOS `mag > 0.1` guard).
     */
    public fun check(lowGX: Float?, lowGY: Float?, lowGZ: Float?): Warning? {
        val x = lowGX ?: 0f
        val y = lowGY ?: 0f
        val z = lowGZ ?: 0f
        val mag = sqrt(x * x + y * y + z * z)
        if (mag <= 0.1f) return null
        val errPct = abs(mag - G_MPS2) / G_MPS2 * 100f
        return if (errPct > WARN_PCT) Warning(mag, errPct) else null
    }
}
