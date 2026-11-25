package com.tinkerbug.tinkerrocket.session

/**
 * #1629: what one flight computer holds for calibration, as last read from
 * the board itself — never from a rocket profile, which describes what the
 * app saved, not what the board flies with.  iOS twin: `BoardCalibration`
 * (Models/BoardCalibration.swift); the two must say the same things.
 *
 * Each answer is null until the board has said it: "not read yet" must never
 * render as "not calibrated".  The flight computer answers three read-only
 * commands from its own storage, and republishes the same frames after every
 * calibration it stores, so the record tracks the board:
 * - [mag]: BLE 61 → 0xCA, APPLIED = a hard-iron calibration is stored
 *   (NVS "mag_cal");
 * - [gyro]: BLE 63 → 0xCB, `valid` = a pad calibration's gyro bias is stored
 *   (NVS "cal");
 * - [accel] and [highGAligned]: BLE 77 → 0xD1, the six-face offset is stored
 *   (NVS "acc_cal") and the high-g's six-face alignment is stored (NVS
 *   "hg_cal").
 *
 * The high-g alignment is a refinement: the high-g flies on its bias without
 * one, so it is shown in Settings but never counted as "not calibrated".
 */
public data class BoardCalibration(
    val mag: Boolean? = null,
    val accel: Boolean? = null,
    val gyro: Boolean? = null,
    val highGAligned: Boolean? = null,
    /** Epoch millis of the last answer folded in, or null when none. */
    val readAtMs: Long? = null,
) {
    /** The calibrations a flight needs, in the order they are named. */
    public enum class Item(public val label: String) {
        MAGNETOMETER("magnetometer"),
        ACCELEROMETER("accelerometer"),
        GYRO("gyro"),
    }

    /** The needed calibrations the board said it does not have. */
    public val missing: List<Item>
        get() = buildList {
            if (mag == false) add(Item.MAGNETOMETER)
            if (accel == false) add(Item.ACCELEROMETER)
            if (gyro == false) add(Item.GYRO)
        }

    /** Every needed calibration is confirmed present. */
    public val fullyCalibrated: Boolean get() = mag == true && accel == true && gyro == true

    /** Nothing has been read from this board yet. */
    public val nothingRead: Boolean
        get() = mag == null && accel == null && gyro == null && highGAligned == null

    /** The dashboard line, or null when nothing is known to be missing. */
    public val notCalibratedLine: String?
        get() = missing.takeIf { it.isNotEmpty() }
            ?.joinToString(", ", prefix = "Not calibrated: ") { it.label }

    /** One line for My Devices. */
    public val summary: String
        get() = when {
            missing.isNotEmpty() -> notCalibratedLine!!
            fullyCalibrated -> "Calibrated"
            nothingRead -> "Calibration not read yet"
            else -> "Calibration not fully read"
        }

    /**
     * Fold a newer reading in: each reply answers one or two of the fields,
     * so only the fields it answers change, and the time is the newer one.
     */
    public fun merged(newer: BoardCalibration): BoardCalibration = BoardCalibration(
        mag = newer.mag ?: mag,
        accel = newer.accel ?: accel,
        gyro = newer.gyro ?: gyro,
        highGAligned = newer.highGAligned ?: highGAligned,
        readAtMs = newer.readAtMs ?: readAtMs,
    )

    public companion object {
        /** A Settings row's value for a needed calibration. */
        public fun rowText(value: Boolean?): String = when (value) {
            true -> "Calibrated"
            false -> "Not calibrated"
            null -> "Not read yet"
        }

        /** The Settings row's value for the high-g alignment. */
        public fun highGRowText(value: Boolean?): String = when (value) {
            true -> "Aligned"
            false -> "Not aligned (flies on its bias)"
            null -> "Not read yet"
        }
    }
}
