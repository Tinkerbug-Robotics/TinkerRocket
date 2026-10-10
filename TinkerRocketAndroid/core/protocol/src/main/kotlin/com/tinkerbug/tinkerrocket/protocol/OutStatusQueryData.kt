package com.tinkerbug.tinkerrocket.protocol

import kotlin.math.abs

/**
 * OutStatusQueryData — msg 0xA0, sensor config from the FlightComputer.
 *
 * Deliberately a SUBSET of the wire struct: the iOS app decodes only the
 * scale/rotation/version fields, and this port pins iOS behavior.  The
 * hg-bias, b2r block, and v5 guidance-target tail present on the wire are
 * NOT decoded here (iOS never reads them from this frame).
 *
 * v4 (#204): per-chip IIS2MDC rotation appended after the b2r block, at
 * fixed byte offset 26 = ism6(1+2+2) + mmc(2) + ver(1) + hg_bias(6) +
 * b2r(2+8).  Dual-gated: format_version >= 4 AND payload >= 28 bytes.
 *
 * v6: mag_type byte at fixed offset 41 (after the v5 guidance-target echo,
 * which stays undecoded here) — which chip is behind the IIS2MDC-named
 * (0xD1) count stream, keying the count→µT scale.  Same dual gate shape:
 * format_version >= 6 AND payload >= 42.
 */
public data class OutStatusQueryData(
    val ism6LowGFsG: Int,        // u8, e.g. 16
    val ism6HighGFsG: Int,       // u16, e.g. 256
    val ism6GyroFsDps: Int,      // u16, e.g. 4000
    val ism6RotZCdeg: Int,       // i16, centi-degrees
    val mmcRotZCdeg: Int,        // i16, centi-degrees
    val formatVersion: Int,      // u8
    val iis2mdcRotZCdeg: Int?,   // i16, centi-degrees; null pre-v4 (#204)
    val magType: Int? = null,    // u8, MAG_TYPE_*; null pre-v6
) {
    /** IMU rotation in degrees. */
    public val imuRotationDeg: Double get() = ism6RotZCdeg / 100.0

    /** Magnetometer (MMC5983MA) rotation in degrees. */
    public val magRotationDeg: Double get() = mmcRotZCdeg / 100.0

    /** IIS2MDC rotation in degrees (format_version >= 4); null on older logs. */
    public val iisRotationDeg: Double? get() = iis2mdcRotZCdeg?.let { it / 100.0 }

    /**
     * Count→µT scale of the IIS2MDC-named mag stream, keyed off [magType].
     * Pre-v6 logs (null) and unknown values fall back to the big board's
     * IIS2MDC — every pre-v6 log came from one.
     */
    public val magUtPerLsb: Double
        get() = if (magType == MAG_TYPE_QMC5883P) QMC5883P_UT_PER_LSB else IIS2MDC_UT_PER_LSB

    /**
     * Per-axis signs carrying the chip's own axes into its normalized frame
     * (right-handed, Z out of the top) before the Z rotation —
     * magTypeChipSign in RocketComputerTypes.h.  IIS2MDC (-1, +1, +1): its
     * axes are left-handed (#1589).  QMC5883P (+1, -1, -1): as configured its
     * Z points into the board (#1590).  Pre-v6 logs (null) and unknown values
     * are the IIS2MDC, as for the scale.
     */
    public val magChipSigns: Triple<Double, Double, Double>
        get() = if (magType == MAG_TYPE_QMC5883P) Triple(1.0, -1.0, -1.0) else Triple(-1.0, 1.0, 1.0)

    /**
     * The I2C-mag rotation to apply to this log's counts: [iisRotationDeg],
     * except that a QMC5883P log stamped +90 is a pre-#1590 Beetle log (its
     * firmware applied the IIS2MDC's angle; no QMC5883P board sits at +90 in
     * the normalized frame) and is read at the Beetle's -90, as
     * plot_flight_data_mini.py does.
     */
    public val iisRotationDegApplied: Double?
        get() {
            val deg = iisRotationDeg ?: return null
            return if (magType == MAG_TYPE_QMC5883P && abs(deg - QMC5883P_PRE_1590_STAMP_DEG) < 1e-6) {
                QMC5883P_BEETLE_ROT_Z_DEG
            } else {
                deg
            }
        }

    public companion object {
        public const val MIN_SIZE: Int = 10
        /** Fixed offset of the v4 IIS2MDC rotation tail (#204). */
        public const val IIS_ROT_OFFSET: Int = 26
        /** Fixed offset of the v6 mag_type byte. */
        public const val MAG_TYPE_OFFSET: Int = 41

        /** MAG_TYPE_* wire values (RocketComputerTypes.h). */
        public const val MAG_TYPE_IIS2MDC: Int = 0
        public const val MAG_TYPE_QMC5883P: Int = 1

        /** IIS2MDC sensitivity: 1.5 mgauss/LSB = 0.15 µT/LSB (datasheet 9.13). */
        public const val IIS2MDC_UT_PER_LSB: Double = 0.15
        /** QMC5883P at ±8 G, 3750 LSB/gauss (QST Table 2) — the mini's #797 mag. */
        public const val QMC5883P_UT_PER_LSB: Double = 100.0 / 3750.0

        /** The QMC5883P rotation the Beetle stamped before #1590, and its true one. */
        public const val QMC5883P_PRE_1590_STAMP_DEG: Double = 90.0
        public const val QMC5883P_BEETLE_ROT_Z_DEG: Double = -90.0

        /** Returns null on a wrong-size payload (skip, don't throw), mirroring iOS throw-and-caller-skips. */
        public fun decode(payload: ByteArray): OutStatusQueryData? {
            if (payload.size < MIN_SIZE) return null
            val b = LeBuffer(payload)
            val ism6LowGFsG = b.u8()
            val ism6HighGFsG = b.u16()
            val ism6GyroFsDps = b.u16()
            val ism6RotZCdeg = b.i16()
            val mmcRotZCdeg = b.i16()
            val formatVersion = b.u8()

            val iis2mdcRotZCdeg: Int? = if (formatVersion >= 4 && payload.size >= 28) {
                LeBuffer(payload, IIS_ROT_OFFSET).i16()
            } else {
                null
            }

            val magType: Int? = if (formatVersion >= 6 && payload.size >= 42) {
                LeBuffer(payload, MAG_TYPE_OFFSET).u8()
            } else {
                null
            }

            return OutStatusQueryData(
                ism6LowGFsG = ism6LowGFsG,
                ism6HighGFsG = ism6HighGFsG,
                ism6GyroFsDps = ism6GyroFsDps,
                ism6RotZCdeg = ism6RotZCdeg,
                mmcRotZCdeg = mmcRotZCdeg,
                formatVersion = formatVersion,
                iis2mdcRotZCdeg = iis2mdcRotZCdeg,
                magType = magType,
            )
        }
    }
}
