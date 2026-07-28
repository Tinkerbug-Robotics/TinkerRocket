package com.tinkerbug.tinkerrocket.protocol

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
 */
public data class OutStatusQueryData(
    val ism6LowGFsG: Int,        // u8, e.g. 16
    val ism6HighGFsG: Int,       // u16, e.g. 256
    val ism6GyroFsDps: Int,      // u16, e.g. 4000
    val ism6RotZCdeg: Int,       // i16, centi-degrees
    val mmcRotZCdeg: Int,        // i16, centi-degrees
    val formatVersion: Int,      // u8
    val iis2mdcRotZCdeg: Int?,   // i16, centi-degrees; null pre-v4 (#204)
) {
    /** IMU rotation in degrees. */
    public val imuRotationDeg: Double get() = ism6RotZCdeg / 100.0

    /** Magnetometer (MMC5983MA) rotation in degrees. */
    public val magRotationDeg: Double get() = mmcRotZCdeg / 100.0

    /** IIS2MDC rotation in degrees (format_version >= 4); null on older logs. */
    public val iisRotationDeg: Double? get() = iis2mdcRotZCdeg?.let { it / 100.0 }

    public companion object {
        public const val MIN_SIZE: Int = 10
        /** Fixed offset of the v4 IIS2MDC rotation tail (#204). */
        public const val IIS_ROT_OFFSET: Int = 26

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

            return OutStatusQueryData(
                ism6LowGFsG = ism6LowGFsG,
                ism6HighGFsG = ism6HighGFsG,
                ism6GyroFsDps = ism6GyroFsDps,
                ism6RotZCdeg = ism6RotZCdeg,
                mmcRotZCdeg = mmcRotZCdeg,
                formatVersion = formatVersion,
                iis2mdcRotZCdeg = iis2mdcRotZCdeg,
            )
        }
    }
}
