package com.tinkerbug.tinkerrocket.protocol

/**
 * ISM6HG256Data — msg 0xA2 (Mini boards), 22-byte packed payload.
 * The legacy 36-byte ICM45686 payload shares the message id — payload size is
 * the discriminator (see [LegacyIcm45686Data]).
 */
public data class Ism6hg256Data(
    val timeUs: Long,        // u32

    // Low-G accelerometer (±16g), raw i16 counts
    val accLowX: Int,
    val accLowY: Int,
    val accLowZ: Int,

    // High-G accelerometer (±256g), raw i16 counts
    val accHighX: Int,
    val accHighY: Int,
    val accHighZ: Int,

    // Gyroscope (±4000 deg/s), raw i16 counts
    val gyroX: Int,
    val gyroY: Int,
    val gyroZ: Int,
) {
    public companion object {
        public const val SIZE: Int = 22

        /** Returns null on a wrong-size payload (skip, don't throw), mirroring iOS throw-and-caller-skips. */
        public fun decode(payload: ByteArray): Ism6hg256Data? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            return Ism6hg256Data(
                timeUs = b.u32(),
                accLowX = b.i16(), accLowY = b.i16(), accLowZ = b.i16(),
                accHighX = b.i16(), accHighY = b.i16(), accHighZ = b.i16(),
                gyroX = b.i16(), gyroY = b.i16(), gyroZ = b.i16(),
            )
        }
    }
}
