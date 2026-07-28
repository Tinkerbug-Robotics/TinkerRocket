package com.tinkerbug.tinkerrocket.protocol

/**
 * BMP585Data — msg 0xA3 (Mini boards), 12-byte packed payload.
 * The legacy 10-byte MS5611 payload shares the message id — payload size is
 * the discriminator (see [LegacyMs5611Data]).
 */
public data class Bmp585Data(
    val timeUs: Long,        // u32
    val tempQ16: Int,        // i32: degC * 65536
    val pressQ6: Long,       // u32: Pa * 64
) {
    public companion object {
        public const val SIZE: Int = 12

        /** Returns null on a wrong-size payload (skip, don't throw), mirroring iOS throw-and-caller-skips. */
        public fun decode(payload: ByteArray): Bmp585Data? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            return Bmp585Data(
                timeUs = b.u32(),
                tempQ16 = b.i32(),
                pressQ6 = b.u32(),
            )
        }
    }
}
