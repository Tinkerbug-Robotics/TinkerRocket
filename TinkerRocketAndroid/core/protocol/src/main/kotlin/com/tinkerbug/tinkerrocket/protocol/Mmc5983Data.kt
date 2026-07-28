package com.tinkerbug.tinkerrocket.protocol

/**
 * MMC5983MAData — msg 0xA4 (Mini boards), 16-byte packed payload.
 * Raw u32 counts per axis (the 18-bit-mask handling is the converter's job).
 * The legacy 10-byte LIS3MDL payload shares the message id — payload size is
 * the discriminator (see [LegacyLis3mdlData]).
 */
public data class Mmc5983Data(
    val timeUs: Long,        // u32
    val magX: Long,          // u32 raw counts
    val magY: Long,
    val magZ: Long,
) {
    public companion object {
        public const val SIZE: Int = 16

        /** Returns null on a wrong-size payload (skip, don't throw), mirroring iOS throw-and-caller-skips. */
        public fun decode(payload: ByteArray): Mmc5983Data? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            return Mmc5983Data(
                timeUs = b.u32(),
                magX = b.u32(), magY = b.u32(), magZ = b.u32(),
            )
        }
    }
}
