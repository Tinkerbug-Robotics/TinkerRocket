package com.tinkerbug.tinkerrocket.protocol

/**
 * IIS2MDCData — msg 0xD1 (new Mini PCB rev, replaces MMC5983MA), 10-byte
 * packed payload.  Raw signed i16 per axis at 0.15 µT/LSB (datasheet 9.13);
 * the µT conversion is the converter layer's job.
 */
public data class Iis2mdcData(
    val timeUs: Long,        // u32
    val magX: Int,           // i16 raw counts (signed)
    val magY: Int,
    val magZ: Int,
) {
    public companion object {
        public const val SIZE: Int = 10

        /** Returns null on a wrong-size payload (skip, don't throw), mirroring iOS throw-and-caller-skips. */
        public fun decode(payload: ByteArray): Iis2mdcData? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            return Iis2mdcData(
                timeUs = b.u32(),
                magX = b.i16(), magY = b.i16(), magZ = b.i16(),
            )
        }
    }
}
