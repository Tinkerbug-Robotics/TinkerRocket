package com.tinkerbug.tinkerrocket.protocol

/**
 * POWERData — msg 0xA6, 10-byte packed payload (RocketComputerTypes.h).
 * Raw wire fields; unit conversion (voltage/current/soc scaling) is the
 * converter layer's job, mirroring the iOS SensorTypes/SensorConverter split.
 */
public data class PowerData(
    val timeUs: Long,        // u32
    val voltageRaw: Int,     // u16: (V / 10.0) * 65535
    val currentRaw: Int,     // i16: (mA / 10000.0) * 32767
    val socRaw: Int,         // i16: (soc + 25) * (32767/150)
) {
    public companion object {
        public const val SIZE: Int = 10

        /** Returns null on a wrong-size payload (skip, don't throw), mirroring iOS throw-and-caller-skips. */
        public fun decode(payload: ByteArray): PowerData? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            return PowerData(
                timeUs = b.u32(),
                voltageRaw = b.u16(),
                currentRaw = b.i16(),
                socRaw = b.i16(),
            )
        }
    }
}
