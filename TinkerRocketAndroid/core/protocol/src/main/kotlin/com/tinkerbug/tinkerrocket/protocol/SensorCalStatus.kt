package com.tinkerbug.tinkerrocket.protocol

/**
 * SensorCalStatus — decoded form of the FC's SensorCalStatusData frame
 * (issue #132): the gyro zero-rate bias + high-g accel bias currently stored
 * in the rocket's NVS.  Arrives on the file_ops characteristic behind a 0xCB
 * discriminator (sibling of the 0xCA mag-cal frame).  Wire layout mirrors
 * SensorCalStatusData in RocketComputerTypes.h.
 *
 * Wire (19 bytes, little-endian, packed):
 *   [0]      u8   valid       (1 if a cal is stored)
 *   [1..2]   i16  gyro_x      (raw gyro LSB)
 *   [3..4]   i16  gyro_y
 *   [5..6]   i16  gyro_z
 *   [7..10]  f32  hg_x        (high-g accel bias, m/s²)
 *   [11..14] f32  hg_y
 *   [15..18] f32  hg_z
 */
public data class SensorCalStatus(
    val valid: Boolean,
    val gyroX: Int,       // i16 raw gyro LSB
    val gyroY: Int,
    val gyroZ: Int,
    val hgX: Float,       // high-g accel bias, m/s²
    val hgY: Float,
    val hgZ: Float,
) {
    public companion object {
        public const val SIZE: Int = 19

        /** Decode the payload *after* the 0xCB discriminator byte; null if too short. */
        public fun decode(payload: ByteArray): SensorCalStatus? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            return SensorCalStatus(
                valid = b.u8() != 0,
                gyroX = b.i16(), gyroY = b.i16(), gyroZ = b.i16(),
                hgX = b.f32(), hgY = b.f32(), hgZ = b.f32(),
            )
        }
    }
}
