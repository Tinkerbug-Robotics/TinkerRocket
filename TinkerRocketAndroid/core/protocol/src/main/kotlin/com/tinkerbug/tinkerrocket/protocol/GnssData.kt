package com.tinkerbug.tinkerrocket.protocol

/**
 * GNSSData — msg 0xA1, 42-byte packed payload (RocketComputerTypes.h).
 * Raw wire fields; unit conversion (×1e-7 deg, mm→m, ×0.1 PDOP) is the
 * converter layer's job, mirroring the iOS SensorTypes/SensorConverter split.
 *
 * First decoder family through the golden walk — the template every later
 * decoder follows: raw fields, explicit widths, length gate, LeBuffer only.
 */
public data class GnssData(
    val timeUs: Long,        // u32
    val year: Int,           // u16
    val month: Int,          // u8
    val day: Int,
    val hour: Int,
    val minute: Int,
    val second: Int,
    val milliSecond: Int,    // u16
    val fixMode: Int,        // u8: 0 none, 1 DR, 2 2D, 3 3D, 4 GNSS+DR, 5 time-only
    val numSats: Int,        // u8
    val pdopX10: Int,        // u8
    val latE7: Int,          // i32, deg * 1e7
    val lonE7: Int,
    val altMm: Int,
    val velEMmps: Int,       // i32, mm/s
    val velNMmps: Int,
    val velUMmps: Int,
    val hAccM: Int,          // u8
    val vAccM: Int,
) {
    public companion object {
        public const val SIZE: Int = 42

        /** Returns null on a wrong-size payload (skip, don't throw — log files survive garbage). */
        public fun decode(payload: ByteArray): GnssData? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            return GnssData(
                timeUs = b.u32(),
                year = b.u16(),
                month = b.u8(), day = b.u8(),
                hour = b.u8(), minute = b.u8(), second = b.u8(),
                milliSecond = b.u16(),
                fixMode = b.u8(), numSats = b.u8(), pdopX10 = b.u8(),
                latE7 = b.i32(), lonE7 = b.i32(), altMm = b.i32(),
                velEMmps = b.i32(), velNMmps = b.i32(), velUMmps = b.i32(),
                hAccM = b.u8(), vAccM = b.u8(),
            )
        }
    }
}
