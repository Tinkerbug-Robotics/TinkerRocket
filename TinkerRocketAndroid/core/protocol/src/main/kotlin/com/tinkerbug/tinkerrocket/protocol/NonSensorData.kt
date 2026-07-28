package com.tinkerbug.tinkerrocket.protocol

/**
 * NonSensorData — msg 0xA5 (Mini boards), 43/44/48/50-byte length ladder
 * (oldest → newest).  Wire layout mirrors C++ NonSensorData in
 * RocketComputerTypes.h.
 *
 * Ladder semantics (regression-pinned against iOS SensorTypes.swift):
 *  - 43 B  pre-#142/#143 legacy — [apogeeFlags] decodes as 0 (matches the
 *    historical "we never recorded these bits" behavior).
 *  - 44 B  +apogee_flags.
 *  - 48 B  +sensor_health (#303) — deliberately NOT exposed: the iOS app has
 *    never surfaced it from the log decode, and this port pins iOS behavior.
 *  - 50 B  +ekf_ticks (#529) — [ekfTicks] is NULL below 50 bytes, never 0:
 *    0 is a real counter value ("EKF not yet initialized"), so the null-vs-0
 *    distinction is semantic (the replay tool derives EKF rate from deltas).
 */
public data class NonSensorData(
    val timeUs: Long,           // u32

    // Quaternion (scalar-first, unit quaternion, i16 * 10000)
    val q0: Int,
    val q1: Int,
    val q2: Int,
    val q3: Int,
    val rollCmd: Int,           // i16, deg * 100

    // Position (cm), i32
    val ePos: Int,
    val nPos: Int,
    val uPos: Int,

    // Velocity (cm/s), i32
    val eVel: Int,
    val nVel: Int,
    val uVel: Int,

    // Flags and state, u8
    val flags: Int,
    val rocketState: Int,

    // KF-filtered barometric altitude rate (dm/s = 0.1 m/s), i16
    val baroAltRateDmps: Int,

    // Pyro channel status bitfield — 4 channels (new PCB):
    //   bit 0 PSF_CH1_CONT, bit 1 PSF_CH1_FIRED, ... bit 7 PSF_CH4_FIRED
    val pyroStatus: Int,        // u8

    // Apogee detector outputs + master vote + relocated reboot/guidance bits:
    //   bit 0 NSF2_GPS_APOGEE, bit 1 NSF2_PITCH_APOGEE, bit 2 NSF2_MASTER_APOGEE,
    //   bit 3 NSF2_REBOOT_RECOVERY, bit 4 NSF2_GUIDANCE_ENABLED
    // Older logs (43-byte struct) decode this field as 0.
    val apogeeFlags: Int,       // u8

    // #529 (50-byte layout): free-running EKF update-tick counter (u16 wrap).
    // null on logs that predate the field — 0 is a real value.
    val ekfTicks: Int?,
) {
    public companion object {
        /** Oldest accepted layout (pre-#142/#143 legacy). */
        public const val MIN_SIZE: Int = 43
        /** +apogee_flags. */
        public const val SIZE_APOGEE_FLAGS: Int = 44
        /** +sensor_health (#303, never surfaced) +ekf_ticks (#529). */
        public const val SIZE_EKF_TICKS: Int = 50

        /** Returns null on a wrong-size payload (skip, don't throw), mirroring iOS throw-and-caller-skips. */
        public fun decode(payload: ByteArray): NonSensorData? {
            if (payload.size < MIN_SIZE) return null
            val hasApogeeFlags = payload.size >= SIZE_APOGEE_FLAGS
            val b = LeBuffer(payload)

            val timeUs = b.u32()
            val q0 = b.i16(); val q1 = b.i16(); val q2 = b.i16(); val q3 = b.i16()
            val rollCmd = b.i16()
            val ePos = b.i32(); val nPos = b.i32(); val uPos = b.i32()
            val eVel = b.i32(); val nVel = b.i32(); val uVel = b.i32()
            val flags = b.u8()
            val rocketState = b.u8()
            val baroAltRateDmps = b.i16()
            val pyroStatus = b.u8()
            val apogeeFlags = if (hasApogeeFlags) b.u8() else 0

            // #529: ekf_ticks sits AFTER sensor_health (#303, 4 bytes, offset
            // 44), which the app has never surfaced — skip over it.
            val ekfTicks: Int?
            if (payload.size >= SIZE_EKF_TICKS) {
                b.position += 4  // sensor_health
                ekfTicks = b.u16()
            } else {
                ekfTicks = null
            }

            return NonSensorData(
                timeUs = timeUs,
                q0 = q0, q1 = q1, q2 = q2, q3 = q3,
                rollCmd = rollCmd,
                ePos = ePos, nPos = nPos, uPos = uPos,
                eVel = eVel, nVel = nVel, uVel = uVel,
                flags = flags,
                rocketState = rocketState,
                baroAltRateDmps = baroAltRateDmps,
                pyroStatus = pyroStatus,
                apogeeFlags = apogeeFlags,
                ekfTicks = ekfTicks,
            )
        }
    }
}
