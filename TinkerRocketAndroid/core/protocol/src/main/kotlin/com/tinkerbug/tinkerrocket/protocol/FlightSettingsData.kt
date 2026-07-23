package com.tinkerbug.tinkerrocket.protocol

/** One roll-profile waypoint as stored in the flight settings frame. */
public data class RollWaypointRaw(
    val timeS: Float,
    val angleDeg: Float,
    val mode: Int,        // u8: 0 = ROLL_SEG_ANGLE, 1 = ROLL_SEG_NULL_RATE
)

/**
 * FlightSettingsData — msg 0xE1, runtime settings snapshot at launch (#165).
 * 188 B v1 / 200 B v2 / 208 B v3 / 210 B v5 / 219 B v6.  Layout mirrors the
 * packed C++ struct in RocketComputerTypes.h byte-for-byte (verified by
 * offset); port of the iOS SensorTypes.swift decoder.
 *
 * Every version tail sits at a FIXED offset (independent of num_waypoints —
 * the roll profile occupies its full 76 bytes on the wire) and is
 * dual-gated on (version, payload length):
 *  - v2 @ 188: board→rocket orientation      (version >= 2 && size >= 200)
 *  - v3 @ 200: fin-angle calibration (#267)  (version >= 3 && size >= 208)
 *  - v5 @ 208: IMU logging rate              (version >= 5 && size >= 210)
 *  - v6 @ 210: flown guidance target (#435)  (version >= 6 && size >= 219)
 */
public data class FlightSettingsData(
    val timeUs: Long,            // u32
    val version: Int,            // u8
    val flags: Int,              // u8
    val rollDelayMs: Int,        // u16

    val kp: Float,
    val ki: Float,
    val kd: Float,
    val dLpfHz: Float,
    val minCmdDeg: Float,
    val maxCmdDeg: Float,

    val kpAngle: Float,
    val kpAngleRateCapDps: Float,

    val gsVRef: Float,
    val gsVMin: Float,
    val gsScaleCap: Float,

    val rollRateSetPoint: Float,

    val ism6LowGFsG: Int,        // u8
    val ism6HighGFsG: Int,       // u16
    val ism6GyroFsDps: Int,      // u16

    val servoBiasUs: List<Int>,  // 4 × i16
    val servoHz: Int,            // i16
    val servoMinUs: Int,         // i16
    val servoMaxUs: Int,         // i16

    val cameraType: Int,         // u8

    // 4 pyro channels (new PCB). Lists indexed 0..3 → channels 1..4.
    val pyroEnabled: List<Boolean>,
    val pyroTriggerMode: List<Int>,    // u8: 0 = time-after-apogee, 1 = altitude-on-descent
    val pyroTriggerValue: List<Float>,

    val fwGitSha: String,        // 12-byte NUL-terminated char array
    val numWaypoints: Int,       // u8, clamped to 8
    val waypoints: List<RollWaypointRaw>,

    // v2 tail: board→rocket mounting orientation @ fixed offset 188.
    // null on v1 frames (pre-orientation firmware, +X-nose assumed).
    val b2rCode: Int?,           // u8
    val b2rMode: Int?,           // u8: 0 default, 1 manual, 2 auto-snap, 3 auto-exact
    val b2rResidualDeg: Float?,  // i16 cdeg / 100 — auto-snap residual angle
    val b2rQuat: List<Float>?,   // 4 × i16 / 10000 — board→rocket quaternion, scalar-first

    // v3 tail: fin-angle calibration (#267) @ fixed offset 200. null pre-v3.
    val finMinDeg: Float?,
    val finMaxDeg: Float?,

    /** v5+ @ 208: IMU logging rate (ISM6HG256 ODR, Hz) that actually flew. */
    val ism6UpdateRateHz: Int?,  // u16

    // v6+ (#435) @ 210: the horizontal guidance aim point that actually
    // FLEW — pad-relative ENU meters, re-converted from the frozen GNSS
    // reference at launch.  guidTgtSrc is the GUID_TGT_* code: 0
    // none/overhead, 1 cmd-28 geodetic (Drift-Cast), 2 cmd-65 profile E/N.
    val guidTgtEM: Float?,
    val guidTgtNM: Float?,
    val guidTgtSrc: Int?,        // u8
) {
    public val useAngleControl: Boolean get() = flags and (1 shl F_USE_ANGLE_CONTROL) != 0
    public val gainScheduleEnabled: Boolean get() = flags and (1 shl F_GAIN_SCHEDULE) != 0
    public val guidanceEnabled: Boolean get() = flags and (1 shl F_GUIDANCE) != 0
    public val servoEnabled: Boolean get() = flags and (1 shl F_SERVO_ENABLED) != 0
    public val fwDirty: Boolean get() = flags and (1 shl F_FW_DIRTY) != 0
    public val soundsEnabled: Boolean get() = flags and (1 shl F_SOUNDS) != 0

    /** "−Z r90"-style mounting name; null on pre-v2 frames. */
    public val b2rDisplayName: String? get() = b2rCode?.let { b2rName(it) }

    public companion object {
        // flags bit positions (match FlightSettingsData::F_* in firmware)
        public const val F_USE_ANGLE_CONTROL: Int = 0
        public const val F_GAIN_SCHEDULE: Int = 1
        public const val F_GUIDANCE: Int = 2
        public const val F_SERVO_ENABLED: Int = 3
        public const val F_FW_DIRTY: Int = 4
        public const val F_SOUNDS: Int = 5

        public const val MIN_SIZE: Int = 188

        /**
         * "−Z r90"-style mounting name (mirrors firmware orientCodeName):
         * which board axis points at the nose + quarter-turn clocking.
         */
        public fun b2rName(code: Int): String {
            val axes = listOf("+X", "-X", "+Y", "-Y", "+Z", "-Z")
            if (code < 0 || code >= 24) return "?"
            val base = axes[code / 4]
            val clock = (code % 4) * 90
            return if (clock == 0) base else "$base r$clock"
        }

        /** Returns null on a wrong-size payload (skip, don't throw), mirroring iOS throw-and-caller-skips. */
        public fun decode(payload: ByteArray): FlightSettingsData? {
            if (payload.size < MIN_SIZE) return null
            val b = LeBuffer(payload)

            val timeUs = b.u32()
            val version = b.u8()
            val flags = b.u8()
            val rollDelayMs = b.u16()

            val kp = b.f32(); val ki = b.f32(); val kd = b.f32()
            val dLpfHz = b.f32()
            val minCmdDeg = b.f32(); val maxCmdDeg = b.f32()

            val kpAngle = b.f32()
            val kpAngleRateCapDps = b.f32()

            val gsVRef = b.f32(); val gsVMin = b.f32(); val gsScaleCap = b.f32()

            val rollRateSetPoint = b.f32()

            val ism6LowGFsG = b.u8()
            val ism6HighGFsG = b.u16()
            val ism6GyroFsDps = b.u16()

            // Servo trim + timing (offset 61)
            val servoBiasUs = List(4) { b.i16() }
            val servoHz = b.i16()
            val servoMinUs = b.i16()
            val servoMaxUs = b.i16()

            val cameraType = b.u8()   // offset 75

            // PyroConfigData (offset 76, 24 bytes — 4 channels)
            val pyroEnabled = ArrayList<Boolean>(4)
            val pyroTriggerMode = ArrayList<Int>(4)
            val pyroTriggerValue = ArrayList<Float>(4)
            repeat(4) {
                pyroEnabled.add(b.u8() != 0)
                pyroTriggerMode.add(b.u8())
                pyroTriggerValue.add(b.f32())
            }

            // fw_git_sha: 12-byte NUL-terminated char array (offset 100..111)
            val shaBytes = b.bytes(12)
            val fwGitSha = shaBytes.takeWhile { it != 0.toByte() }
                .toByteArray().toString(Charsets.UTF_8)

            // roll_profile: RollProfileData @ offset 112
            val nRaw = b.u8()          // num_waypoints (offset 112)
            b.position += 3            // _pad[3]
            val n = minOf(nRaw, 8)
            val waypoints = ArrayList<RollWaypointRaw>(n)
            repeat(n) {                // waypoints @ offset 116, 9 bytes each
                val t = b.f32()
                val a = b.f32()
                val m = b.u8()
                waypoints.add(RollWaypointRaw(timeS = t, angleDeg = a, mode = m))
            }

            // v2 board→rocket orientation tail at fixed offset 188 (after
            // the full 76-byte roll profile, independent of num_waypoints).
            var b2rCode: Int? = null
            var b2rMode: Int? = null
            var b2rResidualDeg: Float? = null
            var b2rQuat: List<Float>? = null
            if (version >= 2 && payload.size >= 200) {
                val o = LeBuffer(payload, 188)
                b2rCode = o.u8()
                b2rMode = o.u8()
                b2rResidualDeg = o.i16() / 100.0f
                b2rQuat = List(4) { o.i16() / 10000.0f }
            }

            // v3 fin-angle calibration tail at fixed offset 200 (#267).
            var finMinDeg: Float? = null
            var finMaxDeg: Float? = null
            if (version >= 3 && payload.size >= 208) {
                val o = LeBuffer(payload, 200)
                finMinDeg = o.f32()
                finMaxDeg = o.f32()
            }

            // v5 IMU logging rate tail at fixed offset 208.
            var ism6UpdateRateHz: Int? = null
            if (version >= 5 && payload.size >= 210) {
                ism6UpdateRateHz = LeBuffer(payload, 208).u16()
            }

            // v6 flown-guidance-target tail at fixed offset 210 (#435).
            var guidTgtEM: Float? = null
            var guidTgtNM: Float? = null
            var guidTgtSrc: Int? = null
            if (version >= 6 && payload.size >= 219) {
                val o = LeBuffer(payload, 210)
                guidTgtEM = o.f32()
                guidTgtNM = o.f32()
                guidTgtSrc = o.u8()
            }

            return FlightSettingsData(
                timeUs = timeUs,
                version = version,
                flags = flags,
                rollDelayMs = rollDelayMs,
                kp = kp, ki = ki, kd = kd,
                dLpfHz = dLpfHz,
                minCmdDeg = minCmdDeg, maxCmdDeg = maxCmdDeg,
                kpAngle = kpAngle,
                kpAngleRateCapDps = kpAngleRateCapDps,
                gsVRef = gsVRef, gsVMin = gsVMin, gsScaleCap = gsScaleCap,
                rollRateSetPoint = rollRateSetPoint,
                ism6LowGFsG = ism6LowGFsG,
                ism6HighGFsG = ism6HighGFsG,
                ism6GyroFsDps = ism6GyroFsDps,
                servoBiasUs = servoBiasUs,
                servoHz = servoHz,
                servoMinUs = servoMinUs,
                servoMaxUs = servoMaxUs,
                cameraType = cameraType,
                pyroEnabled = pyroEnabled,
                pyroTriggerMode = pyroTriggerMode,
                pyroTriggerValue = pyroTriggerValue,
                fwGitSha = fwGitSha,
                numWaypoints = n,
                waypoints = waypoints,
                b2rCode = b2rCode,
                b2rMode = b2rMode,
                b2rResidualDeg = b2rResidualDeg,
                b2rQuat = b2rQuat,
                finMinDeg = finMinDeg,
                finMaxDeg = finMaxDeg,
                ism6UpdateRateHz = ism6UpdateRateHz,
                guidTgtEM = guidTgtEM,
                guidTgtNM = guidTgtNM,
                guidTgtSrc = guidTgtSrc,
            )
        }
    }
}
