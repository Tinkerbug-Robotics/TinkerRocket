package com.tinkerbug.tinkerrocket.protocol

/**
 * Legacy-board payload decoders (pre-Mini PCBs).  Legacy uses different
 * sensors with different payload formats on the SAME message ids —
 * auto-detection is by payload size (e.g. IMU 0xA2: 36 B = legacy ICM45686,
 * 22 B = Mini ISM6HG256).  Ports of the Legacy* structs in the iOS
 * SensorTypes.swift, byte-for-byte.
 */

/** Legacy ICM45686 IMU — msg 0xA2, 36-byte packed payload. */
public data class LegacyIcm45686Data(
    val timeUs: Long,        // u32
    val timeSync: Long,      // u32
    val accX: Int,           // i32 raw LSB (±32g: 0.00059855 m/s²/LSB)
    val accY: Int,
    val accZ: Int,
    val gyroX: Int,          // i32 raw LSB (±4000 dps: 0.00762777 deg/s/LSB)
    val gyroY: Int,
    val gyroZ: Int,
    val temp: Int,           // i32
) {
    public companion object {
        public const val SIZE: Int = 36

        /** Returns null on a wrong-size payload (skip, don't throw), mirroring iOS throw-and-caller-skips. */
        public fun decode(payload: ByteArray): LegacyIcm45686Data? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            return LegacyIcm45686Data(
                timeUs = b.u32(),
                timeSync = b.u32(),
                accX = b.i32(), accY = b.i32(), accZ = b.i32(),
                gyroX = b.i32(), gyroY = b.i32(), gyroZ = b.i32(),
                temp = b.i32(),
            )
        }
    }
}

/** Legacy H3LIS331 high-G accelerometer — msg 0xA9, 10-byte packed payload. */
public data class LegacyH3lis331Data(
    val timeUs: Long,        // u32
    val accX: Int,           // i16 raw LSB (±200g: 0.95788 m/s²/LSB)
    val accY: Int,
    val accZ: Int,
) {
    public companion object {
        public const val SIZE: Int = 10

        /** Returns null on a wrong-size payload (skip, don't throw), mirroring iOS throw-and-caller-skips. */
        public fun decode(payload: ByteArray): LegacyH3lis331Data? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            return LegacyH3lis331Data(
                timeUs = b.u32(),
                accX = b.i16(), accY = b.i16(), accZ = b.i16(),
            )
        }
    }
}

/** Legacy MS5611 barometer — msg 0xA3, 10-byte packed payload. */
public data class LegacyMs5611Data(
    val timeUs: Long,        // u32
    val pressure: Long,      // u32 encoded: hPa = 10 + code * 1190 / 4294967295
    val temperature: Int,    // i16 encoded: °C = ((raw+32768)/65535)*100 - 40
) {
    public companion object {
        public const val SIZE: Int = 10

        /** Returns null on a wrong-size payload (skip, don't throw), mirroring iOS throw-and-caller-skips. */
        public fun decode(payload: ByteArray): LegacyMs5611Data? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            return LegacyMs5611Data(
                timeUs = b.u32(),
                pressure = b.u32(),
                temperature = b.i16(),
            )
        }
    }
}

/** Legacy LIS3MDL magnetometer — msg 0xA4, 10-byte packed payload. */
public data class LegacyLis3mdlData(
    val timeUs: Long,        // u32
    val magX: Int,           // i16 raw LSB (0.014 μT/LSB)
    val magY: Int,
    val magZ: Int,
) {
    public companion object {
        public const val SIZE: Int = 10

        /** Returns null on a wrong-size payload (skip, don't throw), mirroring iOS throw-and-caller-skips. */
        public fun decode(payload: ByteArray): LegacyLis3mdlData? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            return LegacyLis3mdlData(
                timeUs = b.u32(),
                magX = b.i16(), magY = b.i16(), magZ = b.i16(),
            )
        }
    }
}

/** Legacy NonSensor — msg 0xA5, 65-byte packed payload. */
public data class LegacyNonSensorData(
    val timeUs: Long,        // u32

    // Attitude (already in degrees as float, NOT centidegrees)
    val roll: Float,
    val pitch: Float,
    val yaw: Float,
    val rollCmd: Float,

    // Position (cm), i32
    val ePos: Int,
    val nPos: Int,
    val uPos: Int,

    // Velocity (cm/s), i32
    val eVel: Int,
    val nVel: Int,
    val uVel: Int,

    // Additional fields (not in Mini), i32
    val pressureAlt: Int,    // meters
    val altitudeRate: Int,   // dm/s (0.1 m/s)
    val maxAlt: Int,         // meters
    val maxSpeed: Int,       // m/s

    // Flags (separate bools on the wire, not a bitfield)
    val altLandedFlag: Boolean,
    val altApogeeFlag: Boolean,
    val velUApogeeFlag: Boolean,
    val launchFlag: Boolean,
    val rocketState: Int,    // u8
) {
    public companion object {
        public const val SIZE: Int = 65

        /** Returns null on a wrong-size payload (skip, don't throw), mirroring iOS throw-and-caller-skips. */
        public fun decode(payload: ByteArray): LegacyNonSensorData? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            return LegacyNonSensorData(
                timeUs = b.u32(),
                roll = b.f32(), pitch = b.f32(), yaw = b.f32(), rollCmd = b.f32(),
                ePos = b.i32(), nPos = b.i32(), uPos = b.i32(),
                eVel = b.i32(), nVel = b.i32(), uVel = b.i32(),
                pressureAlt = b.i32(), altitudeRate = b.i32(),
                maxAlt = b.i32(), maxSpeed = b.i32(),
                altLandedFlag = b.u8() != 0,
                altApogeeFlag = b.u8() != 0,
                velUApogeeFlag = b.u8() != 0,
                launchFlag = b.u8() != 0,
                rocketState = b.u8(),
            )
        }
    }
}
