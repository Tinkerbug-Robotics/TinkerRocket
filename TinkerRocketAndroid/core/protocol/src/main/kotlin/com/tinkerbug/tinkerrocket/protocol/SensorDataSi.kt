package com.tinkerbug.tinkerrocket.protocol

/**
 * SI-unit output structures produced by [SensorConverter].
 *
 * Port of the "SI Unit Structures" section of iOS SensorTypes.swift.  The raw
 * wire decoders live in their own files (GnssData.kt et al., mirroring the
 * iOS SensorTypes/SensorConverter split); these are the converter layer's
 * outputs only.  Field semantics, including deliberately odd ones, are pinned
 * to iOS — see per-field comments.
 */

/** Rocket state machine value carried in NonSensor frames. */
public enum class RocketState(public val raw: Int) {
    INITIALIZATION(0),
    READY(1),
    PRELAUNCH(2),
    INFLIGHT(3),
    LANDED(4),
    ;

    public companion object {
        /** null on unknown wire values (callers fall back to [INITIALIZATION], matching iOS `?? .initialization`). */
        public fun fromRaw(raw: Int): RocketState? = entries.firstOrNull { it.raw == raw }
    }
}

public data class GnssDataSi(
    val timeUs: Long,
    val year: Int,
    val month: Int,
    val day: Int,
    val hour: Int,
    val minute: Int,
    val second: Int,
    val milliSecond: Int,
    val fixMode: Int,
    val numSats: Int,
    val pdop: Double,
    val lat: Double,      // degrees
    val lon: Double,      // degrees
    val alt: Double,      // meters
    val velE: Double,     // m/s
    val velN: Double,     // m/s
    val velU: Double,     // m/s
    val hAcc: Double,     // meters
    val vAcc: Double,     // meters
)

public data class PowerDataSi(
    val timeUs: Long,
    val voltage: Double,  // V
    val current: Double,  // mA
    val soc: Double,      // %
)

public data class Bmp585DataSi(
    val timeUs: Long,
    val temperature: Double,  // °C
    val pressure: Double,     // Pa
)

public data class Ism6Hg256DataSi(
    val timeUs: Long,
    val lowGAccX: Double,   // m/s²
    val lowGAccY: Double,
    val lowGAccZ: Double,
    val highGAccX: Double,  // m/s²
    val highGAccY: Double,
    val highGAccZ: Double,
    val gyroX: Double,      // deg/s
    val gyroY: Double,
    val gyroZ: Double,
)

public data class Mmc5983MaDataSi(
    val timeUs: Long,
    val magX: Double,  // μT
    val magY: Double,
    val magZ: Double,
)

public data class NonSensorDataSi(
    val timeUs: Long,

    // #514: the raw attitude quaternion, carried through verbatim.
    //
    // roll/pitch/yaw below are NOT a reconstructable Euler triple: `roll` is the
    // body-Z azimuth (it matches the FC roll controller and doesn't gimbal-lock),
    // while `pitch`/`yaw` are ZYX-Euler. Mixing the two conventions silently
    // produces a garbage attitude — so anything that needs the actual orientation
    // must use the quaternion, which is unambiguous.
    val q0: Double,        // scalar-first, unit
    val q1: Double,
    val q2: Double,
    val q3: Double,

    val roll: Double,      // degrees — BODY-Z AZIMUTH, not the Euler roll (see above)
    val pitch: Double,     // degrees — ZYX Euler
    val yaw: Double,       // degrees — ZYX Euler
    val rollCmd: Double,

    val ePos: Double,      // meters
    val nPos: Double,
    val uPos: Double,

    val eVel: Double,      // m/s
    val nVel: Double,
    val uVel: Double,

    val altitudeRate: Double,  // m/s (KF-filtered baro rate from FlightComputer)

    val altLandedFlag: Boolean,
    val altApogeeFlag: Boolean,
    val velUApogeeFlag: Boolean,
    val launchFlag: Boolean,
    // Firmware-set burnout flag (NSF_BURNOUT, bit 4). Source of truth for
    // burnout_time_s in the .json sidecar (#196). Pre-existing logs that
    // predate the firmware setting this bit decode as false.
    val burnoutFlag: Boolean,

    // Per #142/#143: full apogee detector set + master voted result.
    // Decoded from NonSensorData.apogee_flags; legacy 43-byte logs decode all
    // three as false (those flights pre-date the field).
    val gpsApogeeFlag: Boolean,
    val pitchApogeeFlag: Boolean,
    val apogeeFlag: Boolean,       // master voted result

    val rocketState: RocketState,

    // Derived from NonSensorData.pyro_status (4 channels, new PCB).
    // reboot_recovery and guidance_enabled were relocated to apogee_flags.
    val pyro1Continuity: Boolean,
    val pyro2Continuity: Boolean,
    val pyro3Continuity: Boolean,
    val pyro4Continuity: Boolean,
    val pyro1Fired: Boolean,
    val pyro2Fired: Boolean,
    val pyro3Fired: Boolean,
    val pyro4Fired: Boolean,
    val rebootRecovery: Boolean,
    val guidanceEnabled: Boolean,

    // #529: free-running EKF update-tick counter (uint16 wrap), carried through
    // verbatim for the CSV so the achieved EKF rate is recoverable from an app
    // export.  null on logs that predate the 50-byte layout.
    val ekfTicks: Int?,
)
