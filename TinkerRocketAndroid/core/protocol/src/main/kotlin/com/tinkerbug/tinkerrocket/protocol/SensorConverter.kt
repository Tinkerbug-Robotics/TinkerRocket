package com.tinkerbug.tinkerrocket.protocol

import kotlin.math.PI
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

/**
 * Convert raw sensor values to SI units.
 *
 * Direct port of iOS TinkerRocketApp/Models/SensorConverter.swift — the iOS
 * source is the behavioral reference, including regression-pinned constants
 * that look wrong on purpose (fixed ST gyro sensitivity #369/#564, hardcoded
 * full scales, legacy per-LSB constants).  One instance per conversion job:
 * rotation angles are mutable per-device state configured from the 0xA0
 * status-query frame (Mini) or defaulted to 180° (Legacy).
 */
public class SensorConverter {

    // ISM6HG256 configuration (match hardware defaults)
    private val lowGFullScale: Double = 16.0    // ±16g
    private val highGFullScale: Double = 256.0  // ±256g
    private val gyroFullScale: Double = 4000.0  // ±4000 deg/s

    private val gMs2: Double = 9.80665  // Standard gravity in m/s²

    // Sensitivity values (calculated from full scales)
    private val accLowMs2PerLsb: Double
    private val accHighMs2PerLsb: Double
    private val gyroDpsPerLsb: Double

    // Mini sensor-frame → board-frame rotation angles (configurable per device)
    private var ism6RotZRad: Double = 0.0
    private var mmcRotZRad: Double = 0.0
    private var iis2mdcRotZRad: Double = 0.0

    init {
        // Calculate sensitivity values. Mirror the firmware converter
        // (TR_Sensor_Data_Converter.cpp configureISM6HG256FullScale) exactly:
        //   accel: the full ±32768 LSB span maps to the full scale, so
        //          mg/LSB = FS[g] * 1000 / 32768.
        //   gyro : ST gyros use a FIXED per-FS sensitivity, NOT FS/32768 —
        //          8.75 mdps/LSB at ±250 dps, doubling each FS step (140 @
        //          ±4000), i.e. mdps/LSB = FS[dps] * 0.035. The old FS/32768
        //          form (122.07 @ ±4000) understated every CSV gyro value
        //          ~12.8% — fixed in firmware by #369, here by #564.
        val denom = 32768.0

        val lowMgPerLsb = (lowGFullScale * 1000.0) / denom
        val highMgPerLsb = (highGFullScale * 1000.0) / denom
        val gyroMdpsPerLsb = gyroFullScale * 0.035

        // mg/LSB -> m/s² per LSB
        accLowMs2PerLsb = (lowMgPerLsb * 1e-3) * gMs2
        accHighMs2PerLsb = (highMgPerLsb * 1e-3) * gMs2

        // mdps/LSB -> dps per LSB
        gyroDpsPerLsb = gyroMdpsPerLsb * 1e-3
    }

    // MARK: - Rotation Configuration

    /**
     * Configure Mini sensor rotation angles from status query data.
     * Called when a 0xA0 (statusQuery) frame is found in the binary log.
     */
    public fun configureMiniRotation(imuDeg: Double, magDeg: Double, iisDeg: Double? = null) {
        ism6RotZRad = imuDeg * PI / 180.0
        mmcRotZRad = magDeg * PI / 180.0
        // IIS2MDC (new PCB) has its own sensor→board rotation (#204).  Older
        // logs (status query format_version < 4) don't carry it; fall back to
        // the MMC value to preserve prior behavior on those.
        iis2mdcRotZRad = (iisDeg ?: magDeg) * PI / 180.0
    }

    /**
     * Configure Legacy sensor rotation.
     * Legacy firmware logs raw (unrotated) sensor data, so the app
     * applies 180° rotation to convert from sensor frame to rocket body frame.
     */
    @Suppress("UNUSED_PARAMETER")
    public fun configureLegacyRotation(imuDeg: Double = 180.0, magDeg: Double = 180.0) {
        val imuRad = imuDeg * PI / 180.0
        legacyRotC = cos(imuRad)
        legacyRotS = sin(imuRad)
        // Mag rotation for Legacy uses the same legacyRotC/S (same angle for all sensors)
    }

    // MARK: - GNSS Conversion

    public fun convertGNSS(raw: GnssData): GnssDataSi {
        return GnssDataSi(
            timeUs = raw.timeUs,
            year = raw.year,
            month = raw.month,
            day = raw.day,
            hour = raw.hour,
            minute = raw.minute,
            second = raw.second,
            milliSecond = raw.milliSecond,
            fixMode = raw.fixMode,
            numSats = raw.numSats,
            pdop = raw.pdopX10.toDouble() / 10.0,
            lat = raw.latE7.toDouble() * 1e-7,       // degrees
            lon = raw.lonE7.toDouble() * 1e-7,       // degrees
            alt = raw.altMm.toDouble() * 1e-3,       // meters
            velE = raw.velEMmps.toDouble() * 1e-3,   // m/s
            velN = raw.velNMmps.toDouble() * 1e-3,   // m/s
            velU = raw.velUMmps.toDouble() * 1e-3,   // m/s
            hAcc = raw.hAccM.toDouble(),             // meters
            vAcc = raw.vAccM.toDouble(),             // meters
        )
    }

    // MARK: - Power Conversion

    public fun convertPOWER(raw: PowerData): PowerDataSi {
        val voltage = (raw.voltageRaw.toDouble() / 65535.0) * 10.0
        val current = (raw.currentRaw.toDouble() / 32767.0) * 10000.0
        val soc = (raw.socRaw.toDouble() * 150.0 / 32767.0) - 25.0

        return PowerDataSi(
            timeUs = raw.timeUs,
            voltage = voltage,  // V
            current = current,  // mA
            soc = soc,          // %
        )
    }

    // MARK: - BMP585 Conversion

    public fun convertBMP585(raw: Bmp585Data): Bmp585DataSi {
        val temperature = raw.tempQ16.toDouble() / 65536.0  // °C
        val pressure = raw.pressQ6.toDouble() / 64.0        // Pa

        return Bmp585DataSi(
            timeUs = raw.timeUs,
            temperature = temperature,
            pressure = pressure,
        )
    }

    // MARK: - ISM6HG256 Conversion

    public fun convertISM6HG256(raw: Ism6hg256Data): Ism6Hg256DataSi {
        val c = cos(ism6RotZRad)
        val s = sin(ism6RotZRad)

        // Low-g accel (m/s²), then rotate about +Z to board frame
        val lgX = raw.accLowX.toDouble() * accLowMs2PerLsb
        val lgY = raw.accLowY.toDouble() * accLowMs2PerLsb
        val lowGAccX = (lgX * c) - (lgY * s)
        val lowGAccY = (lgX * s) + (lgY * c)
        val lowGAccZ = raw.accLowZ.toDouble() * accLowMs2PerLsb

        // High-g accel (m/s²), then rotate about +Z to board frame
        val hgX = raw.accHighX.toDouble() * accHighMs2PerLsb
        val hgY = raw.accHighY.toDouble() * accHighMs2PerLsb
        val highGAccX = (hgX * c) - (hgY * s)
        val highGAccY = (hgX * s) + (hgY * c)
        val highGAccZ = raw.accHighZ.toDouble() * accHighMs2PerLsb

        // Gyro (deg/s), then rotate about +Z to board frame
        val gX = raw.gyroX.toDouble() * gyroDpsPerLsb
        val gY = raw.gyroY.toDouble() * gyroDpsPerLsb
        val gyroX = (gX * c) - (gY * s)
        val gyroY = (gX * s) + (gY * c)
        val gyroZ = raw.gyroZ.toDouble() * gyroDpsPerLsb

        return Ism6Hg256DataSi(
            timeUs = raw.timeUs,
            lowGAccX = lowGAccX,
            lowGAccY = lowGAccY,
            lowGAccZ = lowGAccZ,
            highGAccX = highGAccX,
            highGAccY = highGAccY,
            highGAccZ = highGAccZ,
            gyroX = gyroX,
            gyroY = gyroY,
            gyroZ = gyroZ,
        )
    }

    // MARK: - MMC5983MA Conversion

    public fun convertMMC5983MA(raw: Mmc5983Data): Mmc5983MaDataSi {
        // Center the raw counts (18-bit value centered at 2^17 = 131072)
        val cx = (raw.magX and 0x3FFFFL) - 131072L
        val cy = (raw.magY and 0x3FFFFL) - 131072L
        val cz = (raw.magZ and 0x3FFFFL) - 131072L

        // Gauss = centered * (8 / 131072)
        // SI: μT = Gauss * 100
        val utPerCount = (8.0 * 100.0) / 131072.0  // 0.006103515625

        val mx = cx.toDouble() * utPerCount
        val my = cy.toDouble() * utPerCount
        val mz = cz.toDouble() * utPerCount

        val c = cos(mmcRotZRad)
        val s = sin(mmcRotZRad)

        // Rotate sensor frame -> board frame about +Z axis
        val magX = (mx * c) - (my * s)
        val magY = (mx * s) + (my * c)
        val magZ = mz

        return Mmc5983MaDataSi(
            timeUs = raw.timeUs,
            magX = magX,  // μT
            magY = magY,
            magZ = magZ,
        )
    }

    // MARK: - IIS2MDC Conversion (new Mini PCB rev)

    /**
     * Convert IIS2MDC raw counts to [Mmc5983MaDataSi] (µT) so CSV writer
     * stays unified between MMC and IIS2MDC boards.  IIS2MDC sensitivity
     * is 0.15 µT/LSB (datasheet 9.13).  Applies the IIS2MDC-specific
     * sensor→board rotation (iis2mdcRotZRad, from status query
     * iis2mdc_rot_z_cdeg, format_version >= 4 — #204), NOT the MMC's.
     */
    public fun convertIIS2MDC(raw: Iis2mdcData): Mmc5983MaDataSi {
        val utPerLsb = 0.15

        val mx = raw.magX.toDouble() * utPerLsb
        val my = raw.magY.toDouble() * utPerLsb
        val mz = raw.magZ.toDouble() * utPerLsb

        val c = cos(iis2mdcRotZRad)
        val s = sin(iis2mdcRotZRad)

        // Sensor frame -> board frame, rotation about +Z.
        val magX = (mx * c) - (my * s)
        val magY = (mx * s) + (my * c)
        val magZ = mz

        return Mmc5983MaDataSi(
            timeUs = raw.timeUs,
            magX = magX,
            magY = magY,
            magZ = magZ,
        )
    }

    // MARK: - Legacy Sensor Conversions
    // Legacy uses different sensors (ICM45686, H3LIS331, MS5611, LIS3MDL).
    // All converters output the same SI types as Mini so CSVGenerator stays unified.

    // ICM45686 ±32g accel, ±4000 dps gyro (low-G only — high-G from separate H3LIS331)
    private val icm45686AccMs2PerLsb: Double = 0.00059855
    private val icm45686GyroDpsPerLsb: Double = 0.00762777

    // H3LIS331 ±200g high-G accelerometer
    private val h3lis331AccMs2PerLsb: Double = 0.95788

    // LIS3MDL magnetometer
    private val lis3mdlUtPerLsb: Double = 0.014

    // Legacy rotation: 180° by default (raw data logged, rotate in app)
    private var legacyRotC: Double = -1.0  // cos(180°)
    private var legacyRotS: Double = 0.0   // sin(180°)

    /**
     * Convert Legacy ICM45686 → [Ism6Hg256DataSi] (high-G fields = 0; they come from H3LIS331)
     * Legacy data is raw (unrotated). Apply 180° rotation to body frame.
     */
    private var legacyDebugCount: Int = 0

    public fun convertLegacyICM45686(raw: LegacyIcm45686Data): Ism6Hg256DataSi {
        // Low-G accel (m/s²) — scale then rotate to body frame
        val ax = raw.accX.toDouble() * icm45686AccMs2PerLsb
        val ay = raw.accY.toDouble() * icm45686AccMs2PerLsb
        val lowGX = (ax * legacyRotC) - (ay * legacyRotS)
        val lowGY = (ax * legacyRotS) + (ay * legacyRotC)
        val lowGZ = raw.accZ.toDouble() * icm45686AccMs2PerLsb

        // Debug: print first 5 samples to trace the transform
        if (legacyDebugCount < 5) {
            println(
                "[LegacyICM] raw_acc_x=${raw.accX} rot_c=$legacyRotC rot_s=$legacyRotS " +
                    "→ low_g_x=${String.format(java.util.Locale.ROOT, "%.4f", lowGX)}",
            )
            legacyDebugCount += 1
        }

        // Gyro (deg/s) — scale then rotate to body frame
        val gx = raw.gyroX.toDouble() * icm45686GyroDpsPerLsb
        val gy = raw.gyroY.toDouble() * icm45686GyroDpsPerLsb
        val gyroX = (gx * legacyRotC) - (gy * legacyRotS)
        val gyroY = (gx * legacyRotS) + (gy * legacyRotC)
        val gyroZ = raw.gyroZ.toDouble() * icm45686GyroDpsPerLsb

        return Ism6Hg256DataSi(
            timeUs = raw.timeUs,
            lowGAccX = lowGX,
            lowGAccY = lowGY,
            lowGAccZ = lowGZ,
            highGAccX = 0.0,  // Comes from separate H3LIS331 sensor
            highGAccY = 0.0,
            highGAccZ = 0.0,
            gyroX = gyroX,
            gyroY = gyroY,
            gyroZ = gyroZ,
        )
    }

    /** Legacy H3LIS331 high-G acceleration in SI units (m/s²). */
    public data class HighGDataSi(
        val timeUs: Long,
        val accX: Double,  // m/s²
        val accY: Double,
        val accZ: Double,
    )

    public fun convertLegacyH3LIS331(raw: LegacyH3lis331Data): HighGDataSi {
        // High-G accel (m/s²) — scale then rotate to body frame
        val ax = raw.accX.toDouble() * h3lis331AccMs2PerLsb
        val ay = raw.accY.toDouble() * h3lis331AccMs2PerLsb
        val hgX = (ax * legacyRotC) - (ay * legacyRotS)
        val hgY = (ax * legacyRotS) + (ay * legacyRotC)
        val hgZ = raw.accZ.toDouble() * h3lis331AccMs2PerLsb

        return HighGDataSi(
            timeUs = raw.timeUs,
            accX = hgX,
            accY = hgY,
            accZ = hgZ,
        )
    }

    /** Convert Legacy MS5611 → [Bmp585DataSi] */
    public fun convertLegacyMS5611(raw: LegacyMs5611Data): Bmp585DataSi {
        // Pressure: hPa = 10 + code * 1190 / 4294967295, then convert to Pa
        val pressureHpa = 10.0 + (raw.pressure.toDouble() * 1190.0 / 4294967295.0)
        val pressurePa = pressureHpa * 100.0

        // Temperature: °C = ((raw + 32768) / 65535) * 100 - 40
        val temperature = ((raw.temperature.toDouble() + 32768.0) / 65535.0) * 100.0 - 40.0

        return Bmp585DataSi(
            timeUs = raw.timeUs,
            temperature = temperature,
            pressure = pressurePa,
        )
    }

    /**
     * Convert Legacy LIS3MDL → [Mmc5983MaDataSi]
     * Legacy data is raw (unrotated). Apply 180° rotation to body frame.
     */
    public fun convertLegacyLIS3MDL(raw: LegacyLis3mdlData): Mmc5983MaDataSi {
        // Raw LSB to μT — scale then rotate to body frame
        val mx = raw.magX.toDouble() * lis3mdlUtPerLsb
        val my = raw.magY.toDouble() * lis3mdlUtPerLsb
        val magX = (mx * legacyRotC) - (my * legacyRotS)
        val magY = (mx * legacyRotS) + (my * legacyRotC)
        val magZ = raw.magZ.toDouble() * lis3mdlUtPerLsb

        return Mmc5983MaDataSi(
            timeUs = raw.timeUs,
            magX = magX,
            magY = magY,
            magZ = magZ,
        )
    }

    /** Convert Legacy NonSensorData → [NonSensorDataSi] */
    public fun convertLegacyNonSensor(raw: LegacyNonSensorData): NonSensorDataSi {
        // Roll/pitch/yaw are already in degrees (float), not centidegrees
        val roll = raw.roll.toDouble()
        val pitch = raw.pitch.toDouble()
        val yaw = raw.yaw.toDouble()
        val rollCmd = raw.rollCmd.toDouble()

        // cm → m
        val ePos = raw.ePos.toDouble() * 0.01
        val nPos = raw.nPos.toDouble() * 0.01
        val uPos = raw.uPos.toDouble() * 0.01

        // cm/s → m/s
        val eVel = raw.eVel.toDouble() * 0.01
        val nVel = raw.nVel.toDouble() * 0.01
        val uVel = raw.uVel.toDouble() * 0.01

        // altitude_rate is in dm/s (decimeters per second) → m/s
        val altitudeRate = raw.altitudeRate.toDouble() * 0.1

        val rocketState = RocketState.fromRaw(raw.rocketState) ?: RocketState.INITIALIZATION

        return NonSensorDataSi(
            timeUs = raw.timeUs,
            // #514: the legacy wire format stores roll/pitch/yaw directly and carries
            // NO quaternion. Emit a zero (non-unit) quaternion as an explicit "absent"
            // marker rather than fabricating one — the CSV writes blanks for it.
            q0 = 0.0, q1 = 0.0, q2 = 0.0, q3 = 0.0,
            roll = roll,
            pitch = pitch,
            yaw = yaw,
            rollCmd = rollCmd,
            ePos = ePos,
            nPos = nPos,
            uPos = uPos,
            eVel = eVel,
            nVel = nVel,
            uVel = uVel,
            altitudeRate = altitudeRate,
            altLandedFlag = raw.altLandedFlag,
            altApogeeFlag = raw.altApogeeFlag,
            velUApogeeFlag = raw.velUApogeeFlag,
            launchFlag = raw.launchFlag,
            // Legacy wire format never carried NSF_BURNOUT or the extra
            // apogee detectors. Sidecar burnout_time_s will be null on
            // these flights (#196).
            burnoutFlag = false,
            gpsApogeeFlag = false,
            pitchApogeeFlag = false,
            apogeeFlag = false,
            rocketState = rocketState,
            // Legacy wire format never carried pyro_status — leave unset.
            pyro1Continuity = false,
            pyro2Continuity = false,
            pyro3Continuity = false,
            pyro4Continuity = false,
            pyro1Fired = false,
            pyro2Fired = false,
            pyro3Fired = false,
            pyro4Fired = false,
            rebootRecovery = false,
            guidanceEnabled = false,
            deployedFlag = false,
            ekfTicks = null,
        )
    }

    // MARK: - Mini NonSensor Conversion

    public fun convertNonSensor(raw: NonSensorData): NonSensorDataSi {
        // Quaternion int16*10000 -> float, then derive angles.
        val q0 = raw.q0.toDouble() / 10000.0
        val q1 = raw.q1.toDouble() / 10000.0
        val q2 = raw.q2.toDouble() / 10000.0
        val q3 = raw.q3.toDouble() / 10000.0
        // Roll: body-Z definition matching the FC roll controller
        // (flight_computer/main.cpp: actual_roll_deg = -atan2(z_east, z_north)).
        // Stays well-defined when the rocket is vertical, unlike the ZYX-Euler
        // roll which gimbal-locks and cycles near pitch ±90°. Keep in sync with
        // TelemetryData.roll.
        val zNorth = 2 * (q1 * q3 + q0 * q2)
        val zEast = 2 * (q2 * q3 - q0 * q1)
        val roll = -atan2(zEast, zNorth) * 180.0 / PI
        // Pitch/yaw stay ZYX-Euler: asin pitch is robust; yaw is the heading and
        // is inherently undefined when vertical (no formula avoids that).
        val pitch = asin((2 * (q0 * q2 - q3 * q1)).coerceIn(-1.0, 1.0)) * 180.0 / PI
        val yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * 180.0 / PI
        val rollCmd = raw.rollCmd.toDouble() / 100.0

        // Centimeters -> meters
        val ePos = raw.ePos.toDouble() * 0.01
        val nPos = raw.nPos.toDouble() * 0.01
        val uPos = raw.uPos.toDouble() * 0.01

        // cm/s -> m/s
        val eVel = raw.eVel.toDouble() * 0.01
        val nVel = raw.nVel.toDouble() * 0.01
        val uVel = raw.uVel.toDouble() * 0.01

        // Extract flags (bitfield).  Mirror the C++ NSF_* masks in
        // tinkerrocket-idf/components/TR_RocketComputerTypes/RocketComputerTypes.h.
        val nsfAltLanded = 1 shl 0
        val nsfAltApogee = 1 shl 1
        val nsfVelApogee = 1 shl 2
        val nsfLaunch = 1 shl 3
        val nsfBurnout = 1 shl 4   // #196: source of truth for burnout_time_s

        val altLandedFlag = (raw.flags and nsfAltLanded) != 0
        val altApogeeFlag = (raw.flags and nsfAltApogee) != 0
        val velUApogeeFlag = (raw.flags and nsfVelApogee) != 0
        val launchFlag = (raw.flags and nsfLaunch) != 0
        val burnoutFlag = (raw.flags and nsfBurnout) != 0

        // Apogee detector outputs + master vote + relocated reboot/guidance bits.
        val nsf2GpsApogee = 1 shl 0
        val nsf2PitchApogee = 1 shl 1
        val nsf2MasterApogee = 1 shl 2
        val nsf2RebootRecovery = 1 shl 3
        val nsf2GuidanceEnabled = 1 shl 4
        val nsf2Deployed = 1 shl 7
        val gpsApogeeFlag = (raw.apogeeFlags and nsf2GpsApogee) != 0
        val pitchApogeeFlag = (raw.apogeeFlags and nsf2PitchApogee) != 0
        val apogeeFlag = (raw.apogeeFlags and nsf2MasterApogee) != 0
        val rebootRecovery = (raw.apogeeFlags and nsf2RebootRecovery) != 0
        val guidanceEnabled = (raw.apogeeFlags and nsf2GuidanceEnabled) != 0
        val deployedFlag = (raw.apogeeFlags and nsf2Deployed) != 0

        val rocketState = RocketState.fromRaw(raw.rocketState) ?: RocketState.INITIALIZATION

        // dm/s -> m/s
        val altitudeRate = raw.baroAltRateDmps.toDouble() * 0.1

        // Pyro status byte — 4 channels × (cont, fired). Mirror C++ PSF_* masks.
        val psfCh1Cont = 1 shl 0
        val psfCh1Fired = 1 shl 1
        val psfCh2Cont = 1 shl 2
        val psfCh2Fired = 1 shl 3
        val psfCh3Cont = 1 shl 4
        val psfCh3Fired = 1 shl 5
        val psfCh4Cont = 1 shl 6
        val psfCh4Fired = 1 shl 7

        return NonSensorDataSi(
            timeUs = raw.timeUs,
            q0 = q0, q1 = q1, q2 = q2, q3 = q3,   // #514: carry the raw quaternion through
            roll = roll,
            pitch = pitch,
            yaw = yaw,
            rollCmd = rollCmd,
            ePos = ePos,
            nPos = nPos,
            uPos = uPos,
            eVel = eVel,
            nVel = nVel,
            uVel = uVel,
            altitudeRate = altitudeRate,
            altLandedFlag = altLandedFlag,
            altApogeeFlag = altApogeeFlag,
            velUApogeeFlag = velUApogeeFlag,
            launchFlag = launchFlag,
            burnoutFlag = burnoutFlag,
            gpsApogeeFlag = gpsApogeeFlag,
            pitchApogeeFlag = pitchApogeeFlag,
            apogeeFlag = apogeeFlag,
            rocketState = rocketState,
            pyro1Continuity = (raw.pyroStatus and psfCh1Cont) != 0,
            pyro2Continuity = (raw.pyroStatus and psfCh2Cont) != 0,
            pyro3Continuity = (raw.pyroStatus and psfCh3Cont) != 0,
            pyro4Continuity = (raw.pyroStatus and psfCh4Cont) != 0,
            pyro1Fired = (raw.pyroStatus and psfCh1Fired) != 0,
            pyro2Fired = (raw.pyroStatus and psfCh2Fired) != 0,
            pyro3Fired = (raw.pyroStatus and psfCh3Fired) != 0,
            pyro4Fired = (raw.pyroStatus and psfCh4Fired) != 0,
            rebootRecovery = rebootRecovery,
            guidanceEnabled = guidanceEnabled,
            deployedFlag = deployedFlag,
            ekfTicks = raw.ekfTicks,   // #529
        )
    }
}
